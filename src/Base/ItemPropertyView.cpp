/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemPropertyView.h"
#include "ItemTreeView.h"
#include "Item.h"
#include "ItemList.h"
#include "ItemManager.h"
#include "MessageView.h"
#include "PutPropertyFunction.h"
#include "ConnectionSet.h"
#include <string>
#include <iostream>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <QTableWidget>
#include <QHeaderView>
#include <QBoxLayout>
#include <QItemDelegate>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {

    bool TRACE_FUNCTIONS = false;

    typedef variant<bool, int, double, string, Selection> ValueVariant;

    typedef variant<function<bool(bool)>, function<bool(int)>, function<bool(double)>,
                    function<bool(const string&)>, function<bool(int)> > FunctionVariant;

    enum TypeId { TYPE_BOOL, TYPE_INT, TYPE_DOUBLE, TYPE_STRING, TYPE_SELECTION };

    struct Property {
        Property(const string& name, ValueVariant value)
            : name(name), value(value), hasValidFunction(false) { }
        Property(const string& name, ValueVariant value, FunctionVariant func)
            : name(name), value(value), func(func), hasValidFunction(true) { }
        string name;
        ValueVariant value;
        FunctionVariant func;
        bool hasValidFunction;
    };
    typedef shared_ptr<Property> PropertyPtr;


    class PropertyItem : public QTableWidgetItem
    {
    public:
        PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value);
        PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func);

        virtual QVariant data(int role) const;
        virtual void setData(int role, const QVariant& qvalue);

        ItemPropertyViewImpl* itemPropertyViewImpl;
        ValueVariant value;
        FunctionVariant func;
        bool hasValidFunction;
    };


    class PropertyDelegate : public QItemDelegate
    {
    public:
        PropertyDelegate(QObject* parent);
        QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
        void setEditorData(QWidget* editor, const QModelIndex& index) const;
        void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const;
        void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    };
}


namespace cnoid {

    class ItemPropertyViewImpl : public PutPropertyFunction
    {
    public:
        ItemPropertyViewImpl(ItemPropertyView* self);
        ~ItemPropertyViewImpl();
        
        void updateProperties();
        void addProperty(const std::string& name, PropertyItem* propertyItem);
        void onItemSelectionChanged(const ItemList<Item>& items);

        ItemPropertyView* self;

        QTableWidget* tableWidget;

        ItemPtr currentItem;
        ConnectionSet itemConnections;
        int tmpListIndex;

        std::vector<PropertyPtr> properties;
        
        PropertyPtr editedProperty;

        bool isPressedPathValid;

        // PutPropertyFunction's virtual functions
        virtual void operator()(const std::string& name, bool value){
            addProperty(name, new PropertyItem(this, value));
        }
        virtual void operator()(const std::string& name, bool value, const boost::function<bool(bool)>& func) {
            addProperty(name, new PropertyItem(this, value, func));
        }
        virtual void operator()(const std::string& name, int value){
            addProperty(name, new PropertyItem(this, value));
        }
        virtual void operator()(const std::string& name, int value, const boost::function<bool(int)>& func){
            addProperty(name, new PropertyItem(this, value, func));
        }
        virtual void operator()(const std::string& name, double value){
            addProperty(name, new PropertyItem(this, value));
        }
        virtual void operator()(const std::string& name, double value, const boost::function<bool(double)>& func){
            addProperty(name, new PropertyItem(this, value, func));
        }
        virtual void operator()(const std::string& name, const std::string& value){
            addProperty(name, new PropertyItem(this, value));
        }
        virtual void operator()(const std::string& name, const std::string& value,
                                const boost::function<bool(const std::string&)>& func){
            addProperty(name, new PropertyItem(this, value, func));
        }
        void operator()(const std::string& name, Selection selection){
            addProperty(name, new PropertyItem(this, selection));
        }
        void operator()(const std::string& name, Selection selection,
                        const boost::function<bool(int which)>& func){
            addProperty(name, new PropertyItem(this, selection, func));
        }
    };
}



PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value)
    : itemPropertyViewImpl(viewImpl),
      value(value)
{
    setFlags(Qt::ItemIsEnabled);
    hasValidFunction = false;
}


PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func)
    : itemPropertyViewImpl(viewImpl),
      value(value),
      func(func)
{
    setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
    hasValidFunction = true;
}


QVariant PropertyItem::data(int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(value.which()){
        case TYPE_BOOL:      return get<bool>(value);
        case TYPE_INT:       return get<int>(value);
        case TYPE_DOUBLE:    return get<double>(value);
        case TYPE_STRING:    return get<string>(value).c_str();
        case TYPE_SELECTION: return get<Selection>(value).label().c_str();
        }
    }
    return QTableWidgetItem::data(role);
}


void PropertyItem::setData(int role, const QVariant& qvalue)
{
    bool accepted = false;
    if(role == Qt::EditRole){

        switch(qvalue.type()){

        case QVariant::Bool:
            accepted = get< function<bool(bool)> >(func)(qvalue.toBool());
            break;

        case QVariant::String:
            accepted = get< function<bool(const string&)> >(func)(qvalue.toString().toStdString());
            break;

        case QVariant::Int:
            accepted = get< function<bool(int)> >(func)(qvalue.toInt());
            break;
            
        case QVariant::Double:
            accepted = get< function<bool(double)> >(func)(qvalue.toDouble());
            break;

        default:
            break;
        }
        if(accepted){
            itemPropertyViewImpl->currentItem->notifyUpdate();
        }
        
    }
}


void ItemPropertyView::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addView(new ItemPropertyView());
        initialized = true;
    }
}


ItemPropertyView::ItemPropertyView()
{
    impl = new ItemPropertyViewImpl(this);
}


ItemPropertyViewImpl::ItemPropertyViewImpl(ItemPropertyView* self)
    : self(self)
{
    self->setName(N_("Property"));
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    isPressedPathValid = false;

    tableWidget = new QTableWidget(self);
    tableWidget->setFrameShape(QFrame::NoFrame);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    tableWidget->horizontalHeader()->hide();
    tableWidget->horizontalHeader()->setStretchLastSection(true);

    tableWidget->verticalHeader()->hide();
    tableWidget->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(tableWidget);
    self->setLayout(vbox);

    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
        bind(&ItemPropertyViewImpl::onItemSelectionChanged, this, _1));
}


ItemPropertyView::~ItemPropertyView()
{
    delete impl;
}


ItemPropertyViewImpl::~ItemPropertyViewImpl()
{
    itemConnections.disconnect();
}


void ItemPropertyViewImpl::updateProperties()
{
    tableWidget->setRowCount(0);

    if(currentItem){
        tmpListIndex = 0;
        properties.clear();
        currentItem->putProperties(*this);
    }
}


void ItemPropertyViewImpl::addProperty(const std::string& name, PropertyItem* propertyItem)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);
    
    tableWidget->setItem(row, 1, propertyItem);
}


void ItemPropertyViewImpl::onItemSelectionChanged(const ItemList<Item>& items)
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::onItemSelectionChanged()" << endl;
    }
    
    ItemPtr item = items.toSingle();

    if(item != currentItem){
        currentItem = item;
        itemConnections.disconnect();
        if(item){
            itemConnections.add(
                item->sigUpdated().connect(
                    bind(&ItemPropertyViewImpl::updateProperties, this)));
            itemConnections.add(
                item->sigNameChanged().connect(
                    bind(&ItemPropertyViewImpl::updateProperties, this)));
        }
        updateProperties();
    }
}



PropertyDelegate::PropertyDelegate(QObject* parent)
    : QItemDelegate(parent)
{

}


QWidget* PropertyDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	return 0;
}


void PropertyDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{

}


void PropertyDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{

}


void PropertyDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);
}
