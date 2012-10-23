/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemTreeView.h"
#include "Item.h"
#include "RootItem.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "Archive.h"
#include "TreeWidget.h"
#include "ConnectionSet.h"
#include <QBoxLayout>
#include <QApplication>
#include <QMouseEvent>
#include <QHeaderView>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cassert>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    // Item of the item tree view
    class ItvItem : public QTreeWidgetItem
    {
    public:
        ItvItem(Item* item, ItemTreeViewImpl* itemTreeViewImpl);
        virtual ~ItvItem();
        virtual QVariant data(int column, int role) const;
        virtual void setData(int column, int role, const QVariant& value);
        ItemPtr item;
        boost::signal<void(bool isChecked)> sigCheckToggled;
        ItemTreeViewImpl* itemTreeViewImpl;
        bool isExpandedBeforeRemoving;
    };

    // Preserved as custom data in an item
    class ItvItemRef : public Referenced
    {
    public:
        ItvItemRef(ItvItem* itvItem) : itvItem(itvItem) { }
        ItvItem* itvItem;
    };
}


namespace cnoid {

    class ItemTreeViewImpl : public TreeWidget, public boost::signals::trackable
    {
    public:

        ItemTreeViewImpl(ItemTreeView* self, RootItem* rootItem, bool showRoot);
        ~ItemTreeViewImpl();

        ItemTreeView* self;
        RootItemPtr rootItem;

        int isProceccingSlotForRootItemSignals;
        ConnectionSet connectionsFromRootItem;

        boost::signal<void(Item* item, bool isChecked)> sigCheckToggled;
        boost::signal<void(bool isChecked)> sigCheckToggledForInvalidItem;
        boost::signal<void(const ItemList<Item>&)> sigSelectionChanged;
        boost::signal<void(const ItemList<Item>&)> sigSelectionOrTreeChanged;

        ItemList<Item> selectedItemList;
        ItemList<Item> copiedItemList;
        ItemList<Item> checkedItemList;
        bool needToUpdateCheckedItemList;
        Menu* popupMenu;
        MenuManager menuManager;

        bool isDropping;

        virtual void mousePressEvent(QMouseEvent* event);
        ItvItem* getItvItem(Item* item);
        ItvItem* getOrCreateItvItem(Item* item);
        void onItemAdded(Item* item);
        void addItem(QTreeWidgetItem* parentTwItem, Item* item);
        void onItemRemoved(Item* item, bool isMoving);
        void onTreeChanged();

        virtual void dropEvent(QDropEvent* event);
        

        void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
        void onRowsInserted(const QModelIndex& parent, int start, int end);
        virtual bool dropMimeData(QTreeWidgetItem* parent, int index, const QMimeData* data, Qt::DropAction action);
        void onSelectionChanged();
        bool isItemSelected(Item* item);
        bool selectItem(Item* item, bool select);
        bool isItemChecked(Item* item);
        bool checkItem(Item* item, bool checked);
        void extractSelectedItemsOfSubTreeTraverse(Item* item, ItemListBase* io_items);
        ItemList<Item>& checkedItems();
        void extractCheckedItems(QTreeWidgetItem* twItem);
        void forEachTopItems(const ItemList<Item>& orgItemList, function<void(Item*)> callback);
        void forEachTopItemsSub(ItemPtr item, ItemList<Item>& items);
        void cutSelectedItems();
        void copySelectedItems();
        ItemPtr duplicateItemsInGivenList(ItemPtr item, ItemPtr duplicated, ItemList<Item>& items);
        void copySelectedItemsWithChildren();
        void addCopiedItemToCopiedItemList(Item* item);
        void pasteItems();
        void moveCutItemsToCopiedItemList(Item* item);
        bool storeState(Archive& archive);
        void storeItemIds(Archive& archive, const char* key, const ItemList<Item>& items);
        bool restoreState(const Archive& archive);
        void restoreItemStates(const Archive& archive, const char* key, function<void(ItemPtr)> stateChangeFunc);
        void storeExpandedItems(Archive& archive);
        void storeExpandedItemsSub(QTreeWidgetItem* parentTwItem, Archive& archive, YamlSequencePtr& expanded);
        void restoreExpandedItems(const Archive& archive);
    };
}


ItvItem::ItvItem(Item* item, ItemTreeViewImpl* itemTreeViewImpl)
    : item(item),
      itemTreeViewImpl(itemTreeViewImpl)
{
    
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);

    if(!item->isSubItem()){
        setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
    }

    setCheckState(1, Qt::Unchecked);
    
    ItvItemRef* ref = new ItvItemRef(this);
    item->setCustomData(0, ref);

    isExpandedBeforeRemoving = false;
}


ItvItem::~ItvItem()
{
    item->clearCustomData(0);
}


QVariant ItvItem::data(int column, int role) const
{
    if((role == Qt::DisplayRole || role == Qt::EditRole) && column == 0){
        return item->name().c_str();
    } else {
        return QTreeWidgetItem::data(column, role);
    }
}


void ItvItem::setData(int column, int role, const QVariant& value)
{
    bool emitCheckToggled = false;
    
    if(column == 0){
        if(role == Qt::DisplayRole || role == Qt::EditRole){
            if(value.type() == QVariant::String){
                if(!value.toString().isEmpty()){
                    item->setName(value.toString().toStdString());
                }
            }
        }
    } else if(column == 1 && role == Qt::CheckStateRole){
        emitCheckToggled = true;
    }
    
    QTreeWidgetItem::setData(column, role, value);
    
    if(emitCheckToggled){
        itemTreeViewImpl->needToUpdateCheckedItemList = true;
        bool checked = ((Qt::CheckState)value.toInt() == Qt::Checked);
        itemTreeViewImpl->sigCheckToggled(item.get(), checked);
        sigCheckToggled(checked);
    }
}


void ItemTreeView::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addView(mainInstance());
        initialized = true;
    }
}


ItemTreeView* ItemTreeView::mainInstance()
{
    static ItemTreeView* itemTreeView = new ItemTreeView(RootItem::mainInstance());
    return itemTreeView;
}


ItemTreeView::ItemTreeView(RootItem* rootItem, bool showRoot)
{
    setName(N_("Items"));
    setDefaultLayoutArea(View::LEFT);
    
    impl = new ItemTreeViewImpl(this, rootItem, showRoot);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(impl);
    setLayout(layout);
}


ItemTreeViewImpl::ItemTreeViewImpl(ItemTreeView* self, RootItem* rootItem, bool showRoot)
    : self(self),
      rootItem(rootItem)
{
    isProceccingSlotForRootItemSignals = 0;
    needToUpdateCheckedItemList = false;
    isDropping = false;

    setColumnCount(2);
    header()->setStretchLastSection(false);
    header()->setResizeMode(0, QHeaderView::Stretch);
    header()->setResizeMode(1, QHeaderView::ResizeToContents);
    header()->swapSections(0, 1);
    setWordWrap(true);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setIndentation(12);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setDragDropMode(QAbstractItemView::InternalMove);

    sigItemSelectionChanged().connect(bind(&ItemTreeViewImpl::onSelectionChanged, this));

    connectionsFromRootItem.add(
        rootItem->sigItemAdded().connect(bind(&ItemTreeViewImpl::onItemAdded, this, _1)));
    connectionsFromRootItem.add(
        rootItem->sigItemRemoved().connect(bind(&ItemTreeViewImpl::onItemRemoved, this, _1, _2)));
    connectionsFromRootItem.add(
        rootItem->sigTreeChanged().connect(bind(&ItemTreeViewImpl::onTreeChanged, this)));

    QObject::connect(model(), SIGNAL(rowsAboutToBeRemoved(const QModelIndex&, int, int)),
                     self, SLOT(onRowsAboutToBeRemoved(const QModelIndex&, int, int)));
    QObject::connect(model(), SIGNAL(rowsInserted(const QModelIndex&, int, int)),
                     self, SLOT(onRowsInserted(const QModelIndex&, int, int)));

    popupMenu = new Menu(this);
    menuManager.setTopMenu(popupMenu);
    menuManager.addItem(_("Cut"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::cutSelectedItems, this));
    menuManager.addItem(_("Copy"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::copySelectedItems, this));
    menuManager.addItem(_("Copy with Children"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::copySelectedItemsWithChildren, this));
    menuManager.addItem(_("Paste"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::pasteItems, this));
}


ItemTreeView::~ItemTreeView()
{

}


ItemTreeViewImpl::~ItemTreeViewImpl()
{
    // On Windows + VC++, boost::signal::trackable, the super class of
    // ItemTreeViewImpl, does not seem to work correctly, and the connection
    // is not disconnected and the program aborted by segmentation fault.
    // So the following explicit disconnection code is added.

    //connectionClassUnregistered.disconnect();
}


RootItem* ItemTreeView::rootItem()
{
    return impl->rootItem.get();
}


void ItemTreeView::showRoot(bool show)
{

}


void ItemTreeViewImpl::mousePressEvent(QMouseEvent* event)
{
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        popupMenu->popup(event->globalPos());
    }
}


ItvItem* ItemTreeViewImpl::getItvItem(Item* item)
{
    ItvItem* itvItem = 0;
    ItvItemRef* ref = dynamic_cast<ItvItemRef*>(item->customData(0));
    if(ref){
        itvItem = ref->itvItem;
    }
    return itvItem;
}


ItvItem* ItemTreeViewImpl::getOrCreateItvItem(Item* item)
{
    ItvItem* itvItem = getItvItem(item);
    if(!itvItem){
        itvItem = new ItvItem(item, this);
    }
    return itvItem;
}


void ItemTreeViewImpl::onItemAdded(Item* item)
{
    isProceccingSlotForRootItemSignals++;
    
    Item* parentItem = item->parentItem();
    if(parentItem){
        if(parentItem == rootItem){
            addItem(invisibleRootItem(), item);
        } else {
            ItvItem* parentItvItem = getItvItem(parentItem);
            if(parentItvItem){
                addItem(parentItvItem, item);
            }
        }
    }

    isProceccingSlotForRootItemSignals--;
}


void ItemTreeViewImpl::addItem(QTreeWidgetItem* parentTwItem, Item* item)
{
    ItvItem* itvItem = getOrCreateItvItem(item);
    parentTwItem->addChild(itvItem);

    if(!parentTwItem->isExpanded()){
        if(!item->isSubItem()){
            parentTwItem->setExpanded(true);
        }
    }

    for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        addItem(itvItem, childItem);
    }
}


void ItemTreeViewImpl::onItemRemoved(Item* item, bool isMoving)
{
    isProceccingSlotForRootItemSignals++;
    
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        QTreeWidgetItem* parentTwItem = itvItem->parent();
        if(parentTwItem){
            parentTwItem->removeChild(itvItem);
        } else {
            takeTopLevelItem(indexOfTopLevelItem(itvItem));
        }
        delete itvItem;
    }

    isProceccingSlotForRootItemSignals--;
}


void ItemTreeViewImpl::dropEvent(QDropEvent* event)
{
    isDropping = true;
    TreeWidget::dropEvent(event);
    isDropping = false;
}
    

void ItemTreeView::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(impl->isProceccingSlotForRootItemSignals == 0){
        impl->onRowsAboutToBeRemoved(parent, start, end);
    }
}


void ItemTreeViewImpl::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    connectionsFromRootItem.block();

    QTreeWidgetItem* parentTwItem = itemFromIndex(parent);
    if(!parentTwItem){
        parentTwItem = invisibleRootItem();
    }

    for(int i=start; i <= end; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            itvItem->isExpandedBeforeRemoving = itvItem->isExpanded();
            if(!isDropping){
                ItemPtr& item = itvItem->item;
                if(!item->isSubItem()){
                    item->detachFromParentItem();
                }
            }
        }
    }
    
    connectionsFromRootItem.unblock();
}


void ItemTreeView::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(impl->isProceccingSlotForRootItemSignals == 0){
        impl->onRowsInserted(parent, start, end);
    }
}


void ItemTreeViewImpl::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    connectionsFromRootItem.block();

    QTreeWidgetItem* parentTwItem = itemFromIndex(parent);
    if(!parentTwItem){
        parentTwItem = invisibleRootItem();
    }

    ItvItem* parentItvItem = dynamic_cast<ItvItem*>(parentTwItem);
    Item* parentItem = parentItvItem ? parentItvItem->item.get() : rootItem.get();

    ItemPtr nextItem = 0;
    if(end + 1 < parentTwItem->childCount()){
        ItvItem* nextItvItem = dynamic_cast<ItvItem*>(parentTwItem->child(end + 1));
        if(nextItvItem){
            nextItem = nextItvItem->item;
        }
    }
    
    for(int i=start; i <= end; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            ItemPtr& item = itvItem->item;
            if(!item->isSubItem()){
                parentItem->insertChildItem(item, nextItem);
            }
            if(itvItem->isExpandedBeforeRemoving){
                itvItem->setExpanded(true);
            }
        }
    }
    
    connectionsFromRootItem.unblock();
}


void ItemTreeViewImpl::onTreeChanged()
{

}


bool ItemTreeViewImpl::dropMimeData(QTreeWidgetItem* parent, int index, const QMimeData* data, Qt::DropAction action)
{
	return TreeWidget::dropMimeData(parent, index, data, action);
}


void ItemTreeViewImpl::onSelectionChanged()
{
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = selectedItems();
    for(int i=0; i < selected.size(); ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(selected[i]);
        if(itvItem){
            selectedItemList.append(itvItem->item);
        }
    }

    sigSelectionChanged(selectedItemList);
    sigSelectionOrTreeChanged(selectedItemList);
}


bool ItemTreeView::isItemSelected(ItemPtr item)
{
    return impl->isItemSelected(item.get());
}


bool ItemTreeViewImpl::isItemSelected(Item* item)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        return itvItem->isSelected();
    }
    return false;
}


bool ItemTreeView::selectItem(ItemPtr item, bool select)
{
    return impl->selectItem(item.get(), select);
}


bool ItemTreeViewImpl::selectItem(Item* item, bool select)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        QModelIndex index = indexFromItem(itvItem);
        selectionModel()->select(index, (select ? QItemSelectionModel::Select : QItemSelectionModel::Deselect));
        return select;
    }
    return false;
}


void ItemTreeView::clearSelection()
{
    impl->selectionModel()->clearSelection();
}


bool ItemTreeView::isItemChecked(ItemPtr item)
{
    return impl->isItemChecked(item.get());
}


bool ItemTreeViewImpl::isItemChecked(Item* item)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        return (itvItem->checkState(1) == Qt::Checked);
    }
    return false;
}
    

bool ItemTreeView::checkItem(ItemPtr item, bool checked)
{
    return impl->checkItem(item.get(), checked);
}


bool ItemTreeViewImpl::checkItem(Item* item, bool checked)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        itvItem->setCheckState(1, (checked ? Qt::Checked : Qt::Unchecked));
        return checked;
    }
    return false;
}


SignalProxy< boost::signal<void(const ItemList<Item>&)> > ItemTreeView::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy< boost::signal<void(const ItemList<Item>&)> > ItemTreeView::sigSelectionOrTreeChanged()
{
    return impl->sigSelectionOrTreeChanged;
}


SignalProxy< boost::signal<void(Item* item, bool isChecked)> > ItemTreeView::sigCheckToggled()
{
    return impl->sigCheckToggled;
}


SignalProxy< boost::signal<void(bool isChecked)> > ItemTreeView::sigCheckToggled(Item* targetItem)
{
    return impl->getOrCreateItvItem(targetItem)->sigCheckToggled;
}


void ItemTreeView::getSelectedItems(ItemListBase& items)
{
    items = impl->selectedItemList;
}


void ItemTreeView::extractSelectedItemsOfSubTree(ItemPtr topItem, ItemListBase& io_items)
{
    topItem->traverse(bind(&ItemTreeViewImpl::extractSelectedItemsOfSubTreeTraverse, impl, _1, &io_items));
}


void ItemTreeViewImpl::extractSelectedItemsOfSubTreeTraverse(Item* item, ItemListBase* io_items)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem && itvItem->isSelected()){
        io_items->appendIfTypeMatches(item);
    }
}


void ItemTreeView::getCheckedItems(ItemListBase& items)
{
    items = impl->checkedItems();
}


ItemList<Item>& ItemTreeViewImpl::checkedItems()
{
    if(needToUpdateCheckedItemList){
        checkedItemList.clear();
        extractCheckedItems(invisibleRootItem());
        needToUpdateCheckedItemList = false;
    }
    return checkedItemList;
}


void ItemTreeViewImpl::extractCheckedItems(QTreeWidgetItem* twItem)
{
    ItvItem* itvItem = dynamic_cast<ItvItem*>(twItem);
    if(itvItem){
        if(itvItem->checkState(1) == Qt::Checked){
            checkedItemList.append(itvItem->item);
        }
    }
    int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        extractCheckedItems(twItem->child(i));
    }
}


void ItemTreeViewImpl::forEachTopItems(const ItemList<Item>& orgItemList, function<void(Item*)> callback)
{
    ItemList<Item> items(orgItemList);
    while(!items.empty()){
        ItemPtr item = items.front();
        items.pop_front();
        forEachTopItemsSub(item, items);
        callback(item.get());
    }
}


void ItemTreeViewImpl::forEachTopItemsSub(ItemPtr item, ItemList<Item>& items)
{
    for(ItemPtr childItem = item->childItem(); (childItem && !items.empty()); childItem = childItem->nextItem()){
        if(childItem == items.front()){
            items.pop_front();
        }
        forEachTopItemsSub(childItem, items);
    }
}


void ItemTreeViewImpl::cutSelectedItems()
{
    copiedItemList.clear();
    forEachTopItems(selectedItemList, bind(&ItemTreeViewImpl::moveCutItemsToCopiedItemList, this, _1));
}


void ItemTreeViewImpl::copySelectedItems()
{
    copiedItemList.clear();

    ItemList<Item> items(selectedItemList);
    while(!items.empty()){
        ItemPtr item = items.front();
        items.pop_front();
        ItemPtr duplicated = duplicateItemsInGivenList(item, 0, items);
        if(duplicated){
            copiedItemList.push_back(duplicated);
        }
    }
}


ItemPtr ItemTreeViewImpl::duplicateItemsInGivenList(ItemPtr item, ItemPtr duplicated, ItemList<Item>& items)
{
    if(!duplicated){
        duplicated = item->duplicate();
    }
    
    for(ItemPtr childItem = item->childItem(); (childItem && !items.empty()); childItem = childItem->nextItem()){
        if(childItem == items.front()){
            items.pop_front();
            ItemPtr duplicatedChild;
            if(childItem->isSubItem()){
                duplicatedChild = duplicated->findItem(childItem->name());
                if(duplicatedChild){
                    duplicateItemsInGivenList(childItem, duplicatedChild, items);
                }
            } else {
                duplicatedChild = duplicateItemsInGivenList(childItem, 0, items);
                if(duplicatedChild){
                    duplicated->addChildItem(duplicatedChild);
                }
            }
        }
    }

    return duplicated;
}


void ItemTreeViewImpl::copySelectedItemsWithChildren()
{
    copiedItemList.clear();
    forEachTopItems(selectedItemList, bind(&ItemTreeViewImpl::addCopiedItemToCopiedItemList, this, _1));
}


void ItemTreeViewImpl::addCopiedItemToCopiedItemList(Item* item)
{
    ItemPtr duplicated = item->duplicateAll();
    if(duplicated){
        copiedItemList.push_back(duplicated);
    }
}


void ItemTreeViewImpl::pasteItems()
{
    if(!copiedItemList.empty()){
        ItemPtr parentItem;
        if(selectedItemList.empty()){
            parentItem = rootItem;
        } else if(selectedItemList.size() == 1){
            parentItem = selectedItemList.front();
        }
        if(parentItem){
            for(size_t i=0; i < copiedItemList.size(); ++i){
                parentItem->addChildItem(copiedItemList[i]->duplicateAll());
            }
        }
    }
}



void ItemTreeViewImpl::moveCutItemsToCopiedItemList(Item* item)
{
    if(!item->isSubItem()){
        copiedItemList.push_back(item);
        item->detachFromParentItem();
    }
}


bool ItemTreeView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool ItemTreeViewImpl::storeState(Archive& archive)
{
    storeItemIds(archive, "selected", selectedItemList);
    storeItemIds(archive, "checked", checkedItems());
    storeExpandedItems(archive);
    return true;
}


void ItemTreeViewImpl::storeItemIds(Archive& archive, const char* key, const ItemList<Item>& items)
{
    YamlSequencePtr idseq = new YamlSequence();
    idseq->setFlowStyle(true);
    for(size_t i=0; i < items.size(); ++i){
        int id = archive.getItemId(items[i]);
        if(id >= 0){
            idseq->append(id);
        }
    }
    if(!idseq->empty()){
        archive.insert(key, idseq);
    }
}


bool ItemTreeView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool ItemTreeViewImpl::restoreState(const Archive& archive)
{
    restoreItemStates(archive, "checked", bind(&ItemTreeView::checkItem, self, _1, true));
    restoreItemStates(archive, "selected", bind(&ItemTreeView::selectItem, self, _1, true));
    restoreExpandedItems(archive);
    return true;
}


void ItemTreeViewImpl::restoreItemStates
(const Archive& archive, const char* key, function<void(ItemPtr)> stateChangeFunc)
{
    const YamlSequence& idseq = *archive.findSequence(key);
    if(idseq.isValid()){
        for(int i=0; i < idseq.size(); ++i){
            int id = idseq[i].toInt();
            if(id >= 0){
                ItemPtr item = archive.findItem(id);
                if(item){
                    stateChangeFunc(item);
                }
            }
        }
    }
}



void ItemTreeViewImpl::storeExpandedItems(Archive& archive)
{
    YamlSequencePtr expanded = new YamlSequence();
    expanded->setFlowStyle(true);
    storeExpandedItemsSub(invisibleRootItem(), archive, expanded);
    if(!expanded->empty()){
        archive.insert("expanded", expanded);
    }
}


void ItemTreeViewImpl::storeExpandedItemsSub(QTreeWidgetItem* parentTwItem, Archive& archive, YamlSequencePtr& expanded)
{
    int n = parentTwItem->childCount();
    for(int i=0; i < n; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            if(itvItem->isExpanded()){
                Item* item = itvItem->item.get();
                if(!item->isSubItem()){
                    int id = archive.getItemId(item);
                    if(id >= 0){
                        expanded->append(id);
                    }
                } else {
                    int j = 0;
                    Item* p = item;
                    while(p->isSubItem()){
                        ++j;
                        p = p->parentItem();
                    }
                    int id = archive.getItemId(p);
                    if(id >= 0){
                        YamlSequencePtr path = new YamlSequence(j+1);
                        path->setFlowStyle(true);
                        while(item->isSubItem()){
                            path->write(j--, item->name(), YAML_DOUBLE_QUOTED);
                            item = item->parentItem();
                        }
                        path->write(0, id);
                        expanded->append(path);
                    }
                }
            }
            if(itvItem->childCount() > 0){
                storeExpandedItemsSub(itvItem, archive, expanded);
            }
        }
    }
}


void ItemTreeViewImpl::restoreExpandedItems(const Archive& archive)
{
    const YamlSequence& expanded = *archive.findSequence("expanded");
    if(expanded.isValid()){
        collapseAll();
        for(int i=0; i < expanded.size(); ++i){
            Item* item = 0;
            if(!expanded[i].isSequence()){
                item = archive.findItem(expanded[i].toInt());
            } else {
                const YamlSequence& path = *expanded[i].toSequence();
                int n = path.size();
                if(n >= 2){
                    item = archive.findItem(path[0].toInt());
                    for(int j=1; item && j < n; ++j){
                        item = item->findItem(path[j].toString());
                    }
                }
            }
            if(item){
                ItvItem* itvItem = getItvItem(item);
                if(itvItem){
                    itvItem->setExpanded(true);
                }
            }
        }
    }
}
