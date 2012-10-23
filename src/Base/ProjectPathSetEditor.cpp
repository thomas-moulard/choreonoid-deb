/**
   @author Shin'ichiro Nakaoka
*/

#include "ProjectPathSetEditor.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "MenuManager.h"
#include "Button.h"
#include <QDialog>
#include <QBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTableWidget>
#include <QHeaderView>
#include <QDialogButtonBox>
#include <boost/bind.hpp>
#include <map>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    class PathSetItem : public QTableWidgetItem
    {
    public:
        QString paths;
        
        PathSetItem(const QString& paths)
            : paths(paths) {
            setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
        }

        virtual QVariant data(int role) const {

            if(role == Qt::DisplayRole || role == Qt::EditRole){
                return paths;
            }
            return QTableWidgetItem::data(role);
        }

        virtual void setData(int role, const QVariant& value) {
            bool accepted = false;
            if(role == Qt::EditRole && value.type() == QVariant::String){
                paths = value.toString();
            } else {
                QTableWidgetItem::setData(role, value);
            }
        }
    };
    

    class PathSetEditor : public QDialog
    {
    public:
        PathSetEditor();

        QTableWidget* tableWidget;
        QLineEdit* newVariableEntry;

        void initAndShow();
        void readPathSetFromArchive();
        void appendPathSet(const QString& name, const QString& paths);
        void writePathSetToArchive();
        void onAppendActivated();
    };

}


void ProjectPathSetEditor::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        PathSetEditor* editor = new PathSetEditor();
        MenuManager& mm = ext->menuManager();
        mm.setPath("/File").setPath(N_("Project File Options"));
        mm.addItem(_("Edit Path Set"))
            ->sigTriggered().connect(bind(&PathSetEditor::initAndShow, editor));
        initialized = true;
    }
}


PathSetEditor::PathSetEditor()
    : QDialog(MainWindow::instance())
{
    setWindowTitle(_("Project File Path Set"));

    QVBoxLayout* vbox = new QVBoxLayout();
    tableWidget = new QTableWidget(this);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    tableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem(_("Variable")));
    tableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem(_("Path List")));
    tableWidget->horizontalHeader()->setStretchLastSection(true);

    tableWidget->verticalHeader()->hide();
    tableWidget->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
    
    vbox->addWidget(tableWidget, 1);

    QHBoxLayout* hbox = new QHBoxLayout();
    QLabel* label = new QLabel(_("Variable"));
    hbox->addWidget(label);
    newVariableEntry = new QLineEdit();
    hbox->addWidget(newVariableEntry);
    PushButton* button = new PushButton(_("Append"));
    hbox->addWidget(button);
    button->sigClicked().connect(bind(&PathSetEditor::onAppendActivated, this));
    vbox->addLayout(hbox);
    
    QPushButton* createButton = new QPushButton(_("&Apply"));
    createButton->setDefault(true);
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(createButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox,SIGNAL(rejected()), this, SLOT(reject()));
    vbox->addWidget(buttonBox);

    setLayout(vbox);
}


void PathSetEditor::initAndShow()
{
    readPathSetFromArchive();
    if(exec() == QDialog::Accepted){
        writePathSetToArchive();
    }
}


void PathSetEditor::readPathSetFromArchive()
{
    YamlMappingPtr pathSets = AppConfig::archive()->openMapping("ArchivePathSet");
    tableWidget->setRowCount(0);

    for(YamlMapping::const_iterator p = pathSets->begin(); p != pathSets->end(); ++p){
        const std::string& name = p->first;
        YamlNodePtr value = p->second;
        if(!name.empty() && value->isString()){
            appendPathSet(name.c_str(), value->toString().c_str());
        }
    }
}


void PathSetEditor::appendPathSet(const QString& name, const QString& paths)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name);
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);

    PathSetItem* pathSetItem = new PathSetItem(paths);
    tableWidget->setItem(row, 1, pathSetItem);
}


void PathSetEditor::writePathSetToArchive()
{
    YamlMappingPtr pathSets = AppConfig::archive()->openMapping("ArchivePathSet");
    pathSets->clear();

    int n = tableWidget->rowCount();
    for(int i=0; i < n; ++i){
        PathSetItem* item = dynamic_cast<PathSetItem*>(tableWidget->item(i, 1));
        if(item){
            string name = tableWidget->item(i, 0)->text().toStdString();
            if(!name.empty() && !item->paths.isEmpty()){
                pathSets->write(name, item->paths.toStdString());
            }
        }
    }
}


void PathSetEditor::onAppendActivated()
{
    appendPathSet(newVariableEntry->text(), "");
}
