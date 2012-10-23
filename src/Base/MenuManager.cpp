/**
   @author Shin'ichiro NAKAOKA
*/

#include "MenuManager.h"
#include <cnoid/MainWindow>
#include <QMenuBar>
#include <boost/tuple/tuple.hpp>
#include <set>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


Menu::Menu(QWidget* parent)
    : QMenu(parent)
{
    initialize();
}


Menu::Menu(const QString& title, QWidget* parent)
    : QMenu(title, parent)
{
    initialize();
}


void Menu::initialize()
{
    connect(this, SIGNAL(triggered(QAction*)), this, SLOT(onTriggered(QAction*)));
    connect(this, SIGNAL(aboutToHide()), this, SLOT(onAboutToHide()));
}


void Menu::onTriggered(QAction* action)
{
    sigTriggered_(action);
}


void Menu::onAboutToHide()
{
    sigAboutToHide_();
}


namespace cnoid {

    class MenuManagerImpl
    {
    public:
        MenuManager* self;
        QWidget* topMenu;
        QWidget* currentMenu;
        bool isBackwardMode;
        typedef set<QAction*> ActionSet;
        ActionSet allActions;
        std::string textDomain;

        MenuManagerImpl(MenuManager* self, QWidget* topMenu);
        ~MenuManagerImpl();
        void setTopMenu(QWidget* topMenu);
        void releaseAllItems();
        void setReference(QAction* item);
        void unsetReference(QAction* item);
        pair<QAction*, QWidget*> findPath(const QString& path, bool createPath);
        void addItem(QWidget* menu, QAction* item);
        void setPath(const QString& path);
        MenuManager& setBackwardMode();
        Action* addItem(const QString& text);
        void addSeparator();
        void removeItem(QAction* item);
    };
}
        

MenuManager::MenuManager()
{
    impl = new MenuManagerImpl(this, 0);
}


MenuManager::MenuManager(QWidget* topMenu)
{
    impl = new MenuManagerImpl(this, topMenu);
}


MenuManagerImpl::MenuManagerImpl(MenuManager* self, QWidget* topMenu)
    : self(self)
{
    this->topMenu = topMenu;
    currentMenu = topMenu;
    isBackwardMode = false;
}


void MenuManager::bindTextDomain(const std::string& domain)
{
    impl->textDomain = domain;
}


void MenuManager::setTopMenu(QWidget* topMenu)
{
    impl->setTopMenu(topMenu);
}


void MenuManagerImpl::setTopMenu(QWidget* topMenu)
{
    if(this->topMenu){
        releaseAllItems();
    }
    this->topMenu = topMenu;
    currentMenu = topMenu;
}


MenuManager::~MenuManager()
{
    delete impl;
}


MenuManagerImpl::~MenuManagerImpl()
{
    releaseAllItems();
}


void MenuManager::releaseAllItems()
{
    impl->releaseAllItems();
}


void MenuManagerImpl::releaseAllItems()
{
    while(true){
        ActionSet::iterator p = allActions.begin();
        if(p == allActions.end()){
            break;
        }
        QAction* action = *p;
        allActions.erase(p);
        unsetReference(action);
    }
    currentMenu = topMenu;
    isBackwardMode = false;
}


void MenuManagerImpl::setReference(QAction* item)
{
    ActionSet::iterator p = allActions.find(item);
    if(p == allActions.end()){
        allActions.insert(item);
        QObject::connect(item, SIGNAL(destroyed()), self, SLOT(onItemDestroyed()));
        int ref = 0;
        QVariant mmref = item->property("mmref");
        if(mmref.isValid()){
            ref = mmref.toInt();
        }
        item->setProperty("mmref", ref + 1);
    }
}


void MenuManager::onItemDestroyed()
{
    impl->allActions.erase(dynamic_cast<QAction*>(sender()));
}


void MenuManagerImpl::unsetReference(QAction* item)
{
    if(QObject::disconnect(item, SIGNAL(destroyed()), self, SLOT(onItemDestroyed()))){
        QVariant mmref = item->property("mmref");
        if(mmref.isValid()){
            int ref = mmref.toInt() - 1;
            if(ref == 0){
                delete item;
            } else {
                item->setProperty("mmref", ref);
            }
        }
    }
}


QAction* MenuManager::findItem(const QString& path)
{
    return impl->findPath(path, false).first;
}


pair<QAction*, QWidget*> MenuManagerImpl::findPath(const QString& path, bool createPath)
{
    int pos = 0;
    int size = path.size();
    QAction* item = 0;
    QWidget* menu = currentMenu;
    
    if(path[pos] == QChar('/')){
        pos++;
        menu = topMenu;
    }

    while(menu && (pos != size)){

        int next = path.indexOf(QChar('/'), pos);
        int length = (next >= 0) ? (next - pos) : next;
        QString name = path.mid(pos, length);

        QList<QAction*> items = menu->actions();
        item = 0;
        for(int i=0; i < items.size(); ++i){
            if(name == items[i]->objectName()){
                item = items[i];
            }
        }
        if(!item){
            if(createPath){
                if(textDomain.empty()){
                    item = new QAction(name, menu);
                } else {
                    item = new QAction(dgettext(textDomain.c_str(), name.toAscii()), menu);
                }
                item->setObjectName(name);
                addItem(menu, item);
                item->setMenu(new QMenu());
            } else {
                break;
            }
        }

        setReference(item);
        menu = item->menu();

        pos = (next >= 0) ? (next + 1) : size;
    }

    return make_pair(item, menu);
}


void MenuManagerImpl::addItem(QWidget* menu, QAction* item)
{
    QList<QAction*> items = menu->actions();
    
    int position;
    
    for(position = items.size() - 1; position >= 0; --position){
        QAction* sibling = items[position];
        if(!sibling->property("isBackward").toBool()){
            break;
        }
    }
    position++;
    
    if(position < items.size()){
        menu->insertAction(items[position], item);
    } else {
        menu->addAction(item);
    }

    setReference(item);

    if(isBackwardMode){
        item->setProperty("isBackward", true);
    }
}
    

/**
   @if jp
   アイテム操作の対象となるメニューを指定する。
   
   メニューアイテムを追加する前にあらかじめ呼んでおく必要がある。
   
   @param menuPath 対象となるメニューへのパスを指定する。
   '/' から始まる場合ルートからの位置になり、そうでない場合は
   現在指定されているメニューからの相対パスになる。
   もしパスに指定されたメニューが存在しない場合はメニューが新たに作成される。
   
   @endif
*/
MenuManager& MenuManager::setPath(const QString& path)
{
    impl->setPath(path);
    return *this;
}


void MenuManagerImpl::setPath(const QString& path)
{
    if(!path.isEmpty() && path[0] == QChar('/')){
        isBackwardMode = false;
    }
    
    QAction* item;
    QWidget* menu;

    tie(item, menu) = findPath(path, true);

    if(!menu){
        cerr << "cnoid::MenuManager warning: setting path to " << path.toLocal8Bit().data() << " failed." << endl;
    }

    currentMenu = menu;

    isBackwardMode = false;
}


MenuManager& MenuManager::setBackwardMode()
{
    impl->isBackwardMode = true;
    return *this;
}


Action* MenuManager::addItem(const QString& text)
{
    return impl->addItem(text);
}


Action* MenuManagerImpl::addItem(const QString& text)
{
    Action* item = new Action(text, currentMenu);
    addItem(currentMenu, item);
    return item;
}


Action* MenuManager::addCheckItem(const QString& text)
{
    Action* item = impl->addItem(text);
    if(item){
        item->setCheckable(true);
    }
    return item;
}


Action* MenuManager::addRadioItem(QActionGroup* group, const QString& text)
{
    Action* item = impl->addItem(text);
    if(item){
        item->setActionGroup(group);
    }
    return item;
}


MenuManager& MenuManager::addSeparator()
{
    impl->addSeparator();
    return *this;
}


/**
   \todo implement smart separator
*/
void MenuManagerImpl::addSeparator()
{
    QAction* separator = new QAction(currentMenu);
    separator->setSeparator(true);
    addItem(currentMenu, separator);
}


void MenuManager::removeItem(QAction* item)
{
    impl->removeItem(item);
}


void MenuManagerImpl::removeItem(QAction* item)
{
    ActionSet::iterator p = allActions.find(item);
    if(p != allActions.end()){
        allActions.erase(p);
        unsetReference(item);
    }
}
