/**
   @author Shin'ichiro NAKAOKA
*/

#include "ExtensionManager.h"
#include "ItemManager.h"
#include "MenuManager.h"
#include "OptionManager.h"
#include "ProjectManager.h"
#include "Plugin.h"
#include "Item.h"
#include "View.h"
#include "ToolBar.h"
#include "MainWindow.h"
#include "App.h"
#include "TimeSyncItemEngineManager.h"
#include "LazyCaller.h"
#include <cnoid/Config>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <set>
#include <stack>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

    class ExtensionManagerImpl
    {
    public:
        ExtensionManagerImpl(ExtensionManager* self, const std::string& moduleName);
        ~ExtensionManagerImpl();

        void setVersion(const std::string& version, bool isPlugin);

        void deleteManagedObjects();;

        ExtensionManager* self;
        scoped_ptr<MenuManager> menuManager;
        scoped_ptr<ItemManager> itemManager;
        scoped_ptr<TimeSyncItemEngineManager> timeSyncItemEngineManger;
        string moduleName;
        string textDomain;

        stack<ExtensionManager::PtrHolderBase*> pointerHolders;

        boost::signal<void()> sigSystemUpdated;
        boost::signal<void()> sigReleaseRequest;

    };

}

namespace {
    
    set<ExtensionManagerImpl*> extensionManagerImpls;

    void emitSigSystemUpdated()
    {
        set<ExtensionManagerImpl*>::iterator p = extensionManagerImpls.begin();
        while(p != extensionManagerImpls.end()){
            (*p)->sigSystemUpdated();
            ++p;
        }
    }

    LazyCaller emitSigSystemUpdatedCaller(emitSigSystemUpdated);
}


ExtensionManager::ExtensionManager(const std::string& moduleName, bool isPlugin)
{
    impl = new ExtensionManagerImpl(this, moduleName);
    impl->setVersion(CNOID_FULL_VERSION_STRING, isPlugin);
}


ExtensionManager::ExtensionManager(const std::string& moduleName, const std::string& version, bool isPlugin)
{
    impl = new ExtensionManagerImpl(this, moduleName);
    impl->setVersion(version, isPlugin);
}
    
    
ExtensionManagerImpl::ExtensionManagerImpl(ExtensionManager* self, const std::string& moduleName)
    : self(self),
      moduleName(moduleName)
{
    extensionManagerImpls.insert(this);
}


ExtensionManager::~ExtensionManager()
{
    delete impl;
}


ExtensionManagerImpl::~ExtensionManagerImpl()
{
    ProjectManager::instance()->disconnectArchivers(moduleName);
    deleteManagedObjects();
    sigReleaseRequest();
    extensionManagerImpls.erase(this);
}


ItemManager& ExtensionManager::itemManager()
{
    if(!impl->itemManager){
        impl->itemManager.reset(new ItemManager(impl->moduleName, menuManager()));
        impl->itemManager->bindTextDomain(impl->textDomain);
    }
    return *impl->itemManager;
}


TimeSyncItemEngineManager& ExtensionManager::timeSyncItemEngineManger()
{
    if(!impl->timeSyncItemEngineManger){
        impl->timeSyncItemEngineManger.reset(new TimeSyncItemEngineManager(impl->moduleName));
    }
    return *impl->timeSyncItemEngineManger;
}


MenuManager& ExtensionManager::menuManager()
{
    if(!impl->menuManager){
        impl->menuManager.reset(new MenuManager(MainWindow::instance()->menuBar()));
        impl->menuManager->bindTextDomain(impl->textDomain);
    }
    return *impl->menuManager;
}


OptionManager& ExtensionManager::optionManager()
{
    static OptionManager optionManager;
    return optionManager;
}


void ExtensionManagerImpl::setVersion(const std::string& version, bool isPlugin)
{
    vector<string> v;
    boost::algorithm::split(v, version, boost::is_any_of("."));

    textDomain = string("Cnoid") + moduleName;
    if(isPlugin){
        textDomain += "Plugin";
    }
    if(!v.empty()){
        textDomain += "-";
        textDomain += v[0];
        if(v.size() >= 2){
            textDomain += string(".") + v[1];
        }
    }
    bindtextdomain(textDomain.c_str(), (App::topDirectory() + "/" + CNOID_LOCALE_SUBDIR).c_str());
}


const char* ExtensionManager::textDomain() const
{
    return impl->textDomain.c_str();
}


ExtensionManager::PtrHolderBase::~PtrHolderBase()
{

}


void ExtensionManager::manageSub(PtrHolderBase* holder)
{
    impl->pointerHolders.push(holder);
}


void ExtensionManager::addView(View* view)
{
    manage(view);
    if(!impl->textDomain.empty()){
        view->setWindowTitle(dgettext(impl->textDomain.c_str(), view->objectName().toAscii()));
    }
    MainWindow::instance()->addView(impl->moduleName, view);
}


void ExtensionManager::addToolBar(ToolBar* toolBar)
{
    manage(toolBar);
    MainWindow::instance()->addToolBar(toolBar);
}


void ExtensionManagerImpl::deleteManagedObjects()
{
    while(!pointerHolders.empty()){
        ExtensionManager::PtrHolderBase* holder = pointerHolders.top();
        delete holder;
        pointerHolders.pop();
    }
}


SignalProxy< boost::signal<void()> > ExtensionManager::sigSystemUpdated()
{
    return impl->sigSystemUpdated;
}


void ExtensionManager::notifySystemUpdate()
{
    emitSigSystemUpdatedCaller.request();
}


SignalProxy< boost::signal<void()> > ExtensionManager::sigReleaseRequest()
{
    return impl->sigReleaseRequest;
}


void ExtensionManager::connectProjectArchiver(
    const std::string& name,
    boost::function<bool(Archive&)> storeFunction,
    boost::function<void(const Archive&)> restoreFunction)
{
    ProjectManager::instance()->connectArchiver(impl->moduleName, name, storeFunction, restoreFunction);
}
