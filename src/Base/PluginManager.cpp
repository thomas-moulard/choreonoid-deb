/**
   @author Shin'ichiro Nakaoka
*/

#include "PluginManager.h"
#include "Plugin.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "DescriptionDialog.h"
#include "App.h"
#include "LazyCaller.h"
#include <cnoid/Config>
#include <QLibrary>
#include <QRegExp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


#ifdef Q_OS_WIN32
static const char* DLL_PREFIX = "";
static const char* DLL_SUFFIX = "dll";
static const char* PATH_DELIMITER = ";";
# ifdef CNOID_DEBUG
static const char* DEBUG_SUFFIX = "d";
# else
static const char* DEBUG_SUFFIX = "";
# endif
#else
# ifdef Q_OS_MAC
static const char* DLL_PREFIX = "";
static const char* DLL_SUFFIX = "dylib";
static const char* PATH_DELIMITER = ":";
static const char* DEBUG_SUFFIX = "";
# else
static const char* DLL_PREFIX = "lib";
static const char* DLL_SUFFIX = "so";
static const char* PATH_DELIMITER = ":";
static const char* DEBUG_SUFFIX = "";
# endif
#endif


namespace cnoid {

    class PluginManagerImpl
    {
    public:

        PluginManagerImpl();
        ~PluginManagerImpl();

        MessageView* mv;

        QRegExp pluginNamePattern;

        struct PluginInfo;
        typedef shared_ptr<PluginInfo> PluginInfoPtr;
        typedef map<std::string, PluginInfoPtr> PluginMap;

        struct PluginInfo {
            PluginInfo(){
                plugin = 0;
                status = PluginManager::NOT_LOADED;
                aboutMenuItem = 0;
                aboutDialog = 0;
            }
            QLibrary dll;
            std::string pathString;
            Plugin* plugin;
            string name;
            vector<string> requisites;
            vector<string> dependents;
            
            int status;
            
            QAction* aboutMenuItem;
            DescriptionDialog* aboutDialog;
        };

        vector<PluginInfoPtr> allPluginInfos;
        PluginMap nameToActivePluginInfoMap;
        PluginMap pathToPluginInfoMap;

        typedef multimap<std::string, std::string> MultiNameMap;
        MultiNameMap oldNameToCurrentPluginNameMap;

        vector<PluginInfoPtr> pluginsToUnload;
        LazyCaller unloadingRequest;
    
        void clearUnusedPlugins();
        void scanPluginFilesInDefaultPath(const std::string& pathList);
        void scanPluginFilesInDirectoyOfExecFile();
        void scanPluginFiles(const std::string& pathString, bool isRecursive);
        void loadAllPlugins();
        bool unloadAllPlugins();
        bool loadPlugin(int index);
        void onAboutDialogTriggered(PluginInfo* info);
        const char* guessActualPluginName(const std::string& name);
        bool unloadPlugin(int index);
        bool unloadPlugin(PluginInfoPtr info);
        void onUnloadingRequest();
    };
}


PluginManager* PluginManager::instance()
{
    static PluginManager* pluginManager = new PluginManager();
    return pluginManager;
}


PluginManager::PluginManager()
{
    impl = new PluginManagerImpl();
}


PluginManagerImpl::PluginManagerImpl()
    : mv(MessageView::mainInstance())
{
    pluginNamePattern.setPattern(QString(DLL_PREFIX) + "Cnoid.+Plugin" + DEBUG_SUFFIX + "\\." + DLL_SUFFIX);

    // for the base module
    PluginInfoPtr info(new PluginInfo);
    info->name = "Base";
    nameToActivePluginInfoMap.insert(make_pair(string("Base"), info));

    unloadingRequest.set(bind(&PluginManagerImpl::onUnloadingRequest, this), IDLE_PRIORITY_LOW);
}


PluginManager::~PluginManager()
{
    delete impl;
}


PluginManagerImpl::~PluginManagerImpl()
{
    unloadAllPlugins();
}


int PluginManager::numPlugins()
{
    return impl->allPluginInfos.size();
}


const std::string& PluginManager::pluginPath(int index)
{
    return impl->allPluginInfos[index]->pathString;
}


const std::string& PluginManager::pluginName(int index)
{
    return impl->allPluginInfos[index]->name;
}


int PluginManager::pluginStatus(int index)
{
    return impl->allPluginInfos[index]->status;
}


/**
   This function scans plugin files in the path list.
   @param pathList List of the directories to scan, which is specified with the same format
   as the PATH environment varibale.
*/
void PluginManager::scanPluginFilesInPathList(const std::string& pathList)
{
    impl->scanPluginFilesInDefaultPath(pathList);
}


void PluginManagerImpl::scanPluginFilesInDefaultPath(const std::string& pathList)
{
    char_separator<char> sep(PATH_DELIMITER);
    tokenizer< char_separator<char> > paths(pathList, sep);
    tokenizer< char_separator<char> >::iterator p;
    for(p = paths.begin(); p != paths.end(); ++p){
        const string& path = *p;
        scanPluginFiles(path, false);
    }
}


void PluginManager::scanPluginFilesInDirectoyOfExecFile()
{
  impl->scanPluginFilesInDirectoyOfExecFile();
}


void PluginManagerImpl::scanPluginFilesInDirectoyOfExecFile()
{
    string pluginDirectory(
        str(boost::format("%1%/%2%")
            % App::topDirectory() % CNOID_PLUGIN_SUBDIR));
    
    scanPluginFiles(pluginDirectory, false);
} 


void PluginManager::scanPluginFiles(const std::string& pathString)
{
    impl->scanPluginFiles(pathString, false);
}


void PluginManagerImpl::scanPluginFiles(const std::string& pathString, bool isRecursive)
{
    filesystem::path pluginPath(pathString);

    if(filesystem::exists(pluginPath)){
       if(filesystem::is_directory(pluginPath)){
           if(!isRecursive){
               filesystem::directory_iterator end;
               for(filesystem::directory_iterator it(pluginPath); it != end; ++it){
                   const filesystem::path& filepath = *it;
                   scanPluginFiles(filepath.file_string(), true);
               }
           }
       } else {
           if(pluginNamePattern.exactMatch(pluginPath.leaf().c_str())){
                PluginMap::iterator p = pathToPluginInfoMap.find(pathString);
                if(p == pathToPluginInfoMap.end()){
                    PluginInfoPtr info(new PluginInfo);
                    info->pathString = pathString;
                    allPluginInfos.push_back(info);
                    pathToPluginInfoMap[pathString] = info;
                }
            }
        }
    }
}


void PluginManager::clearUnusedPlugins()
{
    impl->clearUnusedPlugins();
}


void PluginManagerImpl::clearUnusedPlugins()
{
    vector<PluginInfoPtr> oldList = allPluginInfos;
    allPluginInfos.clear();

    for(size_t i=0; i < oldList.size(); ++i){
        PluginInfoPtr info = oldList[i];
        if(info->status == PluginManager::ACTIVE){
            allPluginInfos.push_back(info);
        } else {
            pathToPluginInfoMap.erase(info->pathString);
        }
    }
}


void PluginManager::loadAllPlugins()
{
    impl->loadAllPlugins();
}


void PluginManagerImpl::loadAllPlugins()
{
    size_t totalNumActivated = 0;
    while(true){
        size_t numActivated = 0;
        for(size_t i=0; i < allPluginInfos.size(); ++i){
            int status = allPluginInfos[i]->status;
            if(status == PluginManager::NOT_LOADED || status == PluginManager::LOADED){
                if(loadPlugin(i)){
                    numActivated++;
                    totalNumActivated++;
                }
            }
        }
        if(numActivated == 0 || totalNumActivated == allPluginInfos.size()){
            break;
        }
    }
}


bool PluginManager::loadPlugin(int index)
{
    return impl->loadPlugin(index);
}


bool PluginManagerImpl::loadPlugin(int index)
{
    QString errorMessage;

    PluginInfoPtr info = allPluginInfos[index];
    
    if(info->status == PluginManager::ACTIVE){
        mv->putln(format(_("Plugin file \"%1%\" has already been activated.")) % info->pathString);

    } else if(info->status == PluginManager::NOT_LOADED){
        mv->putln(format(_("Loading plugin file \"%1%\"")) % info->pathString);
        mv->flush();

        info->dll.setFileName(info->pathString.c_str());

        if(!(info->dll.load())){
            errorMessage = info->dll.errorString();

        } else {
            void* symbol = info->dll.resolve("getChoreonoidPlugin");
            if(!symbol){
                info->status = PluginManager::INVALID;
                errorMessage = _("The plugin entry function \"getChoreonoidPlugin\" is not found.\n");
                errorMessage += info->dll.errorString();

            } else {
                Plugin::PluginEntry getCnoidPluginFunc = (Plugin::PluginEntry)(symbol);
                info->plugin = getCnoidPluginFunc();

                if(!info->plugin){
                    info->status = PluginManager::INVALID;
                    errorMessage = _("The plugin object cannot be created.");

                } else {

                    info->status = PluginManager::LOADED;
                    info->name = info->plugin->name();

                    int numRequisites = info->plugin->numRequisites();
                    for(int i=0; i < numRequisites; ++i){
                        info->requisites.push_back(info->plugin->requisite(i));
                    }
                }
            }
        }
    }

    if(info->status == PluginManager::LOADED){

        PluginMap::iterator p = nameToActivePluginInfoMap.find(info->name);

        if(p != nameToActivePluginInfoMap.end()){
            info->status = PluginManager::CONFLICT;
            PluginInfoPtr loadedPlugin = p->second;
            errorMessage = str(format(_("The plugin conflicts with plugin \"%1%\".")) % loadedPlugin->pathString).c_str();

        } else {
            bool requisitesActive = true;

            // check whether all the required plugins have already been active
            for(size_t i=0; i < info->requisites.size(); ++i){
                PluginMap::iterator q = nameToActivePluginInfoMap.find(info->requisites[i]);
                if(q == nameToActivePluginInfoMap.end()){
                    requisitesActive = false;
                    break;
                }
            }

            if(requisitesActive){
                if(!info->plugin->initialize()){
                    info->status = PluginManager::INVALID;
                    errorMessage = _("The plugin object cannot be intialized.");
                } else {
                    info->status = PluginManager::ACTIVE;
                    nameToActivePluginInfoMap[info->name] = info;

                    // add this plugin to dependent list of the requisites
                    for(size_t i=0; i < info->requisites.size(); ++i){
                        PluginMap::iterator p = nameToActivePluginInfoMap.find(info->requisites[i]);
                        if(p != nameToActivePluginInfoMap.end()){
                            p->second->dependents.push_back(info->name);
                        }
                    }

                    // set an about dialog
                    info->plugin->menuManager().setPath("/Help").setPath(_("About Plugins"))
                        .addItem(str(format(_("About %1% Plugin")) % info->name).c_str())
                        ->sigTriggered().connect(
                            bind(&PluginManagerImpl::onAboutDialogTriggered, this, info.get()));


                    // register old names
                    int numOldNames = info->plugin->numOldNames();
                    for(int i=0; i < numOldNames; ++i){
                        oldNameToCurrentPluginNameMap.insert(
                            make_pair(info->plugin->oldName(i), info->name));
                    }
                    
                    mv->putln(format(_("%1%-plugin has been activated.")) % info->name);
                    mv->flush();
                    ExtensionManager::notifySystemUpdate();
                }
            }
        }
    }

    if(!errorMessage.isEmpty()){
        mv->putln(_("Loading the plugin failed."));
        mv->putln(errorMessage);
        mv->flush();
    }

    return (info->status == PluginManager::ACTIVE);
}


void PluginManagerImpl::onAboutDialogTriggered(PluginInfo* info)
{
    if(!info->aboutDialog){
        info->aboutDialog = new DescriptionDialog();
        info->aboutDialog->setWindowTitle(str(format(_("About %1% Plugin")) % info->name).c_str());
        info->aboutDialog->setDescription(info->plugin->description());
    }

    info->aboutDialog->show();
}


const char* PluginManager::guessActualPluginName(const std::string& name)
{
    return impl->guessActualPluginName(name);
}


const char* PluginManagerImpl::guessActualPluginName(const std::string& name)
{
    PluginMap::iterator p = nameToActivePluginInfoMap.find(name);
    if(p != nameToActivePluginInfoMap.end()){
        return p->second->name.c_str();
    }

    MultiNameMap::iterator q, upper_bound;
    std::pair<MultiNameMap::iterator, MultiNameMap::iterator> range =
        oldNameToCurrentPluginNameMap.equal_range(name);
    for(MultiNameMap::iterator q = range.first; q != range.second; ++q){
        const string& candidate = q->second;
        PluginMap::iterator r = nameToActivePluginInfoMap.find(candidate);
        if(r != nameToActivePluginInfoMap.end()){
            return r->second->name.c_str();
        }
    }

    return 0;
}


bool PluginManager::unloadPlugin(int index)
{
    return impl->unloadPlugin(index);
}


bool PluginManagerImpl::unloadPlugin(int index)
{
    return unloadPlugin(allPluginInfos[index]);
}


bool PluginManagerImpl::unloadPlugin(PluginInfoPtr info)
{
    bool unloadFailed = false;
    
    if(info->status == PluginManager::ACTIVE){

        if(info->plugin){

            for(size_t i=0; i < info->dependents.size(); ++i){
                PluginMap::iterator p = nameToActivePluginInfoMap.find(info->dependents[i]);
                if(p != nameToActivePluginInfoMap.end()){
                    PluginInfoPtr& dependentInfo = p->second;
                    if(dependentInfo->status == PluginManager::ACTIVE ||
                       dependentInfo->status == PluginManager::LOADED){
                        if(!unloadPlugin(dependentInfo)){
                            unloadFailed = true;
                        }
                    }
                }
            }
            if(unloadFailed){
                mv->putln(format(_("Plugin %1% cannot be finalized because its dependent(s) cannot be finalized."))
                          % info->name);
                mv->flush();

            } else if(info->plugin->finalize()){
                info->status = PluginManager::LOADED;
                    
            } else {
                unloadFailed = true;
                mv->putln(format(_("Plugin %1% cannot be finalized.")) % info->name);
                mv->flush();
            }
        }
    }

    if(!unloadFailed && info->status == PluginManager::LOADED){

        delete info->plugin;
        info->plugin = 0;
        if(info->aboutDialog){
            delete info->aboutDialog;
            info->aboutDialog = 0;
        }
        nameToActivePluginInfoMap.erase(info->name);
        info->status = PluginManager::NOT_LOADED;

        ExtensionManager::notifySystemUpdate();

        pluginsToUnload.push_back(info);

        unloadingRequest.request();
    }
    
    return !unloadFailed;
}


void PluginManagerImpl::onUnloadingRequest()
{
    for(size_t i=0; i < pluginsToUnload.size(); ++i){
        PluginInfoPtr& info = pluginsToUnload[i];
        info->dll.unload();
        mv->putln(format(_("Plugin dll %1% has been unloaded.")) % info->pathString);
        mv->flush();
    }
    pluginsToUnload.clear();
}


bool PluginManager::unloadAllPlugins()
{
    return impl->unloadAllPlugins();
}


bool PluginManagerImpl::unloadAllPlugins()
{
    bool failed = false;
    for(size_t i=0; i < allPluginInfos.size(); ++i){
        if(!unloadPlugin(i)){
            failed = true;
        }
    }
    return !failed;
}
