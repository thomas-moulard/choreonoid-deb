/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_PLUGIN_MANAGER_H_INCLUDED
#define CNOID_GUIBASE_PLUGIN_MANAGER_H_INCLUDED

#include <string>

namespace cnoid {

    class PluginManagerImpl;

    class PluginManager
    {
    public:

        static PluginManager* instance();
	
        ~PluginManager();
	
        void scanPluginFilesInPathList(const std::string& pathList);
	void scanPluginFilesInDirectoyOfExecFile();
        void scanPluginFiles(const std::string& pathString);
        void clearUnusedPlugins();
        void loadAllPlugins();
        bool unloadAllPlugins();

        int numPlugins();

        const std::string& pluginPath(int index);
        const std::string& pluginName(int index);

        enum PluginStatus { NOT_LOADED, LOADED, ACTIVE, INVALID, CONFLICT };
        int pluginStatus(int index);
	
        bool loadPlugin(int index);
        bool unloadPlugin(int index);

        const char* guessActualPluginName(const std::string& name);
	
    private:
        PluginManager();

        PluginManagerImpl* impl;

    };
};


#endif
