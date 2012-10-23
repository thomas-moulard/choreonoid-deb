/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PLUGIN_H_INCLUDED
#define CNOID_BASE_PLUGIN_H_INCLUDED

#include "ExtensionManager.h"
#include "exportdecl.h"

namespace cnoid {

    class Item;
    class View;
    class ToolBar;
    class PluginImpl;

    class CNOID_EXPORT Plugin : public ExtensionManager
    {
    public:

	typedef Plugin* (*PluginEntry)();
	
	Plugin(const char* name);
	virtual ~Plugin();
	
	const char* name();
        
        virtual bool initialize();
        virtual bool finalize();

        const char* requisite(int index);
        int numRequisites();

        const char* oldName(int index);
        int numOldNames();
        
        virtual const char* description();
        
    protected:

	void setPluginScope(Item* item);
	void setPluginScope(View* view);
	void setPluginScope(ToolBar* toolBar);

        void require(const char* pluginName);
	void depend(const char* pluginName);

        /**
           When the plugin name is changed but the old project files should be loadable,
           specify old names of the plugin with this function in the constructor.
        */
        void addOldName(const char* name);

        static const char* LGPLtext();

    private:

        Plugin(const Plugin& org); // disable the copy constructor

        PluginImpl* impl;
        
    };
}


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CNOID_IMPLEMENT_PLUGIN_ENTRY(PluginTypeName) \
    extern "C" __declspec(dllexport) cnoid::Plugin* getChoreonoidPlugin() \
    {                                    \
        return new PluginTypeName();      \
    }
#else
#define CNOID_IMPLEMENT_PLUGIN_ENTRY(PluginTypeName) \
    extern "C" cnoid::Plugin* getChoreonoidPlugin() \
    {                                    \
        return new PluginTypeName();      \
    }
#endif


#endif
