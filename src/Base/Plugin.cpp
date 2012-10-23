/**
   @author Shin'ichiro Nakaoka
*/

#include "Plugin.h"
#include "Item.h"
#include "ToolBar.h"
#include "View.h"
#include "Licenses.h"
#include <boost/variant.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;


namespace cnoid {

    class PluginImpl
    {
    public:
        PluginImpl(const char* name);

        string name;
        vector<string> requisites;
        vector<string> oldNames;
    };
}


Plugin::Plugin(const char* name)
    : ExtensionManager(name, true)
{
    impl = new PluginImpl(name);
}


PluginImpl::PluginImpl(const char* name)
    : name(name)
{

}


Plugin::~Plugin() 
{
    delete impl;
}


const char* Plugin::name()
{
    return impl->name.c_str();
}


bool Plugin::initialize()
{
    return true;
}


bool Plugin::finalize()
{
    return true;
}


/**
   When the plugin depends on some other plugins,
   please specify the plugins to depend with this function in the constructor.
*/
void Plugin::require(const char* pluginName)
{
    impl->requisites.push_back(pluginName);
}


/**
   Obsolete. Please use require() instead of this function.
*/
void Plugin::depend(const char* pluginName)
{
    require(pluginName);
}


const char* Plugin::requisite(int index)
{
    return impl->requisites[index].c_str();
}


int Plugin::numRequisites()
{
    return impl->requisites.size();
}


void Plugin::addOldName(const char* name)
{
    impl->oldNames.push_back(name);
}


const char* Plugin::oldName(int index)
{
    return impl->oldNames[index].c_str();
}


int Plugin::numOldNames()
{
    return impl->oldNames.size();
}


const char* Plugin::description()
{
    return "";
}


const char* Plugin::LGPLtext()
{
    return cnoid::LGPLtext();
}
