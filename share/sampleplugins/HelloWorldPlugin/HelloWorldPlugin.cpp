/**
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <boost/bind.hpp>

using namespace cnoid;
using namespace boost;

class HelloWorldPlugin : public Plugin
{
    void onHelloWorldActivated() {
        MessageView::mainInstance()->putln(QObject::tr("Hello World !"));
    }

public:
    
    HelloWorldPlugin() : Plugin("HelloWorld") { }
    
    virtual bool initialize() {

        menuManager().setPath(QObject::tr("/View")).addItem(QObject::tr("Hello World"))
            ->sigTriggered().connect(
                bind(&HelloWorldPlugin::onHelloWorldActivated, this));
        
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
