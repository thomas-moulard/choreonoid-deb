/*! @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Config>
#include <cnoid/Plugin>
#include <cnoid/App>
#include <cnoid/ItemManager>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "WorldItem.h"
//#include "FilterDialogs.h"
#include "KinematicFaultChecker.h"
#include "BodyBar.h"
#include "BodyLinkView.h"
#include "LinkSelectionView.h"
#include "JointSliderView.h"
//#include "WorldView.h"
#include "MultiValueSeqGraphView.h"
#include "MultiAffine3SeqGraphView.h"
#include "KinematicsBar.h"
#include "SimulationBar.h"
#include "KinematicsSimulatorItem.h"
#include "DynamicsSimulatorItem.h"
#include "BodyMotionEngine.h"
#include "SceneBodyManager.h"
#include "SceneWorld.h"
#include "gettext.h"

using namespace cnoid;

namespace {
  
    class BodyPlugin : public Plugin
    {
    public:
        BodyPlugin() : Plugin("Body") { }

        virtual bool initialize(){

            Body::addCustomizerDirectory(
                App::topDirectory() + "/" + CNOID_PLUGIN_SUBDIR + "/customizer");

            initializeBodyItem(*this);
            initializeBodyMotionItem(*this);
            initializeWorldItem(*this);
            initializeKinematicsSimulatorItem(*this);
            initializeDynamicsSimulatorItem(*this);

            initializeBodyMotionEngine(*this);
            //initializeFilterDialogs(*this);
            KinematicFaultChecker::initialize(*this);

            addToolBar(BodyBar::instance());
            addToolBar(KinematicsBar::instance());
            addToolBar(SimulationBar::initialize(this));

            addView(new LinkSelectionView());
            addView(new BodyLinkView());
            addView(new JointSliderView());
            //addView(new WorldView());
            addView(new MultiValueSeqGraphView());
            addView(new MultiAffine3SeqGraphView());

            manage(new SceneBodyManager(*this));
            manage(new SceneWorldManager());

            return true;
        }

        virtual const char* description() {

            static std::string text =
                str(boost::format(_("Body Plugin Version %1%\n")) % CNOID_FULL_VERSION_STRING) +
                "\n" +
                _("This plugin has been developed by Shin'ichiro Nakaoka and Choreonoid Development Team, AIST, "
                  "and is distributed as a part of the Choreonoid package.\n"
                  "\n") +
                LGPLtext() +
                "\n" +
                _("The Collision deteciton module used in this plugin is implemented using "
                  "the OPCODE library (http://www.codercorner.com/Opcode.htm).\n");
        
            return text.c_str();
        
                
        }
        
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(BodyPlugin);
