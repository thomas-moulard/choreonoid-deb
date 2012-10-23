/**
   @author Shin'ichiro Nakaoka
*/

#include "SimulationBar.h"
#include "SimulatorItem.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    Action* se3Check;
}


SimulationBar* SimulationBar::initialize(ExtensionManager* ext)
{
    MenuManager& mm = ext->menuManager();
    mm.setPath("/Options").setPath(N_("Simulation"));
    
    se3Check = mm.addCheckItem(_("Output all link positions"));
    se3Check->setChecked(false);

    return instance();
}


SimulationBar* SimulationBar::instance()
{
    static SimulationBar* instance = new SimulationBar();
    return instance;
}


SimulationBar::SimulationBar()
    : ToolBar("SimulationBar"),
      startIcon(QIcon(":/Body/icons/startsimulation.png")),
      stopIcon(QIcon(":/Body/icons/stopsimulation.png")),
      os(MessageView::mainInstance()->cout())
{
    startStopButton = addButton(startIcon, _("Start simulation"));
    startStopButton->sigClicked().connect(bind(&SimulationBar::onStartStopButtonClicked, this));

    isDoingSimulation = false;
}


SimulationBar::~SimulationBar()
{

}


void SimulationBar::onStartStopButtonClicked()
{
    if(isDoingSimulation){
        stopSimulation();
    } else {
        startSimulation();
    }
}


void SimulationBar::startSimulation()
{
    SimulatorItemPtr simulatorItem =
        ItemTreeView::mainInstance()->selectedItem<SimulatorItem>();

    simulationFinishedConnection.disconnect();

    if(simulatorItem){
        simulatorItem->setAllLinkPositionOutputMode(se3Check->isChecked());

        simulationFinishedConnection =
            simulatorItem->sigSimulationFinished().connect(
                bind(&SimulationBar::stopSimulation, this));
        
        isDoingSimulation = true;

        const static QString tip(_("Stop simulation"));
        startStopButton->setIcon(stopIcon);
        startStopButton->setToolTip(tip);

        simulatorItem->startSimulation();

    } else {
        os << "Simulation cannot start. No simulation item is selected." << endl;
    }
}


void SimulationBar::stopSimulation()
{
    simulationFinishedConnection.disconnect();

    SimulatorItemPtr simulatorItem =
        ItemTreeView::mainInstance()->selectedItem<SimulatorItem>();

    if(simulatorItem){
        if(simulatorItem->isSimulationRunning()){
            simulatorItem->stopSimulation();
        }
    }

    const static QString tip(_("Start simulation"));
    startStopButton->setIcon(startIcon);
    startStopButton->setToolTip(tip);

    isDoingSimulation = false;
}
