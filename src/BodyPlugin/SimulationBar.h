/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SIMULATION_BAR_H_INCLUDED
#define CNOID_BODYPLUGIN_SIMULATION_BAR_H_INCLUDED

#include <cnoid/ToolBar>
#include <iosfwd>

namespace cnoid {

    class SimulationBar : public ToolBar
    {
    public:

        static SimulationBar* initialize(ExtensionManager* ext);
        static SimulationBar* instance();
            
        virtual ~SimulationBar();

    private:

        SimulationBar();

        void onStartStopButtonClicked();
        void startSimulation();
        void stopSimulation();

        std::ostream& os;

        bool isDoingSimulation;
        ToolButton* startStopButton;
        QIcon startIcon;
        QIcon stopIcon;
        boost::signals::connection simulationFinishedConnection;
    };
}

#endif
