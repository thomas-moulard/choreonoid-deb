/**
   @author Shin'ichiro Nakaoka
*/

#include "GraphBar.h"
#include "GraphWidget.h"
#include "ConnectionSet.h"
#include "ExtensionManager.h"
#include "SpinBox.h"
#include "ComboBox.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

    class GraphBarImpl
    {
    public:
        GraphBarImpl(GraphBar* self);

        GraphBar* self;

        ConnectionSet connections;

        ToolButton* orgRenderingToggle;
        ToolButton* velRenderingToggle;
        ToolButton* accRenderingToggle;

        ToolButton* optionToggle;
        vector<QWidget*> optWidgets;
        
        DoubleSpinBox verticalValueRangeSpin;
        SpinBox lineWidthSpin;
        ToolButton* limitValueToggle;
        ToolButton* gridToggle;
        DoubleSpinBox* gridSizeSpin;
        ToolButton* rulersToggle;
        ToolButton* timeBarSyncToggle;
        ComboBox* autoScrollModeCombo;
        ToolButton* editModeToggle;
        ToolButton* freeLineModeRadio;
        ToolButton* lineModeRadio;
        SpinBox* controlPointStepSpin;
        SpinBox* controlPointOffsetSpin;
        ToolButton* highlightingControlPointsToggle;

        GraphWidget* focusedGraphWidget;

        void onOptionToggled();
        void focus(GraphWidget* graphWidget, bool forceUpdate);
        void onVerticalValueRangeChanged(double value);
        void onLineWidthChanged(int value);
        void onRenderingTypesToggled();
        void onLimitValueToggled(bool on);
        void onGridToggled(bool on);
        void onGridSizeChanged(double size);
        void onRulersToggled(bool on);
        void onTimeBarSyncToggled(bool on);
        void onAutoScrollModeChanged(int index);
        void onEditModeToggled(bool on);
        void onFreeLineModeToggled(bool on);
        void onLineModeToggled(bool on);
        void onControlPointStepOrOffsetChanged();
        void onHighlightingControlPointsToggled(bool on);
    };
}

void GraphBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(GraphBar::instance());
        initialized = true;
    }
}

GraphBar* GraphBar::instance()
{
    static GraphBar* graphBar = new GraphBar();
    return graphBar;
}
       

GraphBar::GraphBar() : ToolBar(N_("GraphBar"))
{
    impl = new GraphBarImpl(this);
}


GraphBarImpl::GraphBarImpl(GraphBar* self)
    : self(self)
{
    orgRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/graph.png"), _("Plot trajectories of the target data on the graph view"));
    orgRenderingToggle->setChecked(true);
    connections.add(
        orgRenderingToggle->sigToggled().connect(
            bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    velRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/velocitygraph.png"), _("Plot velocity trajectories"));
    connections.add(
        velRenderingToggle->sigToggled().connect(
            bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    accRenderingToggle = self->addToggleButton(QIcon(":/Base/icons/accgraph.png"), _("Plot acceleration trajectories"));
    connections.add(
        accRenderingToggle->sigToggled().connect(
            bind(&GraphBarImpl::onRenderingTypesToggled, this)));

    optionToggle = self->addToggleButton(QIcon(":/Base/icons/setup.png"), _("Show options of the graph view"));
    connections.add(
        optionToggle->sigToggled().connect(
            bind(&GraphBarImpl::onOptionToggled, this)));
    
    optWidgets.push_back(self->addLabel("y-range, 1.0e"));
    verticalValueRangeSpin.setDecimals(1);
    verticalValueRangeSpin.setRange(-99.8, 99.8);
    verticalValueRangeSpin.setSingleStep(0.1);
    verticalValueRangeSpin.setValue(1.0);
    connections.add(
        verticalValueRangeSpin.sigValueChanged().connect(
            bind(&GraphBarImpl::onVerticalValueRangeChanged, this, _1)));
    self->addWidget(&verticalValueRangeSpin);
    optWidgets.push_back(&verticalValueRangeSpin);

    limitValueToggle = self->addToggleButton(
        _("Limit"),
        _("Show the lower / upper limit values on the graph. Trajectory edit is also limited within the values."));
    limitValueToggle->setChecked(true);
    connections.add(
        limitValueToggle->sigToggled().connect(
            bind(&GraphBarImpl::onLimitValueToggled, this, _1)));
    optWidgets.push_back(limitValueToggle);

    optWidgets.push_back(self->addLabel(_("LW")));
    lineWidthSpin.setRange(1, 9);
    lineWidthSpin.setValue(1);
    connections.add(
        lineWidthSpin.sigValueChanged().connect(
            bind(&GraphBarImpl::onLineWidthChanged, this, _1)));
    self->addWidget(&lineWidthSpin);
    optWidgets.push_back(&lineWidthSpin);
    
    gridToggle = self->addToggleButton(_("Grid"), _("Show a grid on the graph background"));
    connections.add(
        gridToggle->sigToggled().connect(
            bind(&GraphBarImpl::onGridToggled, this, _1)));
    optWidgets.push_back(gridToggle);

    gridSizeSpin = new DoubleSpinBox();
    gridSizeSpin->setDecimals(3);
    gridSizeSpin->setRange(-999.999, 999.999);
    gridSizeSpin->setSingleStep(0.001);
    gridSizeSpin->setValue(0.2);
    connections.add(
        gridSizeSpin->sigValueChanged().connect(
            bind(&GraphBarImpl::onGridSizeChanged, this, _1)));
    self->addWidget(gridSizeSpin);
    optWidgets.push_back(gridSizeSpin);
    
    rulersToggle = self->addToggleButton(_("Ruler"), _("Show rulers"));
    rulersToggle->setChecked(true);
    connections.add(
        rulersToggle->sigToggled().connect(
            bind(&GraphBarImpl::onRulersToggled, this, _1)));
    optWidgets.push_back(rulersToggle);

    optWidgets.push_back(self->addSeparator());

    editModeToggle = self->addToggleButton(_("Edit"), _("Edit mode on/off"));
    connections.add(
        editModeToggle->sigToggled().connect(
            bind(&GraphBarImpl::onEditModeToggled, this, _1)));
    optWidgets.push_back(editModeToggle);

    freeLineModeRadio = self->addRadioButton(_("Free"), _("Free line edit mode"));
    freeLineModeRadio->setChecked(true);
    connections.add(
        freeLineModeRadio->sigToggled().connect(
            bind(&GraphBarImpl::onFreeLineModeToggled, this, _1)));
    optWidgets.push_back(freeLineModeRadio);

    lineModeRadio = self->addRadioButton(_("Line"), _("Line edit mode"));
    connections.add(
        lineModeRadio->sigToggled().connect(
            bind(&GraphBarImpl::onLineModeToggled, this, _1)));
    optWidgets.push_back(lineModeRadio);

    optWidgets.push_back(self->addLabel(_("step")));

    controlPointStepSpin = new SpinBox();
    controlPointStepSpin->setRange(1, 999);
    controlPointStepSpin->setValue(1);
    connections.add(
        controlPointStepSpin->sigValueChanged().connect(
            bind(&GraphBarImpl::onControlPointStepOrOffsetChanged, this)));
    self->addWidget(controlPointStepSpin);
    optWidgets.push_back(controlPointStepSpin);

    optWidgets.push_back(self->addLabel(_("offset")));

    controlPointOffsetSpin = new SpinBox();
    controlPointOffsetSpin->setRange(0, 999);
    controlPointOffsetSpin->setValue(0);
    connections.add(
        controlPointOffsetSpin->sigValueChanged().connect(
            bind(&GraphBarImpl::onControlPointStepOrOffsetChanged, this)));
    self->addWidget(controlPointOffsetSpin);
    optWidgets.push_back(controlPointOffsetSpin);

    highlightingControlPointsToggle = self->addToggleButton(_("CP"), _("Highlighting control points"));
    connections.add(
        highlightingControlPointsToggle->sigToggled().connect(
            bind(&GraphBarImpl::onHighlightingControlPointsToggled, this, _1)));
    optWidgets.push_back(highlightingControlPointsToggle);

    optWidgets.push_back(self->addSeparator());

    timeBarSyncToggle = self->addToggleButton(_("Sync"), _("Cursor movement on a graph updates the time bar position"));
    timeBarSyncToggle->setChecked(true);
    connections.add(
        timeBarSyncToggle->sigToggled().connect(
            bind(&GraphBarImpl::onTimeBarSyncToggled, this, _1)));
    optWidgets.push_back(timeBarSyncToggle);

    //self->addLabel("Auto Scroll");

    autoScrollModeCombo = new ComboBox(self);
    autoScrollModeCombo->addItem(_("off"));
    autoScrollModeCombo->addItem(_("cont."));
    autoScrollModeCombo->addItem(_("page"));
    autoScrollModeCombo->setCurrentIndex(1);
    connections.add(
        autoScrollModeCombo->sigCurrentIndexChanged().connect(
            bind(&GraphBarImpl::onAutoScrollModeChanged, this, _1)));
    self->addWidget(autoScrollModeCombo);
    optWidgets.push_back(autoScrollModeCombo);

    self->setEnabled(false);
    onOptionToggled();

    focusedGraphWidget = 0;
}


GraphBar::~GraphBar()
{
    delete impl;
}


void GraphBarImpl::onOptionToggled()
{
    if(optionToggle->isChecked()){
        for(size_t i=0; i < optWidgets.size(); ++i){
            optWidgets[i]->show();
        }
    } else {
        for(size_t i=0; i < optWidgets.size(); ++i){
            optWidgets[i]->hide();
        }
    }
}
        

GraphWidget* GraphBar::focusedGraphWidget()
{
    return impl->focusedGraphWidget;
}


void GraphBar::focus(GraphWidget* graph, bool forceUpdate)
{
    impl->focus(graph, forceUpdate);
}


void GraphBarImpl::focus(GraphWidget* graph, bool forceUpdate)
{
    if(graph && (forceUpdate || (graph != focusedGraphWidget))){

        focusedGraphWidget = graph;
        self->setEnabled(true);

        connections.block();
        
        editModeToggle->setChecked(graph->mode() == GraphWidget::EDIT_MODE);

        GraphWidget::EditMode editMode = graph->editMode();
        if(editMode == GraphWidget::FREE_LINE_MODE){
            freeLineModeRadio->setChecked(true);
        } else if(editMode == GraphWidget::LINE_MODE){
            lineModeRadio->setChecked(true);
        }

        bool org, vel, acc;
        graph->getRenderingTypes(org, vel, acc);
        orgRenderingToggle->setChecked(org);
        velRenderingToggle->setChecked(vel);
        accRenderingToggle->setChecked(acc);

        double lower, upper;
        graph->getVerticalValueRange(lower, upper);
        verticalValueRangeSpin.setValue(upper);

        lineWidthSpin.setValue(graph->getLineWidth());
                                          
        timeBarSyncToggle->setChecked(graph->isTimeBarSyncMode());
        autoScrollModeCombo->setCurrentIndex(graph->autoScrollMode());
        limitValueToggle->setChecked(graph->showsLimits());
        rulersToggle->setChecked(graph->showsRulers());
        gridToggle->setChecked(graph->showsGrid());

        double width, height;
        graph->getGridSize(width, height);
        gridSizeSpin->setValue(width);

        int step, offset;
        graph->getControlPointStep(step, offset);
        controlPointStepSpin->setValue(step);
        controlPointOffsetSpin->setValue(offset);
        highlightingControlPointsToggle->setChecked(graph->highlightsControlPoints());

        connections.unblock();
    }
}


void GraphBar::releaseFocus(GraphWidget* graphWidget)
{
    if(impl->focusedGraphWidget == graphWidget){
        impl->focusedGraphWidget = 0;
        setEnabled(false);
    }
}


void GraphBarImpl::onVerticalValueRangeChanged(double value)
{
    double upper = pow(10.0, value);
    double lower = -upper;
    focusedGraphWidget->setVerticalValueRange(lower, upper);
}


void GraphBarImpl::onLineWidthChanged(int value)
{
    focusedGraphWidget->setLineWidth(value);
}
        

void GraphBarImpl::onRenderingTypesToggled()
{
    focusedGraphWidget->setRenderingTypes
        (orgRenderingToggle->isChecked(), velRenderingToggle->isChecked(), accRenderingToggle->isChecked());
}


void GraphBarImpl::onLimitValueToggled(bool on)
{
    focusedGraphWidget->showLimits(on);
}


void GraphBarImpl::onGridToggled(bool on)
{
    focusedGraphWidget->showGrid(on);
}


void GraphBarImpl::onGridSizeChanged(double size)
{
    focusedGraphWidget->setGridSize(size, size);
}


void GraphBarImpl::onRulersToggled(bool on)
{
    focusedGraphWidget->showRulers(on);
}


void GraphBarImpl::onTimeBarSyncToggled(bool on)
{
    focusedGraphWidget->setTimeBarSyncMode(on);
}


void GraphBarImpl::onAutoScrollModeChanged(int index)
{
    focusedGraphWidget->setAutoScrollMode((GraphWidget::ScrollMode)index);
}


void GraphBarImpl::onEditModeToggled(bool on)
{
    GraphWidget::Mode mode = on ? GraphWidget::EDIT_MODE : GraphWidget::VIEW_MODE;
    focusedGraphWidget->changeMode(mode);
}


void GraphBarImpl::onFreeLineModeToggled(bool on)
{
    if(on){
        focusedGraphWidget->changeEditMode(GraphWidget::FREE_LINE_MODE);
    }
}


void GraphBarImpl::onLineModeToggled(bool on)
{
    if(on){
        focusedGraphWidget->changeEditMode(GraphWidget::LINE_MODE);
    }
}


void GraphBarImpl::onControlPointStepOrOffsetChanged()
{
    focusedGraphWidget->setControlPointStep(
        controlPointStepSpin->value(), controlPointOffsetSpin->value());
}


void GraphBarImpl::onHighlightingControlPointsToggled(bool on)
{
    focusedGraphWidget->highlightControlPoints(on);
}
