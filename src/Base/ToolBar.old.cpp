/**
   @author Shin'ichiro Nakaoka
*/

#include "ToolBar.h"
#include <cnoid/MainWindow>

using namespace std;
using namespace cnoid;


ToolBar::ToolBar(const QString& title)
    : QToolBar(title)
{
    setObjectName(title);

    radioGroup = 0;
    isNewRadioGroupRequested = true;

    toolBarArea = 0;
    desiredX = 0;
    layoutPriority = 0;
    isStretchable_ = false;
}


ToolBar::~ToolBar()
{

}


Action* ToolBar::addButton(const QString& text, const QString& tooltip)
{
    Action* action = new Action(text, this);
    if(!tooltip.isEmpty()){
        action->setToolTip(tooltip);
    }
    addAction(action);
    return action;
}


Action* ToolBar::addButton(const QIcon& icon, const QString& tooltip)
{
    Action* action = new Action(icon, this);
    if(!tooltip.isEmpty()){
        action->setToolTip(tooltip);
    }
    addAction(action);
    return action;
}


Action* ToolBar::addButton(const char* const* xpm, const QString& tooltip)
{
    return addButton(QIcon(QPixmap(xpm)), tooltip);
}


Action* ToolBar::addToggleButton(const QString& text, const QString& tooltip)
{
    Action* action = addButton(text, tooltip);
    action->setCheckable(true);
    return action;
}


Action* ToolBar::addToggleButton(const QIcon& icon, const QString& tooltip)
{
    Action* action = addButton(icon, tooltip);
    action->setCheckable(true);
    return action;
}


Action* ToolBar::addToggleButton(const char* const* xpm, const QString& tooltip)
{
    Action* action = addButton(xpm, tooltip);
    action->setCheckable(true);
    return action;
}


void ToolBar::requestNewRadioGroup()
{
    radioGroup = 0;
    isNewRadioGroupRequested = true;
}


QActionGroup* ToolBar::currentRadioGroup()
{
    if(isNewRadioGroupRequested){
        radioGroup = new QActionGroup(this);
        isNewRadioGroupRequested = false;
    }
    return radioGroup;
}


void ToolBar::setRadioButton(Action* action)
{
    action->setCheckable(true);
    action->setActionGroup(currentRadioGroup());
}


Action* ToolBar::addRadioButton(const QString& text, const QString& tooltip)
{
    Action* action = addButton(text, tooltip);
    setRadioButton(action);
    return action;
}


Action* ToolBar::addRadioButton(const QIcon& icon, const QString& tooltip)
{
    Action* action = addButton(icon, tooltip);
    setRadioButton(action);
    return action;
}


Action* ToolBar::addRadioButton(const char* const* xpm, const QString& tooltip)
{
    Action* action = addButton(xpm, tooltip);
    setRadioButton(action);
    return action;
}


QLabel* ToolBar::addLabel(const QString& text)
{
    QLabel* label = new QLabel(text, this);
    addWidget(label);
    return label;
}


void ToolBar::addSpace(int size)
{

}


int ToolBar::bestStretchWidth()
{
    int bestWidth = sizeHint().width();

    if(isStretchable() && bestWidth > 0){
        bestWidth = bestWidth * 2;
    }
    return bestWidth;
}


bool ToolBar::storeState(Archive& archive)
{
    return true;
}


bool ToolBar::restoreState(const Archive& archive)
{
    return true;
}
