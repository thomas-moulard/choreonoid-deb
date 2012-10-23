/**
   @author Shin'ichiro NAKAOKA
*/

#include "Action.h"

using namespace cnoid;

Action::Action(QObject* parent)
    : QAction(parent)
{
    initialize();
}


Action::Action(const QString& text, QObject* parent)
    : QAction(text, parent)
{
    initialize();
}


Action::Action(const QIcon& icon, QObject* parent)
    : QAction(parent)
{
    setIcon(icon);
    initialize();
}


Action::Action(const QIcon& icon, const QString& text, QObject* parent)
    : QAction(icon, text, parent)
{
    initialize();
}


void Action::initialize()
{
    connect(this, SIGNAL(triggered(bool)), this, SLOT(onTriggered(bool)));
    connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
}
    

void Action::onTriggered(bool checked)
{
    sigTriggered_();
}


void Action::onToggled(bool checked)
{
    sigToggled_(checked);
}
