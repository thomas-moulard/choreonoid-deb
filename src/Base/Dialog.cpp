/**
   @author Shin'ichiro Nakaoka
*/

#include "Dialog.h"

using namespace cnoid;

Dialog::Dialog(QWidget* parent, Qt::WindowFlags f)
 : QDialog(parent, f)
{
    initialize();
}


void Dialog::initialize()
{
    connect(this, SIGNAL(accepted()), this, SLOT(onAccepted()));
    connect(this, SIGNAL(finished(int)), this, SLOT(onFinished(int)));
    connect(this, SIGNAL(rejected()), this, SLOT(onRejected()));
}


void Dialog::onAccepted()
{
    sigAccepted_();
}


void Dialog::onFinished(int result)
{
    sigFinished_(result);
}


void Dialog::onRejected()
{
    sigRejected_();
}
