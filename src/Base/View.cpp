/**
   @author Shin'ichiro Nakaoka
*/

#include "View.h"
#include "MainWindow.h"
#include <QLayout>

#include <iostream>

using namespace std;
using namespace cnoid;


View::View()
{
    //setSizePolicy(QSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored));
    isActive_ = false;
    isManagedByMainWindow = false;
    defaultLayoutArea_ = CENTER;
}


View::~View()
{
    if(isActive_){
        onDeactivated();
    }

    if(isManagedByMainWindow){
        MainWindow::instance()->removeView(this);
    }
}


void View::setName(const QString& name)
{
    setObjectName(name);
    setWindowTitle(name);
}


bool View::isActive() const
{
    return isActive_;
}


void View::showEvent(QShowEvent* event)
{
    if(!isActive_){
        isActive_ = true;
        onActivated();
        sigActivated_();
    }
}
    
    
void View::hideEvent(QHideEvent* event)
{
    if(isActive_){
        isActive_ = false;
        onDeactivated();
        sigDeactivated_();
    }
}


/**
   Virtual function which is called when the view becomes visible on the main window.

   @note In the current implementation, this function may be continuously called
   two or three times when the perspective changes, and the number of calles does not
   necessarily corresponds to the number of 'onDeactivated()' calles.

   @todo improve the behavior written as note
*/
void View::onActivated()
{
    
}


void View::onDeactivated()
{

}


void View::setDefaultLayoutArea(LayoutArea area)
{
    defaultLayoutArea_ = area;
}


View::LayoutArea View::defaultLayoutArea() const
{
    return defaultLayoutArea_;
}


void View::setLayout(QLayout* layout)
{
    const int margin = 0;
    layout->setContentsMargins(margin, margin, margin, margin);
    QWidget::setLayout(layout);
}


QWidget* View::indicatorOnInfoBar()
{
    return 0;
}


bool View::storeState(Archive& archive)
{
    return true;
}


bool View::restoreState(const Archive& archive)
{
    return true;
}
