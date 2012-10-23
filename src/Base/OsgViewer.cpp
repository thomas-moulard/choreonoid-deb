
#include "OsgViewer.h"
#include <QWheelEvent>


#include <iostream>

using namespace std;
using namespace osgGA;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}


OsgWidget::OsgWidget(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
    : QGLWidget(parent, shareWidget, f)
{
    graphicsWindow_ = new osgViewer::GraphicsWindowEmbedded(0, 0, width(), height());
    setFocusPolicy(Qt::WheelFocus);
}


OsgWidget::~OsgWidget()
{
    if(TRACE_FUNCTIONS){
        cout << "OsgWidget::~OsgWidget()" << endl;
    }
}


void OsgWidget::resizeGL(int width, int height)
{
    if(width > 0 && height > 0){
        graphicsWindow_->getEventQueue()->windowResize(0, 0, width, height);
        graphicsWindow_->resized(0, 0, width, height);
    }
}


/**
   returns a osgGA::GUIEventAdapter::MouseButtonMask value
*/
int OsgWidget::convertToOsgMouseButtonMask(int qtMouseButtons)
{
    int mask = 0;

    if(qtMouseButtons & Qt::LeftButton){
        mask |= GUIEventAdapter::LEFT_MOUSE_BUTTON;
    }
    if(qtMouseButtons & Qt::MidButton){
        mask |= GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
    }
    if(qtMouseButtons & Qt::RightButton){
        mask |= GUIEventAdapter::RIGHT_MOUSE_BUTTON;
    }

    return mask;
}


/**
   returns a osgGA::GUIEventAdapter::KeySymbol value
*/
int OsgWidget::convertToOsgKeySymbol(int qtKeySymbol)
{
    switch(qtKeySymbol){

    case Qt::Key_Escape:       return GUIEventAdapter::KEY_Escape;
    case Qt::Key_Tab:          return GUIEventAdapter::KEY_Tab;
    case Qt::Key_Backspace:    return GUIEventAdapter::KEY_BackSpace;
    case Qt::Key_Return:       return GUIEventAdapter::KEY_Return;
    case Qt::Key_Enter:        return GUIEventAdapter::KEY_Return;
    case Qt::Key_Insert:       return GUIEventAdapter::KEY_Insert;
    case Qt::Key_Delete:       return GUIEventAdapter::KEY_Delete;
    case Qt::Key_Pause:        return GUIEventAdapter::KEY_Pause;
    case Qt::Key_Print:        return GUIEventAdapter::KEY_Print;
    case Qt::Key_SysReq:       return GUIEventAdapter::KEY_Sys_Req;
    case Qt::Key_Clear:        return GUIEventAdapter::KEY_Clear;
    case Qt::Key_Home:         return GUIEventAdapter::KEY_Home;
    case Qt::Key_End:          return GUIEventAdapter::KEY_End;
    case Qt::Key_Left:         return GUIEventAdapter::KEY_Left;
    case Qt::Key_Up:           return GUIEventAdapter::KEY_Up;
    case Qt::Key_Right:        return GUIEventAdapter::KEY_Right;
    case Qt::Key_Down:         return GUIEventAdapter::KEY_Down;
    case Qt::Key_PageUp:       return GUIEventAdapter::KEY_Page_Up;
    case Qt::Key_PageDown:     return GUIEventAdapter::KEY_Page_Down;
    case Qt::Key_Shift:        return GUIEventAdapter::KEY_Shift_L;
    case Qt::Key_Control:      return GUIEventAdapter::KEY_Control_L;
    case Qt::Key_Meta:         return GUIEventAdapter::KEY_Meta_L;
    case Qt::Key_Alt:          return GUIEventAdapter::KEY_Alt_L;
    case Qt::Key_CapsLock:     return GUIEventAdapter::KEY_Caps_Lock;
    case Qt::Key_NumLock:      return GUIEventAdapter::KEY_Num_Lock;
    case Qt::Key_ScrollLock:   return GUIEventAdapter::KEY_Scroll_Lock;
    case Qt::Key_F1:           return GUIEventAdapter::KEY_F1;
    case Qt::Key_F2:           return GUIEventAdapter::KEY_F2;
    case Qt::Key_F3:           return GUIEventAdapter::KEY_F3;
    case Qt::Key_F4:           return GUIEventAdapter::KEY_F4;
    case Qt::Key_F5:           return GUIEventAdapter::KEY_F5;
    case Qt::Key_F6:           return GUIEventAdapter::KEY_F6;
    case Qt::Key_F7:           return GUIEventAdapter::KEY_F7;
    case Qt::Key_F8:           return GUIEventAdapter::KEY_F8;
    case Qt::Key_F9:           return GUIEventAdapter::KEY_F9;
    case Qt::Key_F10:          return GUIEventAdapter::KEY_F10;
    case Qt::Key_F11:          return GUIEventAdapter::KEY_F11;
    case Qt::Key_F12:          return GUIEventAdapter::KEY_F12;
    case Qt::Key_F13:          return GUIEventAdapter::KEY_F13;
    case Qt::Key_F14:          return GUIEventAdapter::KEY_F14;
    case Qt::Key_F15:          return GUIEventAdapter::KEY_F15;
    case Qt::Key_F16:          return GUIEventAdapter::KEY_F16;
    case Qt::Key_F17:          return GUIEventAdapter::KEY_F17;
    case Qt::Key_F18:          return GUIEventAdapter::KEY_F18;
    case Qt::Key_F19:          return GUIEventAdapter::KEY_F19;
    case Qt::Key_F20:          return GUIEventAdapter::KEY_F20;
    case Qt::Key_F21:          return GUIEventAdapter::KEY_F21;
    case Qt::Key_F22:          return GUIEventAdapter::KEY_F22;
    case Qt::Key_F23:          return GUIEventAdapter::KEY_F23;
    case Qt::Key_F24:          return GUIEventAdapter::KEY_F24;
    case Qt::Key_F25:          return GUIEventAdapter::KEY_F25;
    case Qt::Key_F26:          return GUIEventAdapter::KEY_F26;
    case Qt::Key_F27:          return GUIEventAdapter::KEY_F27;
    case Qt::Key_F28:          return GUIEventAdapter::KEY_F28;
    case Qt::Key_F29:          return GUIEventAdapter::KEY_F29;
    case Qt::Key_F30:          return GUIEventAdapter::KEY_F30;
    case Qt::Key_F31:          return GUIEventAdapter::KEY_F31;
    case Qt::Key_F32:          return GUIEventAdapter::KEY_F32;
    case Qt::Key_F33:          return GUIEventAdapter::KEY_F33;
    case Qt::Key_F34:          return GUIEventAdapter::KEY_F34;
    case Qt::Key_F35:          return GUIEventAdapter::KEY_F35;
    case Qt::Key_Super_L:      return GUIEventAdapter::KEY_Super_L;
    case Qt::Key_Super_R:      return GUIEventAdapter::KEY_Super_R;
    case Qt::Key_Menu:         return GUIEventAdapter::KEY_Menu;
    case Qt::Key_Hyper_L:      return GUIEventAdapter::KEY_Hyper_L;
    case Qt::Key_Hyper_R:      return GUIEventAdapter::KEY_Hyper_R;
    case Qt::Key_Help:         return GUIEventAdapter::KEY_Help;
    case Qt::Key_Space:        return GUIEventAdapter::KEY_Space;
    case Qt::Key_Exclam:       return 0x21;
    case Qt::Key_QuoteDbl:     return 0x22;
    case Qt::Key_NumberSign:   return 0x23;
    case Qt::Key_Dollar:       return 0x24;
    case Qt::Key_Percent:      return 0x25;
    case Qt::Key_Ampersand:    return 0x26;
    case Qt::Key_Apostrophe:   return 0x27;
    case Qt::Key_ParenLeft:    return 0x28;
    case Qt::Key_ParenRight:   return 0x29;
    case Qt::Key_Asterisk:     return 0x2a;
    case Qt::Key_Plus:         return 0x2b;
    case Qt::Key_Comma:        return 0x2c;
    case Qt::Key_Minus:        return 0x2d;
    case Qt::Key_Period:       return 0x2e;
    case Qt::Key_Slash:        return 0x2f;
    case Qt::Key_0:            return 0x30;
    case Qt::Key_1:            return 0x31;
    case Qt::Key_2:            return 0x32;
    case Qt::Key_3:            return 0x33;
    case Qt::Key_4:            return 0x34;
    case Qt::Key_5:            return 0x35;
    case Qt::Key_6:            return 0x36;
    case Qt::Key_7:            return 0x37;
    case Qt::Key_8:            return 0x38;
    case Qt::Key_9:            return 0x39;
    case Qt::Key_Colon:        return 0x3a;
    case Qt::Key_Semicolon:    return 0x3b;
    case Qt::Key_Less:         return 0x3c;
    case Qt::Key_Equal:        return 0x3d;
    case Qt::Key_Greater:      return 0x3e;
    case Qt::Key_Question:     return 0x3f;
    case Qt::Key_At:           return 0x40;
    case Qt::Key_A:            return 0x41;
    case Qt::Key_B:            return 0x42;
    case Qt::Key_C:            return 0x43;
    case Qt::Key_D:            return 0x44;
    case Qt::Key_E:            return 0x45;
    case Qt::Key_F:            return 0x46;
    case Qt::Key_G:            return 0x47;
    case Qt::Key_H:            return 0x48;
    case Qt::Key_I:            return 0x49;
    case Qt::Key_J:            return 0x4a;
    case Qt::Key_K:            return 0x4b;
    case Qt::Key_L:            return 0x4c;
    case Qt::Key_M:            return 0x4d;
    case Qt::Key_N:            return 0x4e;
    case Qt::Key_O:            return 0x4f;
    case Qt::Key_P:            return 0x50;
    case Qt::Key_Q:            return 0x51;
    case Qt::Key_R:            return 0x52;
    case Qt::Key_S:            return 0x53;
    case Qt::Key_T:            return 0x54;
    case Qt::Key_U:            return 0x55;
    case Qt::Key_V:            return 0x56;
    case Qt::Key_W:            return 0x57;
    case Qt::Key_X:            return 0x58;
    case Qt::Key_Y:            return 0x59;
    case Qt::Key_Z:            return 0x5a;
    case Qt::Key_BracketLeft:  return 0x5b;
    case Qt::Key_Backslash:    return 0x5c;
    case Qt::Key_BracketRight: return 0x5d;
    case Qt::Key_AsciiCircum:  return 0x5e;
    case Qt::Key_Underscore:   return 0x5f;
    case Qt::Key_QuoteLeft:    return 0x60;
    case Qt::Key_BraceLeft:    return 0x7b;
    case Qt::Key_Bar:          return 0x7c;
    case Qt::Key_BraceRight:   return 0x7d;
    case Qt::Key_AsciiTilde:   return 0x7e;

    default:
        break;
    }
    return 0;
}


/**
   returns a osgGA::GUIEventAdapter::ModKeyMask value
*/
int OsgWidget::convertToOsgModKeyMask(int qtModifiers)
{
    int mask = 0;
    if(qtModifiers & Qt::ShiftModifier){
        mask |= GUIEventAdapter::MODKEY_SHIFT;
    }
    if(qtModifiers & Qt::ControlModifier){
        mask |= GUIEventAdapter::MODKEY_CTRL;
    }
    if(qtModifiers & Qt::AltModifier){
        mask |= GUIEventAdapter::MODKEY_ALT;
    }
    if(qtModifiers & Qt::MetaModifier){
        mask |= GUIEventAdapter::MODKEY_META;
    }
    return mask;
}


/**
   returns a osgGA::GUIEventAdapter::ScrollingMotion value
*/
int OsgWidget::convertToOsgScrollingMotion(QWheelEvent* event)
{
    int delta = event->delta();
    switch(event->orientation()){
    case Qt::Horizontal:
        if(delta > 0){
            return GUIEventAdapter::SCROLL_RIGHT;
        } else if(delta < 0){
            return GUIEventAdapter::SCROLL_LEFT;
        }
        break;
    case Qt::Vertical:
        if(delta > 0){
            return GUIEventAdapter::SCROLL_UP;
        } else if(delta < 0){
            return GUIEventAdapter::SCROLL_DOWN;
        }
        break;
    }
    return GUIEventAdapter::SCROLL_NONE;
}


OsgViewer::OsgViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
    : OsgWidget(parent, shareWidget, f)
{
    getCamera()->setViewport(new osg::Viewport(0, 0, width(), height()));
    getCamera()->setProjectionMatrixAsPerspective(
        30.0f, static_cast<double>(width()) / static_cast<double>(height()), 1.0f, 10000.0f);
    getCamera()->setGraphicsContext(graphicsWindow());

    setThreadingModel(osgViewer::Viewer::SingleThreaded);

    isCollisionVisibleMode_ = false;
}


OsgViewer::~OsgViewer()
{
    if(TRACE_FUNCTIONS){
        cout << "OsgViewer::~OsgViewer()" << endl;
    }
}


void OsgViewer::requestRedraw()
{
    updateGL();
}


void OsgViewer::paintGL()
{
    frame();
    //updateTraversal();
    //renderingTraversals();
}
