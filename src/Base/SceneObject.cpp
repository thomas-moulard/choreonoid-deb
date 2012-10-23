/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneObject.h"
#include "SceneView.h"
#include "MessageView.h"
#include <osgDB/ReadFile>

using namespace cnoid;


SceneViewEvent::SceneViewEvent()
    : point_(0.0, 0.0, 0.0)
{
    x_ = 0.0;
    y_ = 0.0;
    key_ = 0;
    modKeyMask_ = 0;
    button_ = 0;
    scrollingMotion_ = osgGA::GUIEventAdapter::SCROLL_NONE;
    scrollingDelta_ = 0.0;
}


SceneViewEvent::SceneViewEvent(const SceneViewEvent& org)
    : camera_(org.camera_),
      point_(org.point_),
      path_(org.path_)
{
    x_ = org.x_;
    y_ = org.y_;
    key_ = org.key_;
    modKeyMask_ = org.modKeyMask_;
    button_ = org.button_;
    scrollingMotion_ = org.scrollingMotion_;
    scrollingDelta_ = org.scrollingDelta_;
}


void SceneViewEvent::updateIndicator(const std::string& text) const
{
    sceneView->updateIndicator(text);
}


SceneObject::SceneObject()
{
    isActive_ = false;
    isEditable_ = false;
    sceneMode_ = VIEW_MODE;
}


SceneObject::SceneObject(const SceneObject& org, const osg::CopyOp& copyop)
    : osg::Group(org),
      isEditable_(org.isEditable_),
      sceneMode_(org.sceneMode_)
{
    isActive_ = false;
}


SceneObject::~SceneObject()
{
    if(isActive_){
        onDetachedFromScene();
    }
}


const char* SceneObject::libraryName() const
{
    return "cnoid";
}


const char* SceneObject::className() const
{
    return "SceneObject";
}


SceneObject::ReadResult SceneObject::load(const std::string filename)
{
    MessageView::mainInstance()->beginStdioRedirect();
    
    /**
       Probably error message can be obtained by using osgDB::Registry class.
       First, get a ReaderWriter object with getReaderWriterForExtension(),
       and use it to load a file.
       Messages can be obtained from ReadResult object.
       Let's see the implementation of osgDB::readNodeFile(), too.
    */
    osg::Node* node = osgDB::readNodeFile(filename);
    
    if(node){
        addChild(node);
        return osgDB::ReaderWriter::ReadResult(osgDB::ReaderWriter::ReadResult::FILE_LOADED);
    }

    return osgDB::ReaderWriter::ReadResult(osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE);

    MessageView::mainInstance()->endStdioRedirect();
}


void SceneObject::setEditable(bool on)
{
    isEditable_ = on;
}


bool SceneObject::isEditable()
{
    return isEditable_;
}


void SceneObject::onAttachedToScene()
{
    
}


void SceneObject::onDetachedFromScene()
{

}


bool SceneObject::onKeyPressEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneObject::onKeyReleaseEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneObject::onButtonPressEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneObject::onButtonReleaseEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneObject::onDoubleClickEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneObject::onPointerMoveEvent(const SceneViewEvent& event)
{
    return false;
}


void SceneObject::onPointerLeaveEvent(const SceneViewEvent& event)
{

}


bool SceneObject::onScrollEvent(const SceneViewEvent& event)
{
    return false;
}


void SceneObject::onSceneModeChanged()
{

}


bool SceneObject::onUndoRequest()
{
    return false;
}


bool SceneObject::onRedoRequest()
{
    return false;
}

       
void SceneObject::onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager)
{

}
