;/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SCENE_OBJECT_H_INCLUDED
#define CNOID_GUIBASE_SCENE_OBJECT_H_INCLUDED

#include <boost/signal.hpp>
#include <osg/Group>
#include <osg/Camera>
#include <osgDB/ReaderWriter>
#include <osgGA/GUIEventAdapter>
#include "exportdecl.h"

namespace cnoid {

    class MenuManager;
    class SceneView;
    class SceneViewImpl;

    class CNOID_EXPORT SceneViewEvent
    {
      public:

        SceneViewEvent(const SceneViewEvent& org);
        
        inline const osg::Vec3d& point() const { return point_; }
        inline const osg::Vec3d& normal() const { return normal_; }
        inline const osg::NodePath& path() const { return path_; }
        inline osg::Camera* camera() const { return camera_.get(); }

        inline double x() const { return x_; }

        /**
           The value is increasing from the bottom to top (the coordinate of the OpenGL viewport).
        */
        inline double y() const { return y_; }

        /**
           @return a osgGA::GUIEventAdapter::KeySymbol value
        */
        inline int key() const { return key_; }

        /**
           @return a osgGA::GUIEventAdapter::MouseButtonMask value
        */
        inline int button() const { return button_; }

        /**
           @return a osgGA::GUIEventAdapter::ModKeyMask value
        */
        inline int modKeyMask() const { return modKeyMask_; }
        
        /**
           @return a osgGA::GUIEventAdapter::ScrollingMotion value
        */
        inline int scrollingMotion() const { return scrollingMotion_; }

        inline double scrollingDelta() const { return scrollingDelta_; }
        
        void updateIndicator(const std::string& message) const;

      private:

        osg::ref_ptr<osg::Camera> camera_;
        osg::Vec3d point_;
        osg::Vec3d normal_;
        osg::NodePath path_;
        double x_;
        double y_;
        int key_;
        int modKeyMask_;
        int button_;
        int scrollingMotion_;
        double scrollingDelta_;
        SceneView* sceneView;

        SceneViewEvent();
        SceneViewEvent& operator=(const SceneViewEvent& org); // disabled

        friend class SceneViewImpl;
    };
    

    class CNOID_EXPORT SceneObject : public osg::Group
    {
      public:

        SceneObject();
        SceneObject(const SceneObject& org, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

        virtual const char* libraryName() const;
        virtual const char* className () const;

        typedef osgDB::ReaderWriter::ReadResult ReadResult;

        ReadResult load(const std::string filename);

        inline bool isActive() { return isActive_; }

        // This should be protected but VC++ requires it to be public. 
        enum SceneMode { VIEW_MODE, EDIT_MODE };

        void setEditable(bool on);
        bool isEditable();

      protected:

        inline SceneMode sceneMode() { return sceneMode_; }

        inline void requestRedraw(){
            sigRedrawRequest(0);
        }
        
        inline void requestRedrawWhenEffectNodeCreated(){
            sigRedrawRequest(EFFECT_NODE_CREATED);
        }

        virtual ~SceneObject();

        virtual void onAttachedToScene();
        virtual void onDetachedFromScene();

        virtual bool onKeyPressEvent(const SceneViewEvent& event);
        virtual bool onKeyReleaseEvent(const SceneViewEvent& event);
        virtual bool onButtonPressEvent(const SceneViewEvent& event);
        virtual bool onButtonReleaseEvent(const SceneViewEvent& event);
        virtual bool onDoubleClickEvent(const SceneViewEvent& event);
        virtual bool onPointerMoveEvent(const SceneViewEvent& event);
        virtual void onPointerLeaveEvent(const SceneViewEvent& event);
        virtual bool onScrollEvent(const SceneViewEvent& event);
        virtual void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
        virtual void onSceneModeChanged();
        virtual bool onUndoRequest();
        virtual bool onRedoRequest();

      private:

        bool isActive_; // in the scene graph ?
        bool isEditable_;
        SceneMode sceneMode_;

        enum RedrawRequestFlag { EFFECT_NODE_CREATED = 1 };

        /// \todo remove this signal and do the same thing with more simple, efficient way
        boost::signal<void(int flag)> sigRedrawRequest;
        
        friend class SceneViewImpl;
    };

    typedef osg::ref_ptr<SceneObject> SceneObjectPtr;
}

#endif
