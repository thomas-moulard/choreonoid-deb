/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SCENE_VIEW_H_INCLUDED
#define CNOID_GUIBASE_SCENE_VIEW_H_INCLUDED

#include <cnoid/View>
#include "exportdecl.h"

namespace osg {
    class Camera;
    class Node;
}

namespace cnoid {

    class ExtensionManager;
    class OsgViewer;
    class SceneObject;
    class SceneViewImpl;

    class CNOID_EXPORT SceneView : public View
    {
      public:
        static void initialize(ExtensionManager* ext);
        static SceneView* mainInstance();
        
        SceneView();
        virtual ~SceneView();
        
        OsgViewer* viewer();
        
        void requestRedraw();
        
        void addSceneObject(SceneObject* object);
        void removeSceneObject(SceneObject* object);

        void toggleFloorGrid(bool on);
        void toggleShadow(bool on);

        /**
           \todo Test to provide this flexible event signal
        */
        //SignalProxy<boost::signal<bool(SceneObject* object, SceneViewEvent* event)> > sigEvent;

        osg::Camera* getCameraControl();
        bool releaseCameraControl(osg::Camera* camera);

        void addPreprocessingNode(osg::Node* node);
        bool removePreprocessingNode(osg::Node* node);

        bool saveImage(const std::string& filename);

        void setScreenSize(int width, int height);

        void updateIndicator(const std::string& text);

      protected:

        virtual QWidget* indicatorOnInfoBar();
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);

      private:
        friend class SceneViewImpl;
        SceneViewImpl* impl;
    };

}

#endif
