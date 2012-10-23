/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SCENE_BODY_H_INCLUDED
#define CNOID_BODYPLUGIN_SCENE_BODY_H_INCLUDED

#include <boost/dynamic_bitset.hpp>
#include <cnoid/SceneObject>
#include "BodyItem.h"
#include "exportdecl.h"

namespace cnoid {

    class SceneBodyImpl;

    class CNOID_EXPORT SceneBody : public SceneObject
    {
      public:
        SceneBody(BodyItemPtr bodyItem);

        void setLinkVisibilities(const boost::dynamic_bitset<>& visibilities);

        void showCenterOfMass(bool on);
        bool isCenterOfMassVisible() const;

        void showZmp(bool on);
        bool isZmpVisible() const;
		  
        Link* getPointedSceneLink();
        osg::ref_ptr<osg::Node> getPointedShapeNode();
		  
      protected:

        virtual ~SceneBody();
        bool createSceneLinks();

        virtual void onAttachedToScene();
        virtual void onDetachedFromScene();

        virtual bool onKeyPressEvent(const SceneViewEvent& event);
        virtual bool onKeyReleaseEvent(const SceneViewEvent& event);
        virtual bool onButtonPressEvent(const SceneViewEvent& event);
        virtual bool onButtonReleaseEvent(const SceneViewEvent& event);
        virtual bool onDoubleClickEvent(const SceneViewEvent& event);
        virtual bool onPointerMoveEvent(const SceneViewEvent& event);
        virtual void onPointerLeaveEvent(const SceneViewEvent& event);
        virtual void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
        virtual void onSceneModeChanged();
        virtual bool onUndoRequest();
        virtual bool onRedoRequest();
            
      private:

        SceneBodyImpl* impl;

        friend class SceneBodyImpl;
    };
            
    typedef osg::ref_ptr<SceneBody> SceneBodyPtr;
}
    
#endif
