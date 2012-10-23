/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SCENE_WORLD_H_INCLUDED
#define CNOID_BODYPLUGIN_SCENE_WORLD_H_INCLUDED

#include <map>
#include <iosfwd>
#include <boost/signals.hpp>
#include <cnoid/SceneObject>
#include "WorldItem.h"
#include "OsgCollision.h"
#include "exportdecl.h"

namespace cnoid {

    class SceneView;
    class ItemTreeView;
    
    /**
       The role of this class is the visualization of the WorldItem state.
       The bodies contained in a world are visualized by the SceneBody class,
       so that this class visualizes the remaining objects such as the collisions
       between bodies.
    */
    class CNOID_EXPORT SceneWorld : public SceneObject
    {
    public:
        SceneWorld(WorldItemPtr worldItem);

    protected:
        virtual ~SceneWorld();

        virtual void onAttachedToScene();
        virtual void onDetachedFromScene();

    private:
        std::ostream& os;
        WorldItemPtr worldItem;
        osg::ref_ptr<OsgCollision> osgCollision;
        boost::signals::connection connectionWithSigCollisionsUpdated;

        void onCollisionsUpdated();
    };

    typedef osg::ref_ptr<SceneWorld> SceneWorldPtr;


    class SceneWorldManager
    {
    public:
        SceneWorldManager();
        ~SceneWorldManager();
            
    private:
        std::ostream& os;
            
        typedef std::map<WorldItemPtr, SceneWorldPtr> SceneWorldMap;
        SceneWorldMap sceneWorlds;

        SceneView* sceneView;
        ItemTreeView* itemTreeView;

        void onItemAdded(Item* item);
        void onWorldItemDetached(WorldItem* worldItem);
        void onWorldItemCheckToggled(WorldItem* worldItem, bool isChecked);
        void showSceneWorld(WorldItem* worldItem, bool show);
    };
}
    
#endif
