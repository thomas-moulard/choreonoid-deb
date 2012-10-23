/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SceneWorld.h"
#include "OsgCollision.h"
#include "WorldItem.h"
#include <map>
#include <iostream>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <osg/Geode>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/SceneView>
#include <cnoid/MessageView>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}


SceneWorld::SceneWorld(WorldItemPtr worldItem)
    : os(MessageView::mainInstance()->cout()),
      worldItem(worldItem)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorld::SceneWorld()" << endl;
    }
            
    osgCollision = new OsgCollision();
    osgCollision->setColdetPairs(worldItem->coldetPairs);

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(osgCollision.get());

    addChild(geode);
}


SceneWorld::~SceneWorld()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorld::~SceneWorld()" << endl;
    }
}


void SceneWorld::onAttachedToScene()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorld::onAttachedToScene()" << endl;
    }
    
    connectionWithSigCollisionsUpdated = 
        worldItem->sigCollisionsUpdated().connect(
            bind(&SceneWorld::onCollisionsUpdated, this));
}


void SceneWorld::onDetachedFromScene()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorld::onDetachedFromScene()" << endl;
    }
    
    connectionWithSigCollisionsUpdated.disconnect();
}


void SceneWorld::onCollisionsUpdated()
{
    osgCollision->dirtyDisplayList();
    requestRedraw();
}


SceneWorldManager::SceneWorldManager()
    : os(MessageView::mainInstance()->cout())
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorldManager::SceneWorldManager()" << endl;
    }
    
    itemTreeView = ItemTreeView::mainInstance();
    sceneView = SceneView::mainInstance();

    RootItem::mainInstance()->sigItemAdded().connect(
        bind(&SceneWorldManager::onItemAdded, this, _1));
}


SceneWorldManager::~SceneWorldManager()
{

}


void SceneWorldManager::onItemAdded(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorldManager::onItemAdded()" << endl;
    }
    
    if(WorldItem* worldItem = dynamic_cast<WorldItem*>(item)){

        worldItem->sigDetachedFromRoot().connect(
            bind(&SceneWorldManager::onWorldItemDetached, this, worldItem));

        itemTreeView->sigCheckToggled(worldItem).connect(
            bind(&SceneWorldManager::onWorldItemCheckToggled, this, worldItem, _1));
        
        if(itemTreeView->isItemChecked(worldItem)){
            showSceneWorld(worldItem, true);
        }
    }
}


/**
   @todo do disconnecting operation
*/
void SceneWorldManager::onWorldItemDetached(WorldItem* worldItem)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorldManager::onWorldItemDetached()" << endl;
    }
    
    showSceneWorld(worldItem, false);
}


void SceneWorldManager::onWorldItemCheckToggled(WorldItem* worldItem, bool isChecked)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorldManager::onWorldItemShown()" << endl;
    }

    showSceneWorld(worldItem, isChecked);
}


void SceneWorldManager::showSceneWorld(WorldItem* worldItem, bool show)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWorldManager::showSceneWorld()" << endl;
    }

    SceneWorldMap::iterator p = sceneWorlds.find(worldItem);

    if(!show && p != sceneWorlds.end()){
        sceneView->removeSceneObject(p->second.get());
        sceneWorlds.erase(p);
        sceneView->requestRedraw();
        
    } else if(show && p == sceneWorlds.end()){
        SceneWorldPtr sceneWorld = new SceneWorld(worldItem);
        sceneWorlds[worldItem] = sceneWorld;
        sceneView->addSceneObject(sceneWorld.get());
        sceneView->requestRedraw();
    }
}
