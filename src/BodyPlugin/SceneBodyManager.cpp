/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SceneBodyManager.h"
#include "SceneBody.h"
#include "BodyItem.h"
#include "LinkSelectionView.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/SceneView>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <map>
#include <iostream>
#include <cassert>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;

    SceneBodyManager* instance_ = 0;
}

namespace cnoid {

    class SBMImpl : public boost::signals::trackable
    {
    public:
        SBMImpl(ExtensionManager& ext);

        struct SceneBodyInfo {
            BodyItem* bodyItem;
            SceneBodyPtr sceneBody;
            bool isShown;
            signals::connection cSigDetachedFromRoot;
            signals::connection cSigCheckToggled;
            signals::connection cSigLinkSelectionChanged;
            ~SceneBodyInfo(){
                cSigDetachedFromRoot.disconnect();
                cSigCheckToggled.disconnect();
                cSigLinkSelectionChanged.disconnect();
            }
        };

        typedef list< boost::function<SceneBody*(BodyItem*)> > FactoryList;
        FactoryList factories;
    
        typedef map<BodyItemPtr, SceneBodyInfo> SceneBodyInfoMap;
        SceneBodyInfoMap sceneBodyInfoMap;

        SceneView* sceneView;
        ItemTreeView* itemTreeView;

        Action* onlySelectedLinkCheck;

        void onItemAdded(Item* item);
        void onBodyItemDetached(BodyItem* item);
        void showBodyItem(SceneBodyInfo* info, bool show);
        void onOnlySelectedLinkToggled();
        void onLinkSelectionChanged(SceneBodyInfo* info);
        bool store(Archive& archive);
        void restore(const Archive& archive);
    };


    class FactoryHolderImpl : public SceneBodyManager::FactoryHolder
    {
    public:
        FactoryHolderImpl(SBMImpl::FactoryList& factories, SBMImpl::FactoryList::iterator iter)
            : factories(factories), iter(iter) { }
        ~FactoryHolderImpl() { factories.erase(iter); }
    private:
        SBMImpl::FactoryList& factories;
        SBMImpl::FactoryList::iterator iter;
    };
}


SceneBodyManager* SceneBodyManager::instance()
{
    return instance_;
}


SceneBodyManager::SceneBodyManager(ExtensionManager& ext)
{
    assert(!instance_);
    impl = new SBMImpl(ext);
    instance_ = this;
}


SBMImpl::SBMImpl(ExtensionManager& ext)
{
    sceneView = SceneView::mainInstance();

    RootItem::mainInstance()->sigItemAdded().connect(bind(&SBMImpl::onItemAdded, this, _1));

    itemTreeView = ItemTreeView::mainInstance();

    ext.connectProjectArchiver(
        "SceneBodyManager", bind(&SBMImpl::store, this, _1), bind(&SBMImpl::restore, this, _1));

    onlySelectedLinkCheck = ext.menuManager().setPath("/Options/Scene View").addCheckItem(_("Show only selected links"));
    onlySelectedLinkCheck->setChecked(false);
    onlySelectedLinkCheck->sigToggled().connect(bind(&SBMImpl::onOnlySelectedLinkToggled, this));
}


SceneBodyManager::~SceneBodyManager()
{
    delete impl;
}


/**
   @param factory A factory function object that creates a scene body instance of a customized sub class.
   In order to avoid the collision between the multiple factories,
   it is desirable that the factory function checks the actual type of a BodyItem instance and
   a customized scene body is only returned if the type matches. Example code is as follows:
   \code
   SceneBody* factory(BodyItem* bodyItem)
   {
       BodyItemEx* ex = dynamic_cast<BodyItemEx*>(bodyItem);
       if(ex){
           return new SceneBodyEx(ex);
       }
       return 0;
   }
   \endcode
   
   @return A holder object of the registered factory. The destructor of the holder object unregisters the factory.
   This object is usually passed to 'ExtensionManager::manage()' function so that the factory can be
   automatically removed when the plugin is finalized.
*/
SceneBodyManager::FactoryHolder* SceneBodyManager::addSceneBodyFactory(boost::function<SceneBody*(BodyItem*)> factory)
{
    impl->factories.push_front(factory);
    return new FactoryHolderImpl(impl->factories, --impl->factories.end());
}


void SBMImpl::onItemAdded(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "SBMImpl::onItemAdded()" << endl;
    }
    
    if(BodyItem* bodyItem = dynamic_cast<BodyItem*>(item)){

        SceneBodyInfo* info = &sceneBodyInfoMap[bodyItem];

        info->bodyItem = bodyItem;
        if(!info->sceneBody){
            for(FactoryList::iterator p = factories.begin(); p != factories.end(); ++p){
                info->sceneBody = (*p)(bodyItem);
                if(info->sceneBody.valid()){
                    break;
                }
            }
        }
        if(!info->sceneBody){
            info->sceneBody = new SceneBody(bodyItem);
        }

        info->isShown = false;

        info->cSigDetachedFromRoot = bodyItem->sigDetachedFromRoot().connect(
            bind(&SBMImpl::onBodyItemDetached, this, bodyItem));

        info->cSigCheckToggled = itemTreeView->sigCheckToggled(bodyItem).connect(
            bind(&SBMImpl::showBodyItem, this, info, _1));

        if(itemTreeView->isItemChecked(bodyItem)){
            showBodyItem(info, true);
        }
    }
}


void SBMImpl::onBodyItemDetached(BodyItem* item)
{
    if(TRACE_FUNCTIONS){
        cout << "SBMImpl::onBodyItemDetached()" << endl;
    }

    SceneBodyInfoMap::iterator p = sceneBodyInfoMap.find(item);
    if(p != sceneBodyInfoMap.end()){
        SceneBodyInfo* info = &p->second;
        showBodyItem(info, false);
        sceneBodyInfoMap.erase(p);
    }
}


void SBMImpl::showBodyItem(SceneBodyInfo* info, bool show)
{
    if(TRACE_FUNCTIONS){
        cout << "SBMImpl::showBodyItem()" << endl;
    }

    if(info->isShown && !show){
        info->cSigLinkSelectionChanged.disconnect();
        sceneView->removeSceneObject(info->sceneBody.get());
        info->isShown = false;
        sceneView->requestRedraw();
        
    } else if(!info->isShown && show){
        sceneView->addSceneObject(info->sceneBody.get());
        info->isShown = true;

        info->cSigLinkSelectionChanged =
            LinkSelectionView::mainInstance()->sigSelectionChanged(info->bodyItem).connect(
                bind(&SBMImpl::onLinkSelectionChanged, this, info));
        onLinkSelectionChanged(info);

        sceneView->requestRedraw();
    }
}


void SBMImpl::onOnlySelectedLinkToggled()
{
    SceneBodyInfoMap::iterator p;
    for(p = sceneBodyInfoMap.begin(); p != sceneBodyInfoMap.end(); ++p){
        if(onlySelectedLinkCheck->isChecked()){
            onLinkSelectionChanged(&p->second);
        } else {
            dynamic_bitset<> visibilities(p->first->body()->numLinks(), true);
            p->second.sceneBody->setLinkVisibilities(visibilities);
        }
    }
}


void SBMImpl::onLinkSelectionChanged(SceneBodyInfo* info)
{
    if(onlySelectedLinkCheck->isChecked()){
        info->sceneBody->setLinkVisibilities(
            LinkSelectionView::mainInstance()->getLinkSelection(info->bodyItem));
    }
}


bool SBMImpl::store(Archive& archive)
{
    YamlSequencePtr states = new YamlSequence();
    
    SceneBodyInfoMap::iterator p;
    for(p = sceneBodyInfoMap.begin(); p != sceneBodyInfoMap.end(); ++p){
        BodyItem* bodyItem = p->first.get();
        int id = archive.getItemId(bodyItem);
        if(id >= 0){
            SceneBodyPtr sceneBody = p->second.sceneBody;
            YamlMappingPtr state = new YamlMapping();
            state->write("bodyItem", id);
            state->write("editable", sceneBody->isEditable());
            state->write("centerOfMass", sceneBody->isCenterOfMassVisible());
            state->write("zmp", sceneBody->isZmpVisible());
            states->append(state);
        }
    }

    if(!states->empty()){
        archive.insert("sceneBodies", states);
        return true;
    }

    return false;
}


void SBMImpl::restore(const Archive& archive)
{
    YamlSequence& states = *archive["sceneBodies"].toSequence();
    for(int i=0; i < states.size(); ++i){
        YamlMapping* state = states[i].toMapping();
        BodyItem* bodyItem = archive.findItem<BodyItem>(state->get("bodyItem", -1));
        if(bodyItem){
            SceneBodyInfoMap::iterator p = sceneBodyInfoMap.find(bodyItem);
            if(p != sceneBodyInfoMap.end()){
                SceneBodyPtr sceneBody = p->second.sceneBody;
                sceneBody->setEditable(state->get("editable", sceneBody->isEditable()));
                sceneBody->showCenterOfMass(state->get("centerOfMass", sceneBody->isCenterOfMassVisible()));
                sceneBody->showZmp(state->get("zmp", sceneBody->isZmpVisible()));
            }
        }
    }
}
