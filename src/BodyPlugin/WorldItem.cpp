/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldItem.h"
#include "KinematicsBar.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/RootItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/YamlReader>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/LazyCaller>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/dynamic_bitset.hpp>
#include <vector>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace boost::lambda;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;

}

namespace cnoid {

    class WorldItemImpl
    {
    public:
        WorldItemImpl(WorldItem* self);
        WorldItemImpl(WorldItem* self, const WorldItemImpl& org);
        ~WorldItemImpl();
            
        WorldItem* self;
        ostream& os;

        ItemList<BodyItem> bodyItems;

        signals::connection sigItemTreeChangedConnection;
        ConnectionSet sigKinematicStateChangedConnections;

        bool isCollisionDetectionEnabled;
        LazyCaller updateCollisionsCaller;
        KinematicsBar* kinematicsBar;

        struct BodyItemInfo {
            BodyItemInfo(int numLinks)
                : worldCollisionLinkBitSet(numLinks) {
                kinematicStateChanged = false;
                worldCollisionLinkSetChanged = false;
                numValidPairs = 0;
            }
            typedef dynamic_bitset<> updateFlag;
            bool kinematicStateChanged;
            bool worldCollisionLinkSetChanged;
            boost::dynamic_bitset<> worldCollisionLinkBitSet;
            int numValidPairs;
        };
        typedef map<BodyItem*, BodyItemInfo> BodyItemInfoMap;
        BodyItemInfoMap bodyItemInfoMap;

        boost::signal<void()> sigColdetPairsUpdated;
        boost::signal<void()> sigCollisionsUpdated;

        void init();
        void enableCollisionDetection(bool on);
        void clearColdetLinkPairs();
        void updateColdetLinkPairs(bool forceUpdate);
        void onBodyKinematicStateChanged(BodyItem* bodyItem);
        void updateCollisions(bool forceUpdate);
    };
}


namespace {
    class ColdetLinkPairEx : public ColdetLinkPair
    {
    public:
        WorldItemImpl::BodyItemInfo* info[2];
        
        ColdetLinkPairEx(
            WorldItemImpl::BodyItemInfo& p1, Link* link1,
            WorldItemImpl::BodyItemInfo& p2, Link* link2)
            : ColdetLinkPair(link1, link2) {
            info[0] = &p1;
            info[1] = &p2;
        }
    };
}


void cnoid::initializeWorldItem(ExtensionManager& ext)
{
    ext.itemManager().registerClass<WorldItem>(N_("WorldItem"));
    ext.itemManager().addCreationPanel<WorldItem>();
}


WorldItem::WorldItem()
{
    impl = new WorldItemImpl(this);
}


WorldItemImpl::WorldItemImpl(WorldItem* self)
    : self(self),
      os(MessageView::mainInstance()->cout()),
      updateCollisionsCaller(bind(&WorldItemImpl::updateCollisions, this, false), IDLE_PRIORITY_NORMAL)
{
    isCollisionDetectionEnabled = false;
    init();
}


WorldItem::WorldItem(const WorldItem& org)
    : Item(org)
{
    impl = new WorldItemImpl(this, *org.impl);
}


WorldItemImpl::WorldItemImpl(WorldItem* self, const WorldItemImpl& org)
    : self(self),
      os(org.os),
      updateCollisionsCaller(bind(&WorldItemImpl::updateCollisions, this, false), IDLE_PRIORITY_NORMAL)
{
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    init();
}


void WorldItemImpl::init()
{
    kinematicsBar = KinematicsBar::instance();
}
    

WorldItem::~WorldItem()
{
    delete impl;
}


WorldItemImpl::~WorldItemImpl()
{
    sigKinematicStateChangedConnections.disconnect();
    sigItemTreeChangedConnection.disconnect();
}


void WorldItem::enableCollisionDetection(bool on)
{
    impl->enableCollisionDetection(on);
}


void WorldItemImpl::enableCollisionDetection(bool on)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::enableCollisionDetection(" << on << ")" << endl;
    }

    bool changed = false;
    
    if(isCollisionDetectionEnabled && !on){
        clearColdetLinkPairs();
        sigItemTreeChangedConnection.disconnect();
        isCollisionDetectionEnabled = false;
        changed = true;
        
    } else if(!isCollisionDetectionEnabled && on){
        isCollisionDetectionEnabled = true;
        updateColdetLinkPairs(true);
        sigItemTreeChangedConnection =
            RootItem::mainInstance()->sigTreeChanged().connect(
                bind(&WorldItemImpl::updateColdetLinkPairs, this, false));
        changed = true;
    }

    if(changed){
        sigColdetPairsUpdated();
        self->notifyUpdate();
        sigCollisionsUpdated();
    }
}


bool WorldItem::isCollisionDetectionEnabled()
{
    return impl->isCollisionDetectionEnabled;
}


void WorldItemImpl::clearColdetLinkPairs()
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::clearColdetLinkPairs()" << endl;
    }
    
    self->coldetPairs.clear();
    sigKinematicStateChangedConnections.disconnect();
    bodyItemInfoMap.clear();

    for(size_t i=0; i < bodyItems.size(); ++i){
        BodyItem* item = bodyItems[i];
        size_t n = item->body()->numLinks();
        for(size_t j=0; j < n; ++j){
            item->worldColdetPairsOfLink(j).clear();
        }
        bodyItemInfoMap.insert(make_pair(item, BodyItemInfo(n)));
    }
}    


void WorldItemImpl::updateColdetLinkPairs(bool forceUpdate)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::updateColdetLinkPairs()" << endl;
    }

    if(!forceUpdate){
        ItemList<BodyItem> prevBodyItems = bodyItems;
        bodyItems = self->getBodyItems();
        if(bodyItems == prevBodyItems){
            return;
        }
    } else {
        bodyItems = self->getBodyItems();
    }
        
    clearColdetLinkPairs();

    for(BodyItemInfoMap::iterator p1 = bodyItemInfoMap.begin(); p1 != bodyItemInfoMap.end(); ++p1){

        BodyItem* bodyItem1 = p1->first;
        BodyItemInfo& info1 = p1->second;
        BodyPtr body1 = bodyItem1->body();
        
        for(BodyItemInfoMap::iterator p2 = p1; p2 != bodyItemInfoMap.end(); ++p2){

            BodyItem* bodyItem2 = p2->first;
            BodyItemInfo& info2 = p2->second;
            BodyPtr body2 = bodyItem2->body();
                    
            for(int l1 = 0; l1 < body1->numLinks(); ++l1){
                int l2;
                if(p1 == p2){
                    continue;
                    //l2 = l1 + 1;
                } else {
                    l2 = 0;
                }
                Link* link1 = body1->link(l1);
                std::vector<ColdetLinkPairPtr>& pairs1 = bodyItem1->worldColdetPairsOfLink(l1);
                
                for( ; l2 < body2->numLinks(); ++l2){
                    Link* link2 = body2->link(l2);
                    //if(link1 != link2){
                    if(link1->parent != link2 && link2->parent != link1){
                        std::vector<ColdetLinkPairPtr>& pairs2 = bodyItem2->worldColdetPairsOfLink(l2);
                        ColdetLinkPairPtr linkPair = new ColdetLinkPairEx(info1, link1, info2, link2);
                        self->coldetPairs.push_back(linkPair);
                        pairs1.push_back(linkPair);
                        pairs2.push_back(linkPair);
                        info1.numValidPairs++;
                        info2.numValidPairs++;
                    }
                }
            }
        }
    }

    
    for(BodyItemInfoMap::iterator p = bodyItemInfoMap.begin(); p != bodyItemInfoMap.end(); ++p){
        BodyItemInfo& info = p->second;
        if(info.numValidPairs > 0){
            BodyItem* bodyItem = p->first;
            sigKinematicStateChangedConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    bind(&WorldItemImpl::onBodyKinematicStateChanged, this, bodyItem)));
        }
    }

    if(isCollisionDetectionEnabled){
        updateCollisions(true);
    }
}


void WorldItemImpl::onBodyKinematicStateChanged(BodyItem* bodyItem)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::onBodyKinematicStateChanged()" << endl;
    }
    
    BodyItemInfoMap::iterator p = bodyItemInfoMap.find(bodyItem);
    if(p != bodyItemInfoMap.end()){
        BodyItemInfo& info = p->second;
        info.kinematicStateChanged = true;
        updateCollisionsCaller.setPriority(kinematicsBar->collisionDetectionPriority());
        updateCollisionsCaller.request();
    }
}


void WorldItem::updateCollisions()
{
    impl->updateCollisions(true);
}


void WorldItemImpl::updateCollisions(bool forceUpdate)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::updateCollisions()" << endl;
    }

    for(BodyItemInfoMap::iterator p = bodyItemInfoMap.begin(); p != bodyItemInfoMap.end(); ++p){
        BodyItem* bodyItem = p->first;
        BodyItemInfo& info = p->second;
        bodyItem->updateColdetModelPositions(forceUpdate);
        info.worldCollisionLinkSetChanged = bodyItem->updateSelfCollisions(forceUpdate);
        if(forceUpdate){
            info.kinematicStateChanged = true;
        }
    }

    for(size_t i=0; i < self->coldetPairs.size(); ++i){

        ColdetLinkPairEx* linkPair = static_cast<ColdetLinkPairEx*>(self->coldetPairs[i].get());
        BodyItemInfo* info1 = linkPair->info[0];
        BodyItemInfo* info2 = linkPair->info[1];
        
        if(info1->kinematicStateChanged || info2->kinematicStateChanged){
            
            bool prevEmpty = linkPair->collisions().empty();
            //linkPair->updatePositions();
            bool empty = linkPair->detectCollisions().empty();
            if(prevEmpty != empty){
                info1->worldCollisionLinkSetChanged = true;
                info2->worldCollisionLinkSetChanged = true;
            }
            if(!empty){
                info1->worldCollisionLinkBitSet.set(linkPair->link(0)->index);
                info2->worldCollisionLinkBitSet.set(linkPair->link(1)->index);
            }
        }
    }
    
    for(BodyItemInfoMap::iterator p = bodyItemInfoMap.begin(); p != bodyItemInfoMap.end(); ++p){
        
        BodyItem* bodyItem = p->first;
        BodyItemInfo& info = p->second;
        info.kinematicStateChanged = false;
        
        bodyItem->worldCollisionLinkBitSet = (info.worldCollisionLinkBitSet | bodyItem->selfCollisionLinkBitSet);
        
        if(info.worldCollisionLinkSetChanged){
            bodyItem->notifyWorldCollisionLinkSetChange();
        }
        bodyItem->notifyWorldCollisionUpdate();
        
        info.worldCollisionLinkSetChanged = false;
        info.worldCollisionLinkBitSet.reset();
    }
        
    sigCollisionsUpdated();
}


SignalProxy< boost::signal<void()> > WorldItem::sigColdetPairsUpdated()
{
    return impl->sigColdetPairsUpdated;
}


SignalProxy< boost::signal<void()> > WorldItem::sigCollisionsUpdated()
{
    return impl->sigCollisionsUpdated;
}


ItemPtr WorldItem::doDuplicate() const
{
    return new WorldItem(*this);
}


void WorldItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Collision detection"), isCollisionDetectionEnabled(),
                (bind(&WorldItem::enableCollisionDetection, this, _1), true));
}


bool WorldItem::store(Archive& archive)
{
    archive.write("collisionDetection", isCollisionDetectionEnabled());
    return true;
}


bool WorldItem::restore(const Archive& archive)
{
    if(archive.get("collisionDetection", false)){
        archive.addPostProcess(bind(&WorldItemImpl::enableCollisionDetection, impl, true));
    }
    return true;
}
