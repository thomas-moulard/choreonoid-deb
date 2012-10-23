/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_WORLD_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_WORLD_ITEM_H_INCLUDED

#include "BodyItem.h"
#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/ColdetLinkPair>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

    class WorldItemImpl;

    /**
       WorldItem handles collisions between bodies in the world.
       On the other hand, self-collisions in a body are handled by BodyItems.
    */
    class CNOID_EXPORT WorldItem : public Item
    {
      public:

        WorldItem();
        WorldItem(const WorldItem& org);
        virtual ~WorldItem();

        inline ItemList<BodyItem> getBodyItems() const {
            return getSubItems<BodyItem>();
        }

        std::vector<ColdetLinkPairPtr> coldetPairs;

        void enableCollisionDetection(bool on);
        bool isCollisionDetectionEnabled();
        void updateCollisions();

        SignalProxy< boost::signal<void()> > sigColdetPairsUpdated();
        SignalProxy< boost::signal<void()> > sigCollisionsUpdated();
            
      protected:

        virtual ItemPtr doDuplicate() const;
        virtual void doPutProperties(PutPropertyFunction& putProperty);
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);

      private:

        WorldItemImpl* impl;
    };

    typedef boost::intrusive_ptr<WorldItem> WorldItemPtr;

    void initializeWorldItem(ExtensionManager& ext);

}

#endif
