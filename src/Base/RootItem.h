/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ROOT_ITEM_H_INCLUDED
#define CNOID_GUIBASE_ROOT_ITEM_H_INCLUDED

#include "Item.h"
#include "ItemList.h"
#include "exportdecl.h"

namespace cnoid {

    class Project;
    class Item;
    class RootItemImpl;

    /**
       @if jp
       アイテムの木構造のルートとなるクラス。
       @endif
    */
    class CNOID_EXPORT RootItem : public Item
    {
      public:
        static void initialize(ExtensionManager* ext);
        static RootItem* mainInstance();

        RootItem();
        RootItem(const std::string& name);
        RootItem(const RootItem& org);
        virtual ~RootItem();

        SignalProxy< boost::signal<void(RootItem* rootItem)> > sigDestroyed();
        SignalProxy< boost::signal<void(Item* item)> > sigItemAdded();
        SignalProxy< boost::signal<void(Item* item, bool isMoving)> > sigItemRemoving();
        SignalProxy< boost::signal<void(Item* item, bool isMoving)> > sigItemRemoved();
        SignalProxy< boost::signal<void()> > sigTreeChanged();

      protected:

        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);

      private:

        void initializeInstance();

        friend class Item;

        void notifyEventOnItemAdded(Item* item);
        void notifyEventOnItemRemoving(Item* item, bool isMoving);
        void notifyEventOnItemRemoved(Item* item, bool isMoving);

        friend class RootItemImpl;
        RootItemImpl* impl;
    };

    typedef boost::intrusive_ptr<RootItem> RootItemPtr;

}

#endif
