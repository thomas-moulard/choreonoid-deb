/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_H_INCLUDED
#define CNOID_GUIBASE_ITEM_H_INCLUDED

#include "SignalProxy.h"
#include "ItemList.h"
#include "PutPropertyFunction.h"
#include <ctime>
#include <bitset>
#include <string>
#include <cnoid/Referenced>

#include "exportdecl.h"

namespace cnoid {

    class Item;
    typedef boost::intrusive_ptr<Item> ItemPtr;
    
    class RootItem;
    class ItemTreeArchiver;
    class ExtensionManager;
    class PutPropertyFunction;
    class Archive;

    /**
       @if not jp
       @endif

       @if jp
       フレームワーク上で共有されるオブジェクトを表すクラス。   
       モデル・ビュー・コントローラフレームワークにおけるモデル部分の核となる。   
       @endif
    */
    class CNOID_EXPORT Item : public Referenced
    {
      public:

        enum Attribute {
            VISIBLE = 0,
            MOVABLE,
            NAME_EDITABLE,
            CHECKABLE,
            TREE_EXPANDED_BY_DEFAULT,
            NUM_ATTRIBUTES
        };

        Item();
        Item(const std::string& name);
        Item(const Item& item);
	
        inline const std::string& name() const { return name_; }
        virtual void setName(const std::string& name);

        inline bool hasAttribute(Attribute attribute) const { return attributes[attribute]; }

        inline Item* childItem() const { return firstChild_.get(); }
        inline Item* prevItem() const { return prevItem_; }
        inline Item* nextItem() const { return nextItem_.get(); }
        inline Item* parentItem() const { return parent_; }

        void addChildItem(ItemPtr item);
        void addSubItem(ItemPtr item);
        bool isSubItem() const;
        void detachFromParentItem();
        void emitSigDetachedFromRootForSubTree();
        void insertChildItem(ItemPtr item, ItemPtr nextItem);

        RootItem* findRootItem() const;
        Item* findItem(const std::string& path) const;

        template<class ItemType>
            inline ItemType* findItem(const std::string& path) const {
            return dynamic_cast<ItemType*>(findItem(path));
        }

        template <class ItemType> inline ItemList<ItemType> getSubItems() const {
            ItemList<ItemType> itemList;
            pickSubItems(childItem(), itemList);
            return itemList;
        }

        Item* headItem() const;

        template <class ItemType> ItemType* findOwnerItem(bool includeSelf = false) {
            Item* parentItem__ = includeSelf ? this : parentItem();
            while(parentItem__){
                ItemType* ownerItem = dynamic_cast<ItemType*>(parentItem__);
                if(ownerItem){
                    return ownerItem;
                }
                parentItem__ = parentItem__->parentItem();
            }
            return 0;
        }

        void traverse(boost::function<void(Item*)> function);

        ItemPtr duplicate() const;
        ItemPtr duplicateAll() const;

        bool load(const std::string& filename, const std::string& formatId = std::string());
        bool load(const std::string& filename, Item* parent, const std::string& formatId = std::string());
        bool reload();
        bool save(const std::string& filename, const std::string& formatId = std::string());
        bool overwrite(bool forceOverwrite = false, const std::string& formatId = std::string());
        void clearLastAccessInformation();

        const std::string& lastAccessedFileName() const { return lastAccessedFileName_; }
        const std::string& lastAccessedFileFormatId() const { return lastAccessedFileFormatId_; }
        std::time_t timeStampOfLastFileWriting() const { return timeStampOfLastFileWriting_; }
        void setInconsistencyWithLastAccessedFile() { isConsistentWithLastAccessedFile_ = false; }
        bool isConsistentWithLastAccessedFile() const { return isConsistentWithLastAccessedFile_; }

        void putProperties(PutPropertyFunction& putProperty);

        virtual void notifyUpdate();

        inline SignalProxy< boost::signal<void()> > sigNameChanged() {
            return sigNameChanged_;
        }

        inline SignalProxy< boost::signal<void()> > sigUpdated() {
            return sigUpdated_;
        }

        /**
           @if jp
           アイテムツリーにおける本アイテムの位置が変わったとき（新規にツリーに追加されたり、
           ツリー内もしくはツリー間で移動されたとき）に発行されるシグナル．
           位置が変化したか否かはルートからの絶対位置によって判定するため、
           親（祖先）アイテムの位置が変われば子（子孫）アイテムに対しても本関数が呼ばれる。
           本シグナルは RootItem::sigTreeChanged() より前に呼ばれる．
           @endif
        */
        inline SignalProxy< boost::signal<void()> > sigPositionChanged() {
            return sigPositionChanged_;
        }

        /**
           @note obsolete.
        */
        inline SignalProxy< boost::signal<void()> > sigDetachedFromRoot() {
            return sigDetachedFromRoot_;
        }
        /**
           @note Please use this instead of sigDetachedFromRoot()
        */
        inline SignalProxy< boost::signal<void()> > sigDisconnectedFromRoot() {
            return sigDetachedFromRoot_;
        }

        inline static SignalProxy< boost::signal<void(const char* type_info_name)> > sigClassUnregistered() {
            return sigClassUnregistered_;
        }

        Referenced* customData(int id);
        const Referenced* customData(int id) const;
        void setCustomData(int id, ReferencedPtr data);
        void clearCustomData(int id);

      protected:

        // The destructor should not be called in usual ways
        virtual ~Item();

        virtual void onConnectedToRoot();
        virtual void onDisconnectedFromRoot();
        virtual void onPositionChanged();
        
        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
        virtual void doPutProperties(PutPropertyFunction& putProperty);

        static void pickSubItems(Item* item, ItemListBase& itemList);

        // void setAttribute(Attribute attribute) { attributes.set(attribute); }
        inline void unsetAttribute(Attribute attribute) { attributes.reset(attribute); }

      private:

        Item*   parent_;
        ItemPtr firstChild_;
        Item*   lastChild_;
        Item*   prevItem_;
        ItemPtr nextItem_;

        std::string name_;

        std::bitset<NUM_ATTRIBUTES> attributes;

        std::vector<int> extraStates;
        std::vector<ReferencedPtr> extraData;

        bool isSubItem_;

        boost::signal<void()> sigNameChanged_;
        boost::signal<void()> sigDetachedFromRoot_;
        boost::signal<void()> sigUpdated_;
        boost::signal<void()> sigPositionChanged_;

        static boost::signal<void(const char* type_info_name)> sigClassUnregistered_;

        // for file overwriting management, mainly accessed by ItemManagerImpl
        bool isConsistentWithLastAccessedFile_;
        std::string lastAccessedFileName_;
        std::string lastAccessedFileFormatId_;
        std::time_t timeStampOfLastFileWriting_;

        void init();
        void doInsertChildItem(Item* item, Item* nextItem);
        void callSlotsOnPositionChanged();
        void callFuncOnConnectedToRoot();
        void detachFromParentItemSub(bool isMoving);
        void traverse(Item* item, const boost::function<void(Item*)>& function);
        ItemPtr duplicateAllSub(ItemPtr duplicated) const;
        
        void updateLastAccessInformation(const std::string& filename, const std::string& formatId);
        
        friend class RootItem;
        friend class ItemTreeArchiver;
        friend class ItemManagerImpl;
    };

}

#endif
