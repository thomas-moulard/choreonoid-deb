/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_LIST_H_INCLUDED
#define CNOID_GUIBASE_ITEM_LIST_H_INCLUDED

#include <deque>
#include <boost/intrusive_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

    class Item;
    typedef boost::intrusive_ptr<Item> ItemPtr;

    template <class ItemType> class ItemList;

    /**
       @if jp
       アイテムコンテナにおいて、
       自身の対象とする型のアイテムのみを追加するためのインタフェースとなる抽象クラス。
       @endif
    */
    class CNOID_EXPORT ItemListBase
    {
    public:

        ItemListBase() { }

        /**
           @if jp
           代入演算子。
           
           コピー元リストが保持するアイテムのうち、
           コピー先リスとの型に適合するものだけがコピーされる。
           @endif
        */
        template <class LhsType> inline ItemListBase& operator=(const ItemList<LhsType>& org) {
            clear();
            for(size_t i=0; i < org.size(); ++i){ 
                this->appendIfTypeMatches(org[i]); 
            }
            return *this;
        }
    
        virtual ~ItemListBase() { }

        /**
           @if jp
           アイテムを追加する。

           @param item 追加するアイテム。
           このアイテムの型がItemListBaseが対象としている型と一致しているときのみ、
           実際に追加が行われる。
           @endif
        */
        virtual bool appendIfTypeMatches(ItemPtr item) = 0;

        virtual void clear() = 0;
    };


    /**
       @if jp
       対象とする型のアイテムのみを保持するアイテムリスト。
       @endif
    */
    template <class ItemType = Item>
    class ItemList : public ItemListBase
    {
        typedef boost::intrusive_ptr<ItemType> ItemTypePtr;
        typedef std::deque<ItemTypePtr> Container;

    public:

        typedef typename Container::iterator iterator;

        ItemList() { }
        
        /**
           @if jp
           コピーコンストラクタ。
         
           コピー元リストが保持するアイテムのうち、
           コピー先リスとの型に適合するものだけがコピーされる。
           @endif
        */
        template <class LhsType> ItemList(const ItemList<LhsType>& org){
            for(size_t i=0; i < org.size(); ++i){ 
                this->appendIfTypeMatches(org[i]); 
            }
        }

        ItemList(const ItemList& org) : items(org.items) { }

        /**
           @if jp
           代入演算子。

           コピー元リストが保持するアイテムのうち、
           コピー先リスとの型に適合するものだけがコピーされる。
           @endif
        */
        template <class LhsType> inline ItemList& operator=(const ItemList<LhsType>& org){
            static_cast<ItemListBase>(*this) = org;
            return *this;
        }

        inline ItemList& operator=(const ItemList& org){
            items = org.items;
            return *this;
        }

        virtual ~ItemList() { }

        inline bool operator==(const ItemList<ItemType>& lhs){
            return (items == lhs.items);
        }

        inline bool operator!=(const ItemList<ItemType>& lhs){
            return (items != lhs.items);
        }
        
        inline bool empty() const {
            return items.empty();
        }

        inline size_t size() const { 
            return items.size();
        }

        inline iterator begin() {
            return items.begin();
        }

        inline iterator end() {
            return items.end();
        }

        inline ItemType* back() const {
            return items.back().get();
        }

        inline ItemType* front() const {
            return items.front().get();
        }

        inline ItemType* operator[](size_t i) const {
            return items[i].get();
        }

        inline void clear(){
            items.clear();
        }

        inline void append(ItemTypePtr item){
            items.push_back(item);
        }

        inline void push_front(ItemTypePtr item){
            items.push_front(item);
        }
        
        inline void push_back(ItemTypePtr item){
            items.push_back(item);
        }

        inline void pop_front(){
            items.pop_front();
        }
        
        inline void pop_back(){
            items.pop_back();
        }

        virtual bool appendIfTypeMatches(ItemPtr item){
            ItemTypePtr castedItem = boost::dynamic_pointer_cast<ItemType>(item);
            if(castedItem){
                items.push_back(castedItem);
            }
            return castedItem;
        }

        inline boost::intrusive_ptr<ItemType> toSingle(bool allowFromMultiItems = false) const {
            return (items.size() == 1 || (allowFromMultiItems && !items.empty())) ?
                items[0] : ItemTypePtr();
        }

        template <class ItemType2>
        inline boost::intrusive_ptr<ItemType2> extractSingle(bool allowFromMultiItems = false) const {
            const ItemList<ItemType2> narrowdItems(*this);
            return (narrowdItems.size() == 1 || (allowFromMultiItems && !narrowdItems.empty())) ?
                narrowdItems[0] : boost::intrusive_ptr<ItemType2>();
        }

    protected:
        Container items;
    };

}


#endif
