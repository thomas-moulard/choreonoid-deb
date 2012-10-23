/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_TREE_VIEW_H_INCLUDED
#define CNOID_GUIBASE_ITEM_TREE_VIEW_H_INCLUDED

#include "ItemList.h"
#include "SignalProxy.h"
#include <cnoid/View>
#include <QAbstractItemModel>
#include "exportdecl.h"

namespace cnoid {

    class RootItem;
    class ItemTreeViewImpl;

    /**
       @if jp
       アイテムツリーを表示するウィンドウ
       @endif
    */
    class CNOID_EXPORT ItemTreeView : public View
    {
        Q_OBJECT

     public:
        static void initialize(ExtensionManager* ext);
        static ItemTreeView* mainInstance();

        ItemTreeView(RootItem* rootItem, bool showRoot = false);
        ~ItemTreeView();

        RootItem* rootItem();

        void showRoot(bool show);

        /**
           @if jp
           選択状態になっているアイテムのうち、指定した型に適合するものを取得する。
           @endif
        */
        template <class ItemType> inline ItemList<ItemType> selectedItems() {
            ItemList<ItemType> items;
            getSelectedItems(items);
            return items;
        }

        template <class ItemType> inline ItemType* selectedItem(bool fromMultiItems = false) {
            return selectedItems<ItemType>().toSingle(fromMultiItems).get();
        }

        /**
           @if jp
           topItem 以下のサブツリーにおける選択状態アイテムのリストを得る。
           topItem は選択されていてもリストには含まれない。
           @endif
        */
        template <class ItemType> inline ItemList<ItemType> selectedSubItems(ItemPtr topItem) {
            ItemList<ItemType> items;
            extractSelectedItemsOfSubTree(topItem, items);
            return items;
        }

        template <class ItemType> inline ItemType* selectedSubItem(ItemPtr topItem, bool fromMultiItems = false) {
            return selectedSubItems<ItemType>(topItem).toSingle(fromMultiItems).get();
        }
        
        bool isItemSelected(ItemPtr item);
        bool selectItem(ItemPtr item, bool select = true);
        void clearSelection();

        /**
           @if jp
           チェック状態になっているアイテムのうち、指定した型に適合するものを取得する。
           @endif
        */
        template <class ItemType> inline ItemList<ItemType> checkedItems() {
            ItemList<ItemType> items;
            getCheckedItems(items);
            return items;
        }

        bool isItemChecked(ItemPtr item);
        bool checkItem(ItemPtr item, bool check);

        /**
           @if jp
           アイテムの選択状態が変化したときに発行されるシグナル。
           @endif
        */
        SignalProxy< boost::signal<void(const ItemList<Item>&)> > sigSelectionChanged();

        /**
           @if jp
           アイテムの選択状態が変化したか、ツリーの構造が変化したときに発行されるシグナル。
           アイテム間の親子関係もみるようなハンドラはこのシグナルと接続するとよい。
           @endif
        */
        SignalProxy< boost::signal<void(const ItemList<Item>&)> > sigSelectionOrTreeChanged();

        SignalProxy< boost::signal<void(Item* item, bool isChecked)> > sigCheckToggled();

        SignalProxy< boost::signal<void(bool isChecked)> > sigCheckToggled(Item* targetItem);

      protected:

        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);

      private:

        ItemTreeViewImpl* impl;

        void getSelectedItems(ItemListBase& items);
        void getCheckedItems(ItemListBase& items);
        void extractSelectedItemsOfSubTree(ItemPtr topItem, ItemListBase& items);

      private Q_SLOTS:
        void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
        void onRowsInserted(const QModelIndex& parent, int start, int end);
    };
}

#endif
