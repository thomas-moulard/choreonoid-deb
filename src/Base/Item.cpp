/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Item.h"
#include "RootItem.h"
#include "ItemPath.h"
#include "ItemManager.h"
#include <typeinfo>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

//tmp
#include <iostream>

namespace {
    const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {
    boost::signal<void(const char* type_info_name)> Item::sigClassUnregistered_;
}


Item::Item()
{
    init();
}


Item::Item(const std::string& name) :
    name_(name)
{
    init();
}


Item::Item(const Item& org) :
    name_(org.name_)
{
    init();
    attributes = org.attributes;
}


void Item::init()
{
    parent_ = 0;
    prevItem_ = 0;
    lastChild_ = 0;

    attributes.set(); // set all the bits

    isSubItem_ = false;

    isConsistentWithLastAccessedFile_ = false;
    timeStampOfLastFileWriting_ = 0;
}


Item::~Item()
{
    if(TRACE_FUNCTIONS){
        cout << "Item::~Item() of " << name_ << endl;
    }
}


/**
   @if jp
   アイテムの名前を設定／変更する。   

   名前が変わると、sigNameChanged シグナルが発行される。   
   @endif
*/
void Item::setName(const std::string& name)
{
    if(name != name_){
        name_ = name;
        sigNameChanged_();
    }
}


/*
Item* Item::child(int index) const
{
    Item* found = 0;
    int currentIndex = 0;
    for(Item* item = child(); item; item = item->sibling()){
        if(currentIndex == index){
            found = item;
            break;
        }
        ++currentIndex;
    }
    return found;
}
*/


/**
   Returns positions in the parent item.
*/
 /*
int Item::localIndex() const
{
    int index = -1;
    if(parent_){
        index = 0;
        Item* item = parent()->child();
        while(true){
            if(!item){
                throw "Error in Item::localIndex()";
            }
            if(item == this){
                break;
            }
            item = item->sibling();
            ++index;
        }
    }
    return index;
}
 */


  /*
int Item::numChildren() const
{
    int n = 0;
    for(Item* item = child(); item; item = item->sibling()){
        ++n;
    }
    return n;
}
  */


/**
   @if jp
   本アイテム以下のツリーに新たにアイテムを追加する。   
   
   @param item 追加するアイテム
   @endif
*/
void Item::addChildItem(ItemPtr item)
{
    doInsertChildItem(item.get(), 0);
}


/**
   @if jp
   アイテムがその不可欠な構成要素として小アイテムを持つ場合、   
   addChildItemではなく本関数を用いて小アイテムを追加しておくことで、   
   システムはその状況を把握可能となる。   
   この関数によって追加されたアイテムは isSubItem() が true となる。   
   @endif
*/
void Item::addSubItem(ItemPtr item)
{
    item->isSubItem_ = true;
    addChildItem(item);
}


void Item::insertChildItem(ItemPtr item, ItemPtr nextItem)
{
    doInsertChildItem(item.get(), nextItem.get());
}


void Item::doInsertChildItem(Item* item, Item* nextItem)
{
    bool isMoving = false;
    RootItem* rootItem = findRootItem();
    
    if(item->parent_){
        RootItem* srcRootItem = item->parent_->findRootItem();
        if(srcRootItem){
            if(srcRootItem == rootItem){
                isMoving = true;
            }
        }
        item->detachFromParentItemSub(isMoving);
    }
        
    item->parent_ = this;

    if(nextItem && (nextItem->parent_ == this)){
        Item* prevItem = nextItem->prevItem_;
        if(prevItem){
            prevItem->nextItem_ = item;
            item->prevItem_ = prevItem;
        } else {
            firstChild_ = item;
            item->prevItem_ = 0;
        }
        nextItem->prevItem_ = item;
        item->nextItem_ = nextItem;

    } else if(lastChild_){
        lastChild_->nextItem_ = item;
        item->prevItem_ = lastChild_;
        item->nextItem_ = 0;
        lastChild_ = item;
    } else {
        firstChild_ = item;
        lastChild_ = item;
    }

    // This must be before rootItem->notifyEventOnItemAdded().
    item->callSlotsOnPositionChanged();

    if(rootItem){
        rootItem->notifyEventOnItemAdded(item);
        if(!isMoving){
            item->callFuncOnConnectedToRoot();
        }
    }
}


void Item::callSlotsOnPositionChanged()
{
    onPositionChanged();
    sigPositionChanged_();
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
        child->callSlotsOnPositionChanged();
    }
}


void Item::callFuncOnConnectedToRoot()
{
    onConnectedToRoot();
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
        child->callFuncOnConnectedToRoot();
    }
}


bool Item::isSubItem() const
{
    return isSubItem_;
}


/**
   @if jp
   アイテムを親アイテムから切り離す。
   @return ルートアイテムのツリー内から切り離される場合は、そのルートアイテムを返す。
   @endif
*/
void Item::detachFromParentItem()
{
    ItemPtr self = this;
    detachFromParentItemSub(false);
}


void Item::detachFromParentItemSub(bool isMoving)
{
    RootItem* rootItem = findRootItem();
  
    if(rootItem){
        rootItem->notifyEventOnItemRemoving(this, isMoving);
    }

    if(parent_){
        if(prevItem_){
            prevItem_->nextItem_ = nextItem_;
        } else {
            parent_->firstChild_ = nextItem_;
        }
        if(nextItem_){
            nextItem_->prevItem_ = prevItem_;
        } else {
            parent_->lastChild_ = prevItem_;
        }
    
        prevItem_ = 0;
        nextItem_ = 0;
        parent_ = 0;
    }
    isSubItem_ = false;

    if(rootItem){
        rootItem->notifyEventOnItemRemoved(this, isMoving);
        if(!isMoving){
            emitSigDetachedFromRootForSubTree();
        }
    }
}


void Item::emitSigDetachedFromRootForSubTree()
{
    for(ItemPtr child = childItem(); child; child = child->nextItem()){
        child->emitSigDetachedFromRootForSubTree();
    }
    sigDetachedFromRoot_();
    onDisconnectedFromRoot();
}


void Item::onConnectedToRoot()
{

}


void Item::onDisconnectedFromRoot()
{

}


void Item::onPositionChanged()
{

}




namespace {

    Item* findItemSub(Item* current, ItemPath::iterator it, ItemPath::iterator end)
    {
        if(it == end){
            return current;
        }
        
        Item* item = 0;
        
        for(Item* child = current->childItem(); child; child = child->nextItem()){
            if(child->name() == *it){
                item = findItemSub(child, ++it, end);
                if(item){
                    break;
                }
            }
        }
        
        return item;
    }
}


Item* Item::findItem(const std::string& path) const
{
    ItemPath ipath(path);
    return findItemSub(const_cast<Item*>(this), ipath.begin(), ipath.end());
}


void Item::pickSubItems(Item* item, ItemListBase& itemList)
{
    if(item){
        itemList.appendIfTypeMatches(item);
        pickSubItems(item->childItem(), itemList);
        pickSubItems(item->nextItem(), itemList);
    }
}


RootItem* Item::findRootItem() const
{
    Item* current = const_cast<Item*>(this);
    
    while(current->parent_){
	current = current->parent_;
    }

    return dynamic_cast<RootItem*>(current);
}


/**
   @return When the item is embeded one,
   this function returs the first parent item which is not an embeded one.
   Otherwise the item itself is returned.
*/
Item* Item::headItem() const
{
    Item* head = const_cast<Item*>(this);
    while(head->isSubItem_){
        if(head->parent_){
            head = head->parent_;
        } else {
            break;
        }
    }
    return head;
}


void Item::traverse(boost::function<void(Item*)> function)
{
    traverse(this, function);
}


void Item::traverse(Item* item, const boost::function<void(Item*)>& function)
{
    function(item);
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        traverse(child, function);
    }
}

    
/**
   @todo added the 'notifyUpdateLater()' method ?
*/
void Item::notifyUpdate()
{
    sigUpdated_();
}


/**
   @if jp
   アイテムのコピーを生成する。   
   小アイテムについては isFixedToParentItem() が true のときはコピーされるが、   
   false のときはコピーされない。   
   @endif
*/
ItemPtr Item::duplicate() const
{
    ItemPtr duplicated = doDuplicate();
    if(duplicated && (typeid(*duplicated) != typeid(*this))){
        duplicated = 0;
    }
    return duplicated;
}


/**
   @if jp
   小アイテム（サブツリー）も含めたアイテムのコピーを生成する。   
   @endif
*/
ItemPtr Item::duplicateAll() const
{
    return duplicateAllSub(0);
}


ItemPtr Item::duplicateAllSub(ItemPtr duplicated) const
{
    if(!duplicated){
        duplicated = this->duplicate();
    }
    
    if(duplicated){
    	for(ItemPtr child = childItem(); child; child = child->nextItem()){
            ItemPtr duplicatedChildItem;
            if(child->isSubItem()){
                duplicatedChildItem = duplicated->findItem(child->name());
                if(duplicatedChildItem){
                    child->duplicateAllSub(duplicatedChildItem);
                }
            } else {
                duplicatedChildItem = child->duplicateAllSub(0);
                if(duplicatedChildItem){
                    duplicated->addChildItem(duplicatedChildItem);
                }
            }
    	}
    }

    return duplicated;
}


/**
   継承クラスの実装においてこの関数をオーバーライドする。   
*/
ItemPtr Item::doDuplicate() const
{
    return new Item(*this);
}


/**
   This function loads the data of the item from a file by using a pre-registered loading function.
   
   To make this function available, a loading function has to be registered to an ItemManager
   in advance by calling the addLoader() or addLoaderAndSaver() function.  Otherwise,
   this function cannot be used.
   Note that this function should not be overloaded or overridden in the derived classes.
*/
bool Item::load(const std::string& filename, const std::string& formatId)
{
    return ItemManager::load(this, filename, parentItem(), formatId);
}


/**
   @param parentItem specify this when the item is newly created one and will be attached to a parent item
   if loading succeeds.
*/
bool Item::load(const std::string& filename, Item* parent, const std::string& formatId)
{
    return ItemManager::load(this, filename, parent, formatId);
}


bool Item::reload()
{
    if(!lastAccessedFileName_.empty() && !lastAccessedFileFormatId_.empty()){
        return load(lastAccessedFileName_, lastAccessedFileFormatId_);
    }
    return false;
}


/**
   This function saves the data of the item to a file by using a pre-registered saving function.
   
   To make this function available, a saving function has to be registered to an ItemManager
   in advance by calling the addSaver() or addLoaderAndSaver() function.  Otherwise,
   this function cannot be used.
   Note that this function should not be overloaded or overridden in the derived classes.
*/
bool Item::save(const std::string& filename, const std::string& formatId)
{
    return ItemManager::save(this, filename, formatId);
}


/**
   This function save the data of the item to the file from which the data of the item has been loaded.
   
   If the data has not been loaded from a file, a file save dialog opens and user specifies a file.
*/
bool Item::overwrite(bool forceOverwrite, const std::string& formatId)
{
    return ItemManager::overwrite(this, forceOverwrite, formatId);
}


void Item::updateLastAccessInformation(const std::string& filename, const std::string& formatId)
{
    lastAccessedFileName_ = filename;
    lastAccessedFileFormatId_ = formatId;
    timeStampOfLastFileWriting_ = filesystem::last_write_time(filesystem::path(filename));
    isConsistentWithLastAccessedFile_ = true;
}


/**
   Use this function to disable the implicit overwrite next time
*/
void Item::clearLastAccessInformation()
{
    lastAccessedFileName_.clear();
    lastAccessedFileFormatId_.clear();
    isConsistentWithLastAccessedFile_ = false;
}


const Referenced* Item::customData(int id) const
{
    if(id >= (int)extraData.size()){
        return 0;
    }
    return extraData[id].get();
}


Referenced* Item::customData(int id)
{
    if(id >= (int)extraData.size()){
        return 0;
    }
    return extraData[id].get();
}


void Item::setCustomData(int id, ReferencedPtr data)
{
    if(id >= (int)extraData.size()){
        extraData.resize(id + 1, 0);
    }
    extraData[id] = data;
}


void Item::clearCustomData(int id)
{
    if(customData(id)){
        extraData[id] = 0;
    }
}


namespace {
    bool onNamePropertyChanged(Item* item, const string& name)
    {
        if(!name.empty()){
            item->setName(name);
        }
        return !name.empty();
    }
}


/**
   @if jp
   プロパティを表すキーと値のペアを出力する。   

   通常、プロパティビューによって呼ばれ、出力結果がプロパティビューにリスト表示される。   

   @param viewer このオブジェクトに対してプロパティを出力する。   

   @endif
*/
void Item::putProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Name"), name_, bind(onNamePropertyChanged, this, _1));

    std::string moduleName, className;
    ItemManager::getClassIdentifier(this, moduleName, className);
    putProperty(_("Class"), className);
    
    doPutProperties(putProperty);

    if(!lastAccessedFileName_.empty()){
        putProperty(_("File"), lastAccessedFileName_);
    }
    
    putProperty(_("Refs"), refCounter());
    putProperty(_("Sub item ?"), isSubItem_);
}


/**
   @if jp
   派生クラスにて本関数をオーバライドすることで、   
   プロパティリストの表示に対応させることができる。   

   プロパティの出力には Item::putProperty() 関数を使用する。   

   @note Item継承クラスが親クラス（基底クラス）として存在する場合は、通常、   
   本関数の中でまず親クラスの doPutProperties を呼び出し、   
   その後親クラスに対して追加されたプロパティを出力する。   

   @endif
*/
void Item::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool Item::store(Archive& archive)
{
    return false;
}


bool Item::restore(const Archive& archive)
{
    return false;
}
