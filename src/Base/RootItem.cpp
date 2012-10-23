/**
  @author Shin'ichiro Nakaoka
*/

#include "RootItem.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "LazySignal.h"
#include <boost/bind.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

    class RootItemImpl
    {
    public:
        RootItem* self;
	
        RootItemImpl(RootItem* self);
        ~RootItemImpl();
        
        boost::signal<void(RootItem* rootItem)> sigDestroyed;
        boost::signal<void(Item* item)> sigItemAdded;
        boost::signal<void(Item* item, bool isMoving)> sigItemRemoving;
        boost::signal<void(Item* item, bool isMoving)> sigItemRemoved;
        LazySignal< boost::signal<void()> > sigTreeChanged;
    };
}


void RootItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<RootItem>(N_("RootItem"));
        ext->manage(RootItemPtr(mainInstance()));
        initialized = true;
    }
}


RootItem* RootItem::mainInstance()
{
    static RootItem* rootItem = new RootItem("Root");
    return rootItem;
}


RootItem::RootItem()
{
    initializeInstance();
}


RootItem::RootItem(const std::string& name)
    : Item(name)
{
    initializeInstance();
}


RootItem::RootItem(const RootItem& org)
    : Item(org)
{
    initializeInstance();
}


void RootItem::initializeInstance()
{
    impl = new RootItemImpl(this);

    unsetAttribute(NAME_EDITABLE);
    unsetAttribute(MOVABLE);
    unsetAttribute(CHECKABLE);
}


RootItemImpl::RootItemImpl(RootItem* self) :
    self(self)
{

}


RootItem::~RootItem()
{
    delete impl;
}


RootItemImpl::~RootItemImpl()
{
    if(TRACE_FUNCTIONS){
        cout << "RootItemImpl::~RootItemImpl()" << endl;
    }
    sigDestroyed(self);
}


SignalProxy< boost::signal<void(RootItem* rootItem)> > RootItem::sigDestroyed()
{
    return impl->sigDestroyed;
}


SignalProxy< boost::signal<void(Item* item)> > RootItem::sigItemAdded()
{
    return impl->sigItemAdded;
}


/**
   @if jp
   本シグナルを所有するルートアイテムからのパスに所属する子アイテムがパスから
   取り除かれる直前に発行されるシグナル。
   
   @param isMoving アイテムが移動中であって、再び本ルートアイテムからのパスに
   所属する場合、true となる。
   
   @todo できれば本シグナルは itemRemoved() シグナルで置き換えてdeprecatedとしたい。
*/
SignalProxy< boost::signal<void(Item* item, bool isMoving)> > RootItem::sigItemRemoving()
{
    return impl->sigItemRemoving;
}


/**
   @if jp
   本シグナルを所有するルートアイテムからのパスに所属する子アイテムがパスから
   取り除かれた後に呼ばれるスロットを接続する。
   
   @param isMoving アイテムが移動中であって、再び本ルートアイテムからのパスに
   所属する場合、true となる。
*/
SignalProxy< boost::signal<void(Item* item, bool isMoving)> > RootItem::sigItemRemoved()
{
    return impl->sigItemRemoved;
}


/**
   @if jp
   アイテムの追加・削除など、アイテムツリーの構造が変化したときに呼ばれるスロットを接続する。
   
   sigItemAdded や sigItemRemoving とは異なり、一度に行われる一連の操作に対して
   １回だけまとめて発行される。正確には、フレームワークのイベントループでキューにあるイベント
   が処理されてから実行される。
   
   @todo 「1回だけまとめて」は恐らくプロジェクト読み込み時などには守られていないので，
   この点改善しておく．
   @endif
*/
SignalProxy< boost::signal<void()> > RootItem::sigTreeChanged()
{
    return impl->sigTreeChanged.signal();
}


void RootItem::notifyEventOnItemAdded(Item* item)
{
    if(TRACE_FUNCTIONS){
        cout << "RootItem::notifyEventOnItemAdded()" << endl;
    }
    impl->sigItemAdded(item);
    impl->sigTreeChanged.request();
}


void RootItem::notifyEventOnItemRemoving(Item* item, bool isMoving)
{
    impl->sigItemRemoving(item, isMoving);
    impl->sigTreeChanged.request();
}


void RootItem::notifyEventOnItemRemoved(Item* item, bool isMoving)
{
    impl->sigItemRemoved(item, isMoving);
    impl->sigTreeChanged.request();
}


ItemPtr RootItem::doDuplicate() const
{
    return new RootItem(*this);
}


bool RootItem::store(Archive& archive)
{
    return true;
}


bool RootItem::restore(const Archive& archive)
{
    return true;
}

