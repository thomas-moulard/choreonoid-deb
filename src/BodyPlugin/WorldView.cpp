/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "WorldView.h"
#include "WorldItem.h"
#include <boost/bind.hpp>
#include <QLabel>
#include <QBoxLayout>
#include <cnoid/Button>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
    const char* emptyNameString = "--------";
}


namespace cnoid {

    class WorldViewImpl : public boost::signals::trackable
    {
    public:
        WorldViewImpl(WorldView* self);
        virtual ~WorldViewImpl();

        WorldView* self;
        ostream& os;
        WorldItemPtr currentWorldItem;
        QLabel currentWorldLabel;
        CheckBox collisionDetectionCheck;
        signals::connection connectionOfCollisionDetectionToggled;
        signals::connection connectionOfCurrentWorldItemDetachedFromRoot;
        signals::connection connectionOfCurrentWordItemUpdated;
            
        void onItemTreeChanged();
        void onItemSelectionChanged(const ItemList<WorldItem>& worldItems);
        void setCurrentWorldItem(WorldItemPtr worldItem);
        void update();
        void onWorldItemDetachedFromRoot(WorldItemPtr worldItem);
        void onCollisionDetectionToggled(bool checked);
    };
}


WorldView::WorldView()
{
    impl = new WorldViewImpl(this);
}


WorldViewImpl::WorldViewImpl(WorldView* self)
  : self(self),
    os(MessageView::mainInstance()->cout())
{
    self->setName(N_("World"));
    self->setDefaultLayoutArea(View::CENTER);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(QString(_("World")) + ":"));
    currentWorldLabel.setText(emptyNameString);
    currentWorldLabel.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    hbox->addWidget(&currentWorldLabel);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    collisionDetectionCheck.setText(_("Collision detection"));
    collisionDetectionCheck.setChecked(false);

    connectionOfCollisionDetectionToggled = 
        collisionDetectionCheck.sigToggled().connect(
            bind(&WorldViewImpl::onCollisionDetectionToggled, this, _1));
    
    hbox->addWidget(&collisionDetectionCheck);
    vbox->addLayout(hbox);

    vbox->addStretch();
    self->setLayout(vbox);

    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
        bind(&WorldViewImpl::onItemSelectionChanged, this, _1));
}


WorldView::~WorldView()
{
  delete impl;
}


WorldViewImpl::~WorldViewImpl()
{

}


void WorldViewImpl::onItemTreeChanged()
{

}


void WorldViewImpl::onItemSelectionChanged(const ItemList<WorldItem>& worldItems)
{
    if(TRACE_FUNCTIONS){
        os << "WorldViewImpl::onItemSelectionChanged()" << endl;
    }
    
    WorldItemPtr worldItem = worldItems.toSingle();

    if(worldItem && worldItem != currentWorldItem){
        setCurrentWorldItem(worldItem);
    }
}
    

void WorldViewImpl::setCurrentWorldItem(WorldItemPtr worldItem)
{
    if(TRACE_FUNCTIONS){
        os << "WorldViewImpl::setCurrentWorldItem()" << endl;
    }
    
    connectionOfCurrentWorldItemDetachedFromRoot.disconnect();
    connectionOfCurrentWordItemUpdated.disconnect();

    currentWorldItem = worldItem;

    if(worldItem){
        connectionOfCurrentWorldItemDetachedFromRoot =
            worldItem->sigDetachedFromRoot().connect(
                bind(&WorldViewImpl::onWorldItemDetachedFromRoot, this, worldItem));

        connectionOfCurrentWordItemUpdated =
            worldItem->sigUpdated().connect(bind(&WorldViewImpl::update, this));
    }

    update();
}


void WorldViewImpl::update()
{
    connectionOfCollisionDetectionToggled.block();

    if(currentWorldItem){
        currentWorldLabel.setText(currentWorldItem->name().c_str());
        collisionDetectionCheck.setChecked(currentWorldItem->isCollisionDetectionEnabled());
    } else {
        currentWorldLabel.setText("");
        collisionDetectionCheck.setChecked(false);
    }

    connectionOfCollisionDetectionToggled.unblock();
}


void WorldViewImpl::onWorldItemDetachedFromRoot(WorldItemPtr worldItem)
{
    if(TRACE_FUNCTIONS){
        os << "WorldViewImpl::onWorldItemDetachedFromRoot()" << endl;
    }
    
    if(currentWorldItem == worldItem){
        setCurrentWorldItem(0);
    }
}


void WorldViewImpl::onCollisionDetectionToggled(bool checked)
{
    if(TRACE_FUNCTIONS){
        os << "WorldViewImpl::onCollisionDetectionToggled()" << endl;
    }
    
    if(currentWorldItem){
        currentWorldItem->enableCollisionDetection(checked);
    }
}
