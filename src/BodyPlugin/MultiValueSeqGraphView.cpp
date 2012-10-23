/**
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeqGraphView.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/Link>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}


MultiValueSeqGraphView::MultiValueSeqGraphView()
    : graph(this)
{
    setDefaultLayoutArea(View::BOTTOM);
    
    setName(N_("Multi Value Seq"));

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(&graph);
    setLayout(vbox);
    
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
        bind(&MultiValueSeqGraphView::onItemSelectionChanged, this, _1));

    linkSelection = LinkSelectionView::mainInstance();
}


MultiValueSeqGraphView::~MultiValueSeqGraphView()
{
    bodyItemConnections.disconnect();
}


QWidget* MultiValueSeqGraphView::indicatorOnInfoBar()
{
    return &graph.statusLabel();
}


void MultiValueSeqGraphView::onItemSelectionChanged(const ItemList<MultiValueSeqItem>& items)
{
    if(items.empty()){
        return;
    }

    if(itemInfos.size() == items.size()){
        bool unchanged = true;
        int i=0;
        for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){
            if(it->item != items[i++]){
                unchanged = false;
                break;
            }
        }
        if(unchanged){
            return;
        }
    }
            
    itemInfos.clear();

    for(size_t i=0; i < items.size(); ++i){
        BodyItemPtr bodyItem = items[i]->findOwnerItem<BodyItem>();
        if(bodyItem){
            itemInfos.push_back(ItemInfo());
            list<ItemInfo>::iterator it = --itemInfos.end();
            it->item = items[i];
            it->seq = it->item->seq();
            it->bodyItem = bodyItem;

            it->connections.add(it->item->sigUpdated().connect(
                                     bind(&MultiValueSeqGraphView::onDataItemUpdated, this, it)));

            it->connections.add(it->item->sigDetachedFromRoot().connect(
                                     bind(&MultiValueSeqGraphView::onDataItemDetachedFromRoot, this, it)));
        }
    }

    updateBodyItems();
    setupGraphWidget();
}


void MultiValueSeqGraphView::onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter)
{
    itemInfos.erase(itemInfoIter);
    updateBodyItems();
    setupGraphWidget();
}


void MultiValueSeqGraphView::updateBodyItems()
{
    bodyItemConnections.disconnect();
    bodyItems.clear();
    
    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        set<BodyItemPtr>::iterator p = bodyItems.find(it->bodyItem);

        if(p == bodyItems.end()){

            bodyItems.insert(it->bodyItem);

            bodyItemConnections.add(
                linkSelection->sigSelectionChanged(it->bodyItem).connect(
                    bind(&MultiValueSeqGraphView::setupGraphWidget, this)));
            
            bodyItemConnections.add(
                it->bodyItem->sigDetachedFromRoot().connect(
                    bind(&MultiValueSeqGraphView::onBodyItemDetachedFromRoot, this, it->bodyItem)));
        }
    }
}


void MultiValueSeqGraphView::onBodyItemDetachedFromRoot(BodyItemPtr bodyItem)
{
    bool erased = false;
    list<ItemInfo>::iterator it = itemInfos.begin();
    while(it != itemInfos.end()){
       if(it->bodyItem == bodyItem){
           it = itemInfos.erase(it);
           erased = true;
       } else {
           ++it;
       }
    }
    if(erased){
        updateBodyItems();
        setupGraphWidget();
    }
}


void MultiValueSeqGraphView::setupGraphWidget()
{
    graph.clearDataHandlers();

    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        if(it->bodyItem){

            MultiValueSeqPtr seq = it->item->seq();
            int numParts = seq->numParts();
            BodyPtr body = it->bodyItem->body();
            const std::vector<int>& selectedLinkIndices = linkSelection->getSelectedLinkIndices(it->bodyItem);
            
            for(size_t i=0; i < selectedLinkIndices.size(); ++i){
                Link* link = body->link(selectedLinkIndices[i]);
                if(link && link->jointId >= 0 && link->jointId < numParts){
                    addJointTrajectory(it, link, seq);
                }
            }
        }
    }
}


void MultiValueSeqGraphView::addJointTrajectory(std::list<ItemInfo>::iterator itemInfoIter, Link* joint, MultiValueSeqPtr seq)
{
    GraphDataHandlerPtr handler(new GraphDataHandler());

    handler->setLabel(joint->name());
    handler->setValueLimits(joint->llimit, joint->ulimit);
    handler->setVelocityLimits(joint->lvlimit, joint->uvlimit);
                
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->setDataRequestCallback(
        bind(&MultiValueSeqGraphView::onDataRequest, this, itemInfoIter, joint->jointId, _1, _2, _3));
    handler->setDataModifiedCallback(
        bind(&MultiValueSeqGraphView::onDataModified, this, itemInfoIter, joint->jointId, _1, _2, _3));
                
    graph.addDataHandler(handler);
    itemInfoIter->handlers.push_back(handler);
}


void MultiValueSeqGraphView::onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter)
{
    if(TRACE_FUNCTIONS){
        cout << "MultiValueSeqGraphView::onDataItemUpdated()" << endl;
    }

    const MultiValueSeqPtr& seq = itemInfoIter->item->seq();
    int newNumFrames = seq->numFrames();
    double newFrameRate = seq->frameRate();
    
    for(size_t i=0; i < itemInfoIter->handlers.size(); ++i){
        if(TRACE_FUNCTIONS){
            cout << "MultiValueSeqGraphView::onDataItemUpdated(" << i << ")" << endl;
        }
        itemInfoIter->handlers[i]->setFrameProperties(newNumFrames, newFrameRate);
        itemInfoIter->handlers[i]->update();
    }
}


void MultiValueSeqGraphView::onDataRequest
(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* out_values)
{
    MultiValueSeq::View part = itemInfoIter->seq->part(jointId);
    for(int i=0; i < size; ++i){
        out_values[i] = part[frame + i];
    }
}


void MultiValueSeqGraphView::onDataModified
(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* values)
{
    MultiValueSeq::View part = itemInfoIter->seq->part(jointId);
    for(int i=0; i < size; ++i){
        part[frame + i] = values[i];
    }
    
    itemInfoIter->connections.block();
    itemInfoIter->item->notifyUpdate();
    itemInfoIter->connections.unblock();
}


bool MultiValueSeqGraphView::storeState(Archive& archive)
{
    return graph.storeState(archive);
}


bool MultiValueSeqGraphView::restoreState(const Archive& archive)
{
    return graph.restoreState(archive);
}

