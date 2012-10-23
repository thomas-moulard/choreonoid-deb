/**
   @author Shin'ichiro Nakaoka
*/

#include "MultiAffine3SeqGraphView.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/Link>
#include <cnoid/EigenUtil>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}


MultiAffine3SeqGraphView::MultiAffine3SeqGraphView()
    : graph(this)
{
    setName(N_("Multi Affine3 Seq"));
    setDefaultLayoutArea(View::BOTTOM);

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    static const char* rpyLabels[] = { "R", "P", "Y" };

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(0);
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    vbox->addStretch();
    setupElementToggleSet(vbox, xyzToggles, xyzLabels, true);
    setupElementToggleSet(vbox, rpyToggles, rpyLabels, false);
    vbox->addStretch();
    
    hbox->addLayout(vbox);
    hbox->addWidget(&graph, 1);
    
    setLayout(hbox);

    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
        bind(&MultiAffine3SeqGraphView::onItemSelectionChanged, this, _1));

    linkSelection = LinkSelectionView::mainInstance();
}


void MultiAffine3SeqGraphView::setupElementToggleSet
(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive)
{
    for(int i=0; i < 3; ++i){
        toggles[i].setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        box->addWidget(&toggles[i]);
        toggles[i].setChecked(isActive);
        toggles[i].setText(labels[i]);

        toggleConnections.add(
            toggles[i].sigToggled().connect(
                bind(&MultiAffine3SeqGraphView::setupGraphWidget, this)));
    }
}


MultiAffine3SeqGraphView::~MultiAffine3SeqGraphView()
{
    bodyItemConnections.disconnect();
}


QWidget* MultiAffine3SeqGraphView::indicatorOnInfoBar()
{
    return &graph.statusLabel();
}


void MultiAffine3SeqGraphView::onItemSelectionChanged(const ItemList<MultiAffine3SeqItem>& items)
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
                                    bind(&MultiAffine3SeqGraphView::onDataItemUpdated, this, it)));

            it->connections.add(it->item->sigDetachedFromRoot().connect(
                                    bind(&MultiAffine3SeqGraphView::onDataItemDetachedFromRoot, this, it)));
        }
    }

    updateBodyItems();
    setupGraphWidget();
}


void MultiAffine3SeqGraphView::onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter)
{
    itemInfos.erase(itemInfoIter);
    updateBodyItems();
    setupGraphWidget();
}


void MultiAffine3SeqGraphView::updateBodyItems()
{
    bodyItemConnections.disconnect();
    bodyItems.clear();
    
    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        set<BodyItemPtr>::iterator p = bodyItems.find(it->bodyItem);

        if(p == bodyItems.end()){

            bodyItems.insert(it->bodyItem);

            bodyItemConnections.add(
                linkSelection->sigSelectionChanged(it->bodyItem).connect(
                    bind(&MultiAffine3SeqGraphView::setupGraphWidget, this)));
            
            bodyItemConnections.add(
                it->bodyItem->sigDetachedFromRoot().connect(
                    bind(&MultiAffine3SeqGraphView::onBodyItemDetachedFromRoot, this, it->bodyItem)));
        }
    }
}


void MultiAffine3SeqGraphView::onBodyItemDetachedFromRoot(BodyItemPtr bodyItem)
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


void MultiAffine3SeqGraphView::setupGraphWidget()
{
    graph.clearDataHandlers();

    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        if(it->bodyItem){

            MultiAffine3SeqPtr seq = it->item->seq();
            int numParts = seq->numParts();
            BodyPtr body = it->bodyItem->body();
            const std::vector<int>& selectedLinkIndices = linkSelection->getSelectedLinkIndices(it->bodyItem);
            
            for(size_t i=0; i < selectedLinkIndices.size(); ++i){
                Link* link = body->link(selectedLinkIndices[i]);
                if(link && link->index < numParts){
                    addPositionTrajectory(it, link, seq);
                }
            }
        }
    }
}


void MultiAffine3SeqGraphView::addPositionTrajectory
(std::list<ItemInfo>::iterator itemInfoIter, Link* link, MultiAffine3SeqPtr seq)
{
    
    for(int i=0; i < 2; ++i){
        ToggleToolButton* toggles = (i == 0) ? xyzToggles : rpyToggles;
        for(int j=0; j < 3; ++j){

            if(toggles[j].isChecked()){
    
                GraphDataHandlerPtr handler(new GraphDataHandler());

                handler->setLabel(link->name());
            
                handler->setFrameProperties(seq->numFrames(), seq->frameRate());
                handler->setDataRequestCallback(
                    bind(&MultiAffine3SeqGraphView::onDataRequest, this, itemInfoIter, link->index, i, j, _1, _2, _3));
                handler->setDataModifiedCallback(
                    bind(&MultiAffine3SeqGraphView::onDataModified, this, itemInfoIter, link->index, i, j, _1, _2, _3));
            
                graph.addDataHandler(handler);
                itemInfoIter->handlers.push_back(handler);
            }
        }
    }
}


void MultiAffine3SeqGraphView::onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter)
{
    if(TRACE_FUNCTIONS){
        cout << "MultiAffine3SeqGraphView::onDataItemUpdated()" << endl;
    }

    const MultiAffine3SeqPtr& seq = itemInfoIter->item->seq();
    int newNumFrames = seq->numFrames();
    double newFrameRate = seq->frameRate();
    
    for(size_t i=0; i < itemInfoIter->handlers.size(); ++i){
        if(TRACE_FUNCTIONS){
            cout << "MultiAffine3SeqGraphView::onDataItemUpdated(" << i << ")" << endl;
        }
        itemInfoIter->handlers[i]->setFrameProperties(newNumFrames, newFrameRate);
        itemInfoIter->handlers[i]->update();
    }
}


void MultiAffine3SeqGraphView::onDataRequest
(std::list<ItemInfo>::iterator itemInfoIter, int linkIndex, int type, int axis, int frame, int size, double* out_values)
{
    MultiAffine3Seq::View part = itemInfoIter->seq->part(linkIndex);
    if(type == 0){ // xyz
        for(int i=0; i < size; ++i){
            const Affine3& p = part[frame + i];
            out_values[i] = p.translation()[axis];
        }
    } else { // rpy
        for(int i=0; i < size; ++i){
            const Affine3& p = part[frame + i];
            out_values[i] = rpyFromRot(p.linear())[axis];
        }
    }
}


void MultiAffine3SeqGraphView::onDataModified
(std::list<ItemInfo>::iterator itemInfoIter, int linkIndex, int type, int axis, int frame, int size, double* values)
{
    MultiAffine3Seq::View part = itemInfoIter->seq->part(linkIndex);
    if(type == 0){ // xyz
        for(int i=0; i < size; ++i){
            Affine3& p = part[frame + i];
            p.translation()[axis] = values[i];
        }
    } else { // rpy
        for(int i=0; i < size; ++i){
            Affine3& p = part[frame + i];
            Vector3 rpy(rpyFromRot(p.linear()));
            rpy[axis] = values[i];
            p.linear() = rotFromRpy(rpy);
        }
    }        
    
    itemInfoIter->connections.block();
    itemInfoIter->item->notifyUpdate();
    itemInfoIter->connections.unblock();
}


bool MultiAffine3SeqGraphView::storeState(Archive& archive)
{
    if(graph.storeState(archive)){
        YamlSequence& visibleElements = *archive.createFlowStyleSequence("visibleElements");
        for(int i=0; i < 3; ++i){
            if(xyzToggles[i].isChecked()){
                visibleElements.append(i);
            }
        }
        for(int i=0; i < 3; ++i){
            if(rpyToggles[i].isChecked()){
                visibleElements.append(i+3);
            }
        }
        return true;
    }
    return false;
}


bool MultiAffine3SeqGraphView::restoreState(const Archive& archive)
{
    if(graph.restoreState(archive)){
        const YamlSequence& visibleElements = *archive.findSequence("visibleElements");
        if(visibleElements.isValid()){
            toggleConnections.block();
            for(int i=0; i < 3; ++i){
                xyzToggles[i].setChecked(false);
                rpyToggles[i].setChecked(false);
            }
            for(int i=0; i < visibleElements.size(); ++i){
                int index = visibleElements[i].toInt();
                if(index < 3){
                    xyzToggles[index].setChecked(true);
                } else {
                    rpyToggles[index-3].setChecked(true);
                }
            }
            toggleConnections.unblock();
        }
        return true;
    }
    return false;
}
