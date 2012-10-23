/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_MULTI_AFFINE_SEQ_GRAPH_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_MULTI_AFFINE_SEQ_GRAPH_VIEW_H_INCLUDED

#include "BodyItem.h"
#include "LinkSelectionView.h"
#include <set>
#include <list>
#include <vector>
#include <QBoxLayout>
#include <boost/signals.hpp>
#include <cnoid/Button>
#include <cnoid/MultiAffine3SeqItem>
#include <cnoid/Link>
#include <cnoid/View>
#include <cnoid/GraphWidget>
#include <cnoid/ItemList>
#include <cnoid/ConnectionSet>

namespace cnoid {

    class Archive;

    /**
       @todo Define and implement the API for installing an index selection interface
       and move this class into GuiBase module
    */
    class MultiAffine3SeqGraphView : public View, public boost::signals::trackable
    {
    public:
            
        MultiAffine3SeqGraphView();
        ~MultiAffine3SeqGraphView();
            
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);
            
    protected:
            
        virtual QWidget* indicatorOnInfoBar();
            
    private:
            
        GraphWidget graph;
        LinkSelectionView* linkSelection;

        ToggleToolButton xyzToggles[3];
        ToggleToolButton rpyToggles[3];

        ConnectionSet toggleConnections;

        struct ItemInfo
        {
            ~ItemInfo(){
                connections.disconnect();
            }
            MultiAffine3SeqItemPtr item;
            MultiAffine3SeqPtr seq;
            BodyItemPtr bodyItem;
            ConnectionSet connections;
            std::vector<GraphDataHandlerPtr> handlers;
        };

        std::list<ItemInfo> itemInfos;

        std::set<BodyItemPtr> bodyItems;
        ConnectionSet bodyItemConnections;

        void setupElementToggleSet(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive);
        void onItemSelectionChanged(const ItemList<MultiAffine3SeqItem>& dataItems);
        void onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter);
        void updateBodyItems();
        void onBodyItemDetachedFromRoot(BodyItemPtr bodyItem);
        void setupGraphWidget();
        void addPositionTrajectory(std::list<ItemInfo>::iterator itemInfoIter, Link* link, MultiAffine3SeqPtr seq);
        void onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter);

        void onDataRequest(
            std::list<ItemInfo>::iterator itemInfoIter,
            int linkIndex, int type, int axis, int frame, int size, double* out_values);
        void onDataModified(
            std::list<ItemInfo>::iterator itemInfoIter,
            int linkIndex, int type, int axis, int frame, int size, double* values);

    };
}

#endif
