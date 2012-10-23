/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_MULTI_VALUE_SEQ_GRAPH_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_MULTI_VALUE_SEQ_GRAPH_VIEW_H_INCLUDED

#include "BodyItem.h"
#include "LinkSelectionView.h"
#include <set>
#include <list>
#include <vector>
#include <boost/signals.hpp>
#include <cnoid/Link>
#include <cnoid/MultiValueSeqItem>
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
    class MultiValueSeqGraphView : public View, public boost::signals::trackable
    {
      public:
            
        MultiValueSeqGraphView();
        ~MultiValueSeqGraphView();
            
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);
            
      protected:

        virtual QWidget* indicatorOnInfoBar();
            
      private:
            
        GraphWidget graph;
        LinkSelectionView* linkSelection;

        struct ItemInfo
        {
            ~ItemInfo(){
                connections.disconnect();
            }
            MultiValueSeqItemPtr item;
            MultiValueSeqPtr seq;
            BodyItemPtr bodyItem;
            ConnectionSet connections;
            std::vector<GraphDataHandlerPtr> handlers;
        };

        std::list<ItemInfo> itemInfos;

        std::set<BodyItemPtr> bodyItems;
        ConnectionSet bodyItemConnections;

        void onItemSelectionChanged(const ItemList<MultiValueSeqItem>& dataItems);
        void onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter);
        void updateBodyItems();
        void onBodyItemDetachedFromRoot(BodyItemPtr bodyItem);
        void setupGraphWidget();
        void addJointTrajectory(std::list<ItemInfo>::iterator itemInfoIter, Link* joint, MultiValueSeqPtr seq);
        void onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter);
        void onDataRequest(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* out_values);
        void onDataModified(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* values);

    };
}

#endif
