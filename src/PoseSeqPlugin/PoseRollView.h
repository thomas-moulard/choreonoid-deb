/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CHOREOGRAPHY_POSE_ROLL_VIEW_H_INCLUDED
#define CNOID_CHOREOGRAPHY_POSE_ROLL_VIEW_H_INCLUDED

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

    class Archive;
    class PoseRollViewImpl;
        
    class CNOID_EXPORT PoseRollView : public cnoid::View
    {
      public:
        static PoseRollView* instance();

        PoseRollView();
        ~PoseRollView();

        /**
           This is a function for a temporary test.
           Please don't use this function.
        */
        void onInsertPoseButtonClicked(void);
        
      private:
        PoseRollViewImpl* impl;
        
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);
        virtual bool eventFilter(QObject *obj, QEvent *event);
        
    };
}

#endif
