/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PENETRATION_BLOCKER_H_INCLUDED
#define CNOID_BODY_PENETRATION_BLOCKER_H_INCLUDED

#include "Link.h"
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

    class PenetrationBlockerImpl;

    class CNOID_EXPORT PenetrationBlocker
    {
      public:
        PenetrationBlocker(Link* targetLink);
        void addOpponentLink(Link* link);
        void setDepth(double depth);
        void start();
        bool adjust(Vector3& io_p, const Matrix3& R, const Vector3& pushDirection);
      private:
        PenetrationBlockerImpl* impl;
    };

    typedef boost::shared_ptr<PenetrationBlocker> PenetrationBlockerPtr;
}

#endif
