/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_COMPOSITE_IK_H_INCLUDED
#define CNOID_BODY_COMPOSITE_IK_H_INCLUDED

#include "Body.h"
#include "InverseKinematics.h"
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

    class Link;

    class CNOID_EXPORT CompositeIK : public InverseKinematics
    {
      public:

        CompositeIK(BodyPtr body, Link* targetLink);
        ~CompositeIK();

        BodyPtr body() const { return body_; }
        bool addBaseLink(Link* link);
        void setMaxIKerror(double e);

        virtual bool hasAnalyticalIK() const;
        virtual bool calcInverseKinematics(const Vector3& p, const Matrix3& R);
        

      private:

        BodyPtr body_;
        Link* targetLink_;
        struct PathInfo {
            JointPathPtr path;
            Vector3 p0;
            Matrix3 R0;
        };
        std::vector<PathInfo> pathList;
        bool isAnalytical_;
    };

    typedef boost::shared_ptr<CompositeIK> CompositeIKptr;
}

#endif
