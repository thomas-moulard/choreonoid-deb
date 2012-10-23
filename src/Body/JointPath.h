/**
   \file
   \brief The header file of the LinkPath and JointPath classes
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_JOINT_PATH_H_INCLUDED
#define CNOID_BODY_JOINT_PATH_H_INCLUDED

#include "LinkPath.h"
#include "InverseKinematics.h"
#include <cnoid/EigenTypes>
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT JointPath : public InverseKinematics
    {
      public:
		
        JointPath();
        JointPath(Link* base, Link* end);
        JointPath(Link* end);
        virtual ~JointPath();
		
        bool find(Link* base, Link* end);
        bool find(Link* end);

        inline bool empty() const {
            return joints.empty();
        }
		
        inline int numJoints() const {
            return joints.size();
        }
		
        inline Link* joint(int index) const {
            return joints[index];
        }

        inline Link* baseLink() const {
            return linkPath.baseLink();
        }

        inline Link* endLink() const {
            return linkPath.endLink();
        }
        
        inline bool isJointDownward(int index) const {
            return (index >= numUpwardJointConnections);
        }

        inline void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const {
            linkPath.calcForwardKinematics(calcVelocity, calcAcceleration);
        }
		
        void calcJacobian(Eigen::MatrixXd& out_J) const;
		
        void setMaxIKerror(double e);
        void setBestEffortIKMode(bool on);
        
        // InverseKinematics Interface
        virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R);
        virtual bool hasAnalyticalIK() const;

        bool calcInverseKinematics(
            const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R);
		
      protected:
		
        virtual void onJointPathUpdated();
		
        double maxIKerrorSqr;
        bool isBestEffortIKMode;
		
      private:
		
        void initialize();
        void extractJoints();

        LinkPath linkPath;
        std::vector<Link*> joints;
        int numUpwardJointConnections;
    };

    typedef boost::shared_ptr<JointPath> JointPathPtr;
	
};


#endif
