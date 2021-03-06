/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_H_INCLUDED
#define CNOID_BODY_LINK_H_INCLUDED

#include <string>
#include <ostream>
#include <vector>
#include <cnoid/EigenTypes>
#include <cnoid/ColdetModel>
#include "exportdecl.h"

namespace cnoid {
    class Link;
}

CNOID_EXPORT std::ostream& operator<<(std::ostream &out, cnoid::Link& link);

namespace cnoid {

    class Body;

    class CNOID_EXPORT Link {

      public:

        Link();
        Link(const Link& link);
        ~Link();

        inline const std::string& name() {
            return name_;
        }
        inline void setName(const std::string& name){
            name_ = name;
        }

        inline bool isValid() { return (index >= 0); }
        void addChild(Link* link);
        bool detachChild(Link* link);
        inline bool isRoot() { return !parent; }

        inline void setAttitude(const Matrix3& R) { this->R = R * Rs.transpose(); }
        inline Matrix3 attitude() { return this->R * Rs; }
        inline Matrix3 calcRfromAttitude(const Matrix3& R) { return R * Rs.transpose(); }

        /**
           @brief compute sum of m x wc of subtree
           @note assuming wc is already computed by Body::calcCM()
        */
        void calcSubMassCM();

        /**
           @deprecated use setAttitude().
        */
        inline void setSegmentAttitude(const Matrix3& R) { this->R = R * Rs.transpose(); }

        /**
           @deprecated use attitude().
        */
        inline Matrix3 segmentAttitude() { return this->R * Rs; }

        void updateColdetModelPosition() {
            coldetModel->setPosition(R, p);
        }

        Body* body;

        int index; 
        int jointId;  ///< jointId value written in a model file
		
        enum JointType {
            FIXED_JOINT,   ///< fixed joint(0 dof)
            FREE_JOINT,   /// 6-DOF root link
            ROTATIONAL_JOINT,	///< rotational joint (1 dof)
            SLIDE_JOINT	///< translational joint (1 dof)
        };
		
        JointType jointType;

        Link* parent;
        Link* sibling;
        Link* child;

        Vector3	p;      ///< position

        /**
           Internal world attitude.
           In the model computation, it corresponds to the identity matrix
           when all the joint angles of a robot are zero so that multiplication of
           local attitdue matrices can be omitted to simplify the computation.
           If you want to use the original coordinate in the model file,
           use setAttitude() and attitude() to access.
        */
        Matrix3 R;

        Vector3	v;      ///< linear velocity 
        Vector3	w;      ///< angular velocity, omega
        Vector3	dv;     ///< linear acceleration
        Vector3	dw;     ///< derivative of omega

        double q;       ///< joint value
        double dq;      ///< joint velocity
        double ddq;     ///< joint acceleration
        double u;       ///< joint torque

        Vector3 a;      ///< rotational joint axis (self local)
        Vector3 d;      ///< translation joint axis (self local)
        Vector3 b;      ///< relative position (parent local)

        Matrix3 Rs;    ///< relative attitude of the link segment (self local)

        double m;       ///< mass
        Matrix3 I;     ///< inertia tensor (self local, around c)
        Vector3 c;      ///< center of mass (self local)
        Vector3 wc;     ///< R * c + p
        
        Vector3 vo;     ///< translation elements of spacial velocity
        Vector3 dvo;    ///< derivative of vo

        /** A unit vector of angular velocity (the world coordinate) generated by the joint 
            The value is parent->R * a when the joint is the rotational type. */
        Vector3 sw;
		
        /** A unit vector of spatial velocity (the world coordinate) generated by the joint.
            The value is parent->R * d when the joint is the translation type. */
        Vector3 sv;
		
        Vector3 cv;     ///< dsv * dq (cross velocity term)
        Vector3 cw;     ///< dsw * dq (cross velocity term)

        Vector3 fext;   ///< external force 
        Vector3 tauext; ///< external torque (around the world origin)

        // needed ?
        //Vector3			f;      ///< force from the parent link 
        //Vector3			tau;    ///< torque from the parent link
		
        Matrix3 Iww;   ///< bottm right block of the articulated inertia
        Matrix3 Iwv;   ///< bottom left block (transpose of top right block) of the articulated inertia
        Matrix3 Ivv;   ///< top left block of the articulated inertia
        Vector3 pf;     ///< bias force (linear element)
        Vector3 ptau;   ///< bias force (torque element)
        Vector3 hhv;    ///< top block of Ia*s
        Vector3 hhw;    ///< bottom bock of Ia*s 
        double uu;
        double dd;      ///< Ia*s*s^T  
		
        double Jm2;    ///< Equivalent rotor inertia: n^2*Jm [kg.m^2]

        double ulimit;  ///< the upper limit of joint values
        double llimit;  ///< the lower limit of joint values
        double uvlimit; ///< the upper limit of joint velocities
        double lvlimit; ///< the lower limit of joint velocities

        double defaultJointValue;
        double torqueConst;
        double encoderPulse;
        double Ir;      ///< rotor inertia [kg.m^2]
        double gearRatio;
        double gearEfficiency;
        double rotorResistor;

        bool isHighGainMode;

        ColdetModelPtr coldetModel;

        struct ConstraintForce {
            Vector3 point;
            Vector3 force;
        };

        typedef std::vector<ConstraintForce> ConstraintForceArray;
        ConstraintForceArray constraintForces;

        double  subm;			///< mass of subtree
        Vector3 submwc;			///< sum of m x wc of subtree

      private:

        std::string name_;

        Link& operator=(const Link& link); // no implementation is given to disable the copy operator
        void setBodyIter(Body* body);
        friend std::ostream& ::operator<<(std::ostream &out, Link& link);
        void putInformation(std::ostream& out); // for the iostream output
    };

};
	

#endif
