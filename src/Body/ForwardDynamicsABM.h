/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_ABM_H_INCLUDED
#define CNOID_BODY_FORWARD_DYNAMICS_ABM_H_INCLUDED

#include "ForwardDynamics.h"
#include <boost/intrusive_ptr.hpp>
#include <vector>
#include "exportdecl.h"

namespace cnoid
{
    class LinkTraverse;
    class AccelSensor;
    class ForceSensor;

    /**
	   Forward dynamics calculation using Featherstone's Articulated Body Method (ABM)
    */
    class CNOID_EXPORT ForwardDynamicsABM : public ForwardDynamics {

    public:
        
        ForwardDynamicsABM(BodyPtr body);
        ~ForwardDynamicsABM();
        
        virtual void initialize();
        virtual void calcNextState();

    private:
        
        void calcMotionWithEulerMethod();
        void integrateRungeKuttaOneStep(double r, double dt);
        void calcMotionWithRungeKuttaMethod();

        /**
           compute position/orientation/velocity
         */
        void calcABMPhase1();

        /**
           compute articulated inertia
         */
        void calcABMPhase2();
        void calcABMPhase2Part1();
        void calcABMPhase2Part2();

        /**
           compute joint acceleration/spatial acceleration
         */
        void calcABMPhase3();

        inline void calcABMFirstHalf();
        inline void calcABMLastHalf();

        void updateForceSensors();
        void updateForceSensor(ForceSensor* sensor);
		
        // Buffers for the Runge Kutta Method
        Vector3 p0;
        Matrix3 R0;
        Vector3 vo0;
        Vector3 w0;
        std::vector<double> q0;
        std::vector<double> dq0;
		
        Vector3 vo;
        Vector3 w;
        Vector3 dvo;
        Vector3 dw;
        std::vector<double> dq;
        std::vector<double> ddq;

    };
	
};

#endif
