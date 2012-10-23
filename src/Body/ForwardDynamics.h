/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_H_INCLUDED
#define CNOID_BODY_FORWARD_DYNAMICS_H_INCLUDED

#include "Body.h"
#include <boost/shared_ptr.hpp>
#include <boost/intrusive_ptr.hpp>
#include "exportdecl.h"

namespace cnoid
{
    class AccelSensor;

    /**
       This class calculates the forward dynamics of a Body object
       by using the Featherstone's articulated body algorithm.
       The class also integrates motion using the Euler method or RungeKutta method.
    */
    class CNOID_EXPORT ForwardDynamics {

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        ForwardDynamics(BodyPtr body);
        virtual ~ForwardDynamics();
        
        void setGravityAcceleration(const Vector3& g);
        void setEulerMethod();
        void setRungeKuttaMethod();
        void setTimeStep(double timeStep);
        void enableSensors(bool on);

        virtual void initialize() = 0;
        virtual void calcNextState() = 0;

      protected:

        virtual void initializeSensors();
        virtual void updateSensorsFinal();

        /**
           @brief update position/orientation using spatial velocity
           @param out_p p(t+dt)
           @param out_R R(t+dt)
           @param p0 p(t)
           @param R0 R(t)
           @param w angular velocity
           @param v0 spatial velocity
           @param dt time step[s]
        */
        static void SE3exp(Vector3& out_p, Matrix3& out_R,
                           const Vector3& p0, const Matrix3& R0,
                           const Vector3& w, const Vector3& vo, double dt);
		
        BodyPtr body;
        Vector3 g;
        double timeStep;
        bool sensorsEnabled;

        enum { EULER_METHOD, RUNGEKUTTA_METHOD } integrationMode;

      private:

        void updateAccelSensor(AccelSensor* sensor);
        void initializeAccelSensors();

        // varialbes for calculating sensor values
        // preview control gain matrices for force sensors
        Matrix2 A;
        Vector2 B;
    };

    typedef boost::shared_ptr<ForwardDynamics> ForwardDynamicsPtr;
	
};

#endif
