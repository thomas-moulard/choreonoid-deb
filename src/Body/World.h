/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_WORLD_H_INCLUDED
#define CNOID_BODY_WORLD_H_INCLUDED

#include "Body.h"
#include "ForwardDynamics.h"
#include <map>
#include "exportdecl.h"

namespace cnoid {

    class Link;

    class CNOID_EXPORT WorldBase
    {
      public:
        WorldBase();
        virtual ~WorldBase();

        /**
           @brief get the number of bodies in this world
           @return the number of bodies
        */ 
        inline int numBodies() { return bodyInfoArray.size(); }

        /**
           @brief get body by index
           @param index of the body
           @return body
        */
        BodyPtr body(int index);

        /**
           @brief get body by name
           @param name of the body
           @return body
        */
        BodyPtr body(const std::string& name);

        /**
           @brief get forward dynamics computation method for body
           @param index index of the body
           @return forward dynamics computation method
        */
        inline ForwardDynamicsPtr forwardDynamics(int index) {
            return bodyInfoArray[index].forwardDynamics;
        }

        /**
           @brief get index of body by name
           @param name of the body
           @return index of the body
        */
        int bodyIndex(const std::string& name);

        /**
           @brief add body to this world
           @param body
           @return index of the body
           @note This must be called before initialize() is called.
        */
        int addBody(BodyPtr body);

        /**
           @brief clear bodies in this world
        */
        void clearBodies();

        /**
           @brief clear collision pairs
        */
        void clearCollisionPairs();

        /**
           @brief set time step
           @param dt time step[s]
        */
        void setTimeStep(double dt);

        /**
           @brief get time step
           @return time step[s]
        */
        double timeStep(void) const { return timeStep_; }
	
        /**
           @brief set current time
           @param tm current time[s]
        */
        void setCurrentTime(double tm);

        /**
           @brief get current time
           @return current time[s]
        */
        double currentTime(void) const { return currentTime_; }
	
        /**
           @brief set gravity acceleration
           @param g gravity acceleration[m/s^2]
        */
        void setGravityAcceleration(const Vector3& g);

        /**
           @brief get gravity acceleration
           @return gravity accleration
        */
        inline const Vector3& gravityAcceleration() { return g; }

        /**
           @brief enable/disable sensor simulation
           @param on true to enable, false to disable
           @note This must be called before initialize() is called.
        */
        void enableSensors(bool on);

        /**
           @brief choose euler method for integration
        */
        void setEulerMethod();

        /**
           @brief choose runge-kutta method for integration
        */
        void setRungeKuttaMethod();

        /**
           @brief initialize this world. This must be called after all bodies are registered.
        */
        virtual void initialize();

        /**
           @brief compute forward dynamics and update current state
        */
        virtual void calcNextState();

        /**
           @brief get index of link pairs
           @param link1 link1
           @param link2 link2
           @return pair of index and flag. The flag is true if the pair was already registered, false othewise.
        */
        std::pair<int,bool> getIndexOfLinkPairs(Link* link1, Link* link2);

      protected:

        double currentTime_;
        double timeStep_;

        struct BodyInfo {
            BodyPtr body;
            ForwardDynamicsPtr forwardDynamics;
        };
        std::vector<BodyInfo> bodyInfoArray;

        bool sensorsAreEnabled;

      private:
        typedef std::map<std::string, int> NameToIndexMap;
        NameToIndexMap nameToBodyIndexMap;

        typedef std::map<BodyPtr, int> BodyToIndexMap;
        BodyToIndexMap bodyToIndexMap;
		
        struct LinkPairKey {
            Link* link1;
            Link* link2;
            bool operator<(const LinkPairKey& pair2) const;
        };
        typedef std::map<LinkPairKey, int> LinkPairKeyToIndexMap;
        LinkPairKeyToIndexMap linkPairKeyToIndexMap;

        int numRegisteredLinkPairs;
		
        Vector3 g;

        bool isEulerMethod; // Euler or Runge Kutta ?

    };


    template <class TConstraintForceSolver> class World : public WorldBase
    {
    public:
        TConstraintForceSolver constraintForceSolver;

        World() : constraintForceSolver(*this) { }

        virtual void initialize() {
            WorldBase::initialize();
            constraintForceSolver.initialize();
        }

        virtual void calcNextState(){
            constraintForceSolver.solve();
            WorldBase::calcNextState();
        }
    };

};

#endif
