/** \file
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H_INCLUDED
#define CNOID_BODY_CONSTRAINT_FORCE_SOLVER_H_INCLUDED

#include "exportdecl.h"

namespace cnoid
{
    class Link;
    class CFSImpl;
    class WorldBase;
	
    class CNOID_EXPORT ConstraintForceSolver
    {
        CFSImpl* impl;
		
      public:
        ConstraintForceSolver(WorldBase& world);
        ~ConstraintForceSolver();
		
        bool addCollisionCheckLinkPair
            (int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double culling_thresh, double epsilon);
        void clearCollisionCheckLinkPairs();

        void setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError);
        bool enableJointRangeStopper(bool isEnabled);
        bool enableVelocityOverwriting(bool isEnabled);
        void enableConstraintForceOutput(bool on);
        void setNegativeVelocityRatioForPenetration(double ratio);

        void initialize(void);
        void solve();
        void clearExternalForces();
    };
};


#endif
