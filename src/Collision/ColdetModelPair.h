/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_COLLISION_COLDET_MODEL_PAIR_H_INCLUDED
#define CNOID_COLLISION_COLDET_MODEL_PAIR_H_INCLUDED

#include "CollisionData.h"
#include "ColdetModel.h"
#include "CollisionPairInserter.h"
#include <cnoid/Referenced>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT ColdetModelPair : public Referenced
    {
      public:
        ColdetModelPair();
        ColdetModelPair(ColdetModelPtr model0, ColdetModelPtr model1,
                        double tolerance=0);
        ColdetModelPair(const ColdetModelPair& org);
        virtual ~ColdetModelPair();

        void set(ColdetModelPtr model0, ColdetModelPtr model1);

        inline ColdetModel* model(int index) { return models[index].get(); }

        inline std::vector<collision_data>& detectCollisions() {
            return detectCollisionsSub(true);
        }

        inline std::vector<collision_data>& collisions() {
            return collisionPairInserter.cdContact;
        }

        inline void clearCollisions(){
            collisionPairInserter.cdContact.clear();
        }

        inline bool checkCollision() {
            return !detectCollisionsSub(false).empty();
        }

        double computeDistance(double *point0, double *point1);

        /**
           @param out_triangle0, out_triangle1 Indices of the triangle pair that are originally registered by ColdeModel::setTraiangle().
           @param out_point0, out_point1 The closest points 
        */
        double computeDistance(int& out_triangle0, double* out_point0, int& out_triangle1, double* out_point1);

        bool detectIntersection();

        double tolerance() const { return tolerance_; }
        void setTolerance(double tolerance){
            tolerance_ = tolerance;
        }

      private:
        std::vector<collision_data>& detectCollisionsSub(bool detectAllContacts);
        bool detectMeshMeshCollisions(bool detectAllContacts);
        bool detectPlaneCylinderCollisions(bool detectAllContacts);

        ColdetModelPtr models[2];
        double tolerance_;

        CollisionPairInserter collisionPairInserter;

        int boxTestsCount;
        int triTestsCount;
    };

    typedef boost::intrusive_ptr<ColdetModelPair> ColdetModelPairPtr;
}


#endif
