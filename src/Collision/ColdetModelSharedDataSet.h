/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_COLDET_MODEL_SHARED_DATA_SET_H_INCLUDED
#define CNOID_COLDET_MODEL_SHARED_DATA_SET_H_INCLUDED

#include "ColdetModel.h"
#include "Opcode/Opcode.h"
#include <vector>

using namespace std;
using namespace cnoid;

namespace cnoid {

    class ColdetModelSharedDataSet
    {
    public:
        ColdetModelSharedDataSet();

        bool build();

        // need two instances ?
        Opcode::Model model;
        Opcode::MeshInterface iMesh;

        vector<IceMaths::Point> vertices;
        vector<IceMaths::IndexedTriangle> triangles;

        ColdetModel::PrimitiveType pType;
        std::vector<float> pParams;

        inline int getAABBTreeDepth() {
            return AABBTreeMaxDepth;
        };
        inline int getNumofBB(int depth){
            return numBBMap.at(depth);
        };
        inline int getmaxNumofBB(){
            if(AABBTreeMaxDepth>0){
                return numBBMap.at(AABBTreeMaxDepth-1);
            } else {
                return 0;
            }
        };

      private:
        int refCounter;
        int AABBTreeMaxDepth;
        std::vector<int> numBBMap;
        std::vector<int> numLeafMap;
        int computeDepth(const Opcode::AABBCollisionNode* node, int currentDepth, int max );

        friend class ColdetModel;
    };
}

#endif
