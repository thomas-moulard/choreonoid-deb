/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_TRIANGULATOR_H_INCLUDED
#define CNOID_UTIL_TRIANGULATOR_H_INCLUDED

#include "VrmlNodes.h"
#include <boost/dynamic_bitset.hpp>

namespace cnoid {
    
    class Triangulator
    {
    public:

        inline void setVertices(const MFVec3f& vertices) {
            this->vertices = &vertices;
        }

        /**
           @return The number of triangles
        */
        int apply(const std::vector<int>& polygon);

        /**
           Triangulated indices.
           This value is available after calling the 'apply' method.
           The indices are local ones in the polygon index vector given to the apply method.
        */
        inline const std::vector<int>& triangles() {
            return triangles_;
        }

    private:

        enum Convexity { FLAT, CONVEX, CONCAVE };
        
        const MFVec3f* vertices;
        const std::vector<int>* orgPolygon;                                                                  
        std::vector<int> triangles_;
        std::vector<int> workPolygon;
        SFVec3f ccs; // cyclic cross sum
        boost::dynamic_bitset<> earMask;

        inline const SFVec3f& vertex(int localIndex){
            return (*vertices)[(*orgPolygon)[localIndex]];
        }

        inline const SFVec3f& workVertex(int workPolygonIndex){
            return (*vertices)[(*orgPolygon)[workPolygon[workPolygonIndex]]];
        }

        Convexity calcConvexity(int ear);
        bool checkIfEarContainsOtherVertices(int ear);
    };

}

#endif
