
#ifndef CNOID_COLLISION_COLLISION_DATA_H_INCLUDED
#define CNOID_COLLISION_COLLISION_DATA_H_INCLUDED

#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

    // this is for the client
    
    class collision_data
    {
      public:
        int id1;
        int id2;
        
        int num_of_i_points;
        Vector3 i_points[4];
        int i_point_new[4];
        
        Vector3 n_vector;
        double depth;
        
        Vector3 n; // normal vector of triangle id1
        Vector3 m; // normal vector of triangle id2
        int c_type; // c_type=1 for vertex-face contact, c_type=2 for edge-edge contact
    };
}
    
#endif
