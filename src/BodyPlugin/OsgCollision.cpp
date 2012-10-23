/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "OsgCollision.h"
#include <cnoid/OsgViewer>

#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}


OsgCollision::OsgCollision()
{
    ppairs = 0;
    setSupportsDisplayList(false);
}


OsgCollision::OsgCollision(const OsgCollision& org, const osg::CopyOp& copyop)
    : osg::Drawable(org, copyop)
{
    ppairs = org.ppairs;
    setSupportsDisplayList(false);
}


/**
   \todo Use displaylists
*/
void OsgCollision::drawImplementation(osg::RenderInfo& ri) const
{
    OsgViewer* viewer = dynamic_cast<OsgViewer*>(ri.getView());

    if(viewer && !viewer->isCollisionVisibleMode()){
        return;
    }
    
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glColor3d(0.0, 1.0, 0.0);
    glBegin(GL_LINES);

    const std::vector<ColdetLinkPairPtr>& pairs = *ppairs;

    for(size_t i=0; i < pairs.size(); ++i){
        const std::vector<collision_data>& cols = pairs[i]->collisions();
        for(size_t j=0; j < cols.size(); ++j){
            const collision_data& col = cols[j];
            const Vector3 n = 50.0 * col.depth * col.n_vector;
            for(int k=0; k < col.num_of_i_points; ++k){
                if(col.i_point_new[k]){
                    const Vector3& p = col.i_points[k];
                    glVertex3dv(p.data());
                    glVertex3dv(Vector3(p + n).data());
                }
            }
        }
    }

    glEnd();
    glPopAttrib();
}


/**
   @todo improved the implementation.
*/
osg::BoundingBox OsgCollision::computeBound() const
{
    osg::BoundingBox bbox;
    return bbox;
}
