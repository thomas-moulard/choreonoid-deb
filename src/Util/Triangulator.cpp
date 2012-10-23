/*!
  @author Shin'ichiro Nakaoka
*/

#include "Triangulator.h"

using namespace std;
using namespace cnoid;

int Triangulator::apply(const vector<int>& polygon)
{
    triangles_.clear();

    size_t numOrgVertices = polygon.size();

    if(numOrgVertices > earMask.size()){
        earMask.resize(numOrgVertices);
    }

    if(numOrgVertices < 3){
        return 0;
    } else if(numOrgVertices == 3){
        triangles_.push_back(0);
        triangles_.push_back(1);
        triangles_.push_back(2);
        return 1;
    }
    
    orgPolygon = &polygon;

    workPolygon.resize(numOrgVertices);
    for(size_t i=0; i < numOrgVertices; ++i){
        workPolygon[i] = i;
    }
    
    ccs.setZero();
    const SFVec3f& o = vertex(0);
    for(size_t i=1; i < numOrgVertices - 1; ++i){
        ccs += (vertex(i) - o).cross(vertex((i+1) % numOrgVertices) - o);
    }

    int numTriangles = 0;

    while(true) {
        int n = workPolygon.size();
        if(n < 3){
            break;
        }
        int target = -1;
        for(int i=0; i < n; ++i){
            Convexity convexity = calcConvexity(i);
            if(convexity == FLAT){
                target = i;
                break;
            } else if(convexity == CONVEX){
                if(!checkIfEarContainsOtherVertices(i)){
                    triangles_.push_back(workPolygon[(i + n - 1) % n]);
                    triangles_.push_back(workPolygon[i]);
                    triangles_.push_back(workPolygon[(i + 1) % n]);
                    target = i;
                    numTriangles++;
                    break;
                }
            }
        }
        if(target < 0){
            break;
        }
        for(int i = target + 1; i < n; ++i){
            workPolygon[target++] = workPolygon[i];
        }
        workPolygon.pop_back();
    }
    
    return numTriangles;
}


Triangulator::Convexity Triangulator::calcConvexity(int ear)
{
    int n = workPolygon.size();

    const SFVec3f& p0 = workVertex((ear + n - 1) % n);
    SFVec3f a = workVertex(ear) - p0;
    SFVec3f b = workVertex((ear + 1) % n) - p0;
    SFVec3f ccs = a.cross(b);

    Convexity convexity;
    
    if((ccs.norm() / (a.norm() + b.norm())) < 1.0e-4){
        convexity = FLAT;
    } else {
        convexity = (this->ccs.dot(ccs) > 0.0) ? CONVEX : CONCAVE;
    }

    return convexity;
}
    

bool Triangulator::checkIfEarContainsOtherVertices(int ear)
{
    bool contains = false;

    const int n = workPolygon.size();

    if(n > 3){
        const int prev = (ear + n -1) % n;
        const int next = (ear+1) % n;
        const SFVec3f& a = workVertex(prev);
        const SFVec3f& b = workVertex(ear);
        const SFVec3f& c = workVertex(next);

        earMask[prev] = earMask[ear] = earMask[next] = 1;

        for(size_t i=0; i < workPolygon.size(); ++i){
            if(!earMask[i]){
                const SFVec3f& p = workVertex(i);
                if(((a - p).cross(b - p)).dot(ccs) <= 0){
                    continue;
                }
                if(((b - p).cross(c - p)).dot(ccs) <= 0){
                    continue;
                }
                if(((c - p).cross(a - p)).dot(ccs) <= 0){
                    continue;
                }
                contains = true;
                break;
            }
        }

        earMask[prev] = earMask[ear] = earMask[next] = 0;
    }

    return contains;
}
