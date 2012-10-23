/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_TRIANGLE_MESH_SHAPER_H_INCLUDED
#define CNOID_UTIL_TRIANGLE_MESH_SHAPER_H_INCLUDED

#include "VrmlNodes.h"
#include <boost/signal.hpp>
#include "exportdecl.h"

namespace cnoid
{
    class TMSImpl;
    
    class CNOID_EXPORT TriangleMeshShaper
    {
      public:

        TriangleMeshShaper();
        ~TriangleMeshShaper();

        void setDivisionNumber(int n);
        void setNormalGenerationMode(bool on);
        VrmlNodePtr apply(VrmlNodePtr topNode);
        SFNode getOriginalGeometry(VrmlShapePtr shapeNode);
        void defaultTextureMapping(VrmlShape* shapeNode);
        
        boost::signal<void(const std::string& message)> sigMessage;
        
        bool convertBox(VrmlBox* box, VrmlIndexedFaceSetPtr& triangleMesh);

      private:
        TMSImpl* impl;

        void defaultTextureMappingFaceSet(VrmlIndexedFaceSet* triangleMesh);
        void defaultTextureMappingElevationGrid(VrmlElevationGrid* grid, VrmlIndexedFaceSet* triangleMesh);
        void defaultTextureMappingBox(VrmlIndexedFaceSet* triangleMesh);
        void defaultTextureMappingCone(VrmlIndexedFaceSet* triangleMesh);
        void defaultTextureMappingCylinder(VrmlIndexedFaceSet* triangleMesh);
        void defaultTextureMappingSphere(VrmlIndexedFaceSet* triangleMesh, double radius);
        void defaultTextureMappingExtrusion(VrmlIndexedFaceSet* triangleMesh, VrmlExtrusion* extrusion );
        int faceofBox(SFVec3f* point);
        int findPoint(MFVec2f& points, SFVec2f& target);
        double calcangle(SFVec3f& point);
    };

    enum { LEFT, TOP, FRONT, BOTTOM, RIGHT, BACK };

};

#endif

