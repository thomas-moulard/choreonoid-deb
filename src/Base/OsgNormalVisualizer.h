
#ifndef CNOID_GUIBASE_OSG_NORMAL_VISUALIZER_H_INCLUDED
#define CNOID_GUIBASE_OSG_NORMAL_VISUALIZER_H_INCLUDED

#include <osg/Geode>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT NormalVisualizer : public osg::Geode 
    {
      public:
        enum Mode { SURFACE, VERTEX };
        NormalVisualizer(osg::Node *node, float scale = 0.01, Mode mode = SURFACE);
      protected:
        ~NormalVisualizer() {}
    };
    

    class CNOID_EXPORT SurfaceNormalVisualizer : public NormalVisualizer
    {
      public:
        SurfaceNormalVisualizer(Node *node, float scale = 0.01)
            : NormalVisualizer(node, scale, SURFACE) { }
      protected:
        ~SurfaceNormalVisualizer() { }
    };
  
  
    class CNOID_EXPORT VertexNormalVisualizer : public NormalVisualizer
    {
      public:
        VertexNormalVisualizer( Node *node, float scale = 0.01)
            : NormalVisualizer(node, scale, VERTEX) { }
      protected:
        ~VertexNormalVisualizer() { }
    };
  
}

#endif
