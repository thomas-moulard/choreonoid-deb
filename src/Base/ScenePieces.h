/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SCENE_PIECES_H_INCLUDED
#define CNOID_GUIBASE_SCENE_PIECES_H_INCLUDED

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT SceneMarker : public osg::MatrixTransform
    {
      public:
        virtual ~SceneMarker();
        void setPosition(const Vector3& p);
    };

    class CNOID_EXPORT CrossMarker : public SceneMarker
    {
      public:
        CrossMarker(const osg::Vec4& color, float size, float lineWidth = 1.0f);
        void setSize(float size);
      private:
        osg::ref_ptr<osg::Geode> geode;
        osg::Vec4 color;
        float lineWidth;
    };

    class CNOID_EXPORT SphereMarker : public SceneMarker
    {
      public:
        SphereMarker(float radius, const osg::Vec4& color);
        void setRadius(float radius);
        void setCross(const osg::Vec4& color, float size, float lineWidth = 1.0f);
      private:
        osg::ref_ptr<osg::Sphere> sphere;
        osg::ref_ptr<osg::Geometry> cross;
    };

    class CNOID_EXPORT BBMarker : public osg::Geode
    {
      public:
        BBMarker(const osg::BoundingBox& bbox, const osg::Vec4& color, float width);
        BBMarker(const osg::BoundingBox& bbox, const osg::Vec4& color);
        
      private:
        void create(const osg::BoundingBox& bbox, const osg::Vec4& color, float width);
        void addMarker(double x, double y, double z, float width);
    };


    class CNOID_EXPORT AttitudeDragger : public osg::MatrixTransform
    {
      public:
        AttitudeDragger();
        void attachTo(osg::Group* parent);
        inline void detach() { attachTo(0); }
        inline float radius() { return radius_; }
        
      private:
        osg::ref_ptr<osg::Geode> draggers[3];
        osg::Group* currentParent;
        float radius_;
    };
}

#endif
