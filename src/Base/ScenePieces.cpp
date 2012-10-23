/**
   @author Shin'ichiro Nakaoka
*/

#include "ScenePieces.h"
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/LightModel>
#include <osg/LineWidth>
#include <osg/Depth>

using namespace std;
using namespace cnoid;

namespace {

    osg::Geometry* createCrossGeometry(const osg::Vec4& color, float size, float lineWidth)
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::StateSet* state = geom->getOrCreateStateSet();

        state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        //state->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
        //state->setMode(GL_BLEND, osg::StateAttribute::ON);
        osg::Material* mat = new osg::Material;
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
        state->setAttribute(mat);

        if(lineWidth != 1.0f){
            state->setAttribute(new osg::LineWidth(lineWidth));
        }
    
        osg::Vec3Array* v = new osg::Vec3Array;
        geom->setVertexArray(v);

        v->push_back(osg::Vec3(-size,  0.0f,  0.0f));
        v->push_back(osg::Vec3( size,  0.0f,  0.0f));
        v->push_back(osg::Vec3( 0.0f, -size,  0.0f));
        v->push_back(osg::Vec3( 0.0f,  size,  0.0f));
        v->push_back(osg::Vec3( 0.0f,  0.0f, -size));
        v->push_back(osg::Vec3( 0.0f,  0.0f,  size));
        
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));

        return geom;
    }
}


SceneMarker::~SceneMarker()
{

}


void SceneMarker::setPosition(const Vector3& p)
{
    setMatrix(osg::Matrix(1.0,  0.0,  0.0,  0.0,
                          0.0,  1.0,  0.0,  0.0,
                          0.0,  0.0,  1.0,  0.0,
                          p(0), p(1), p(2), 1.0));
}


CrossMarker::CrossMarker(const osg::Vec4& color, float size, float lineWidth)
    : color(color),
      lineWidth(lineWidth)
{
    geode = new osg::Geode;
    geode->addDrawable(createCrossGeometry(color, size, lineWidth));
    geode->setDataVariance(osg::Object::STATIC);
    addChild(geode.get());
}


void CrossMarker::setSize(float size)
{
    geode->removeDrawables(0);
    geode->addDrawable(createCrossGeometry(color, size, lineWidth));
}


SphereMarker::SphereMarker(float radius, const osg::Vec4& color)
{
    sphere = new osg::Sphere;
    sphere->setRadius(radius);
    osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere.get());
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(drawable);

    osg::StateSet* ss = drawable->getOrCreateStateSet();

    osg::Material* material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    material->setEmission(osg::Material::FRONT_AND_BACK, color);
    ss->setAttribute(material, osg::StateAttribute::OVERRIDE);

    // The code to enable alpha blending
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(false);
    ss->setAttributeAndModes(depth, osg::StateAttribute::ON);

    geode->setDataVariance(osg::Object::STATIC);
    addChild(geode);
}


void SphereMarker::setRadius(float r)
{
    sphere->setRadius(r);
    
}


void SphereMarker::setCross(const osg::Vec4& color, float size, float lineWidth)
{
    osg::Geode* geode = dynamic_cast<osg::Geode*>(getChild(0));
    if(geode){
        if(cross){
            geode->removeDrawable(cross.get());
        }
        cross = createCrossGeometry(color, size, lineWidth);
        geode->addDrawable(cross);
    }
}


BBMarker::BBMarker(const osg::BoundingBox& bbox, const osg::Vec4& color, float width)
{
    create(bbox, color, width);
}


BBMarker::BBMarker(const osg::BoundingBox& bbox, const osg::Vec4& color)
{
    double h = bbox.xMax() - bbox.xMin();
    double w = bbox.yMax() - bbox.yMin();
    double d = bbox.zMax() - bbox.zMin();
    double width = (h + w + d) / 3.0 / 10.0;
    create(bbox, color, width);
}


void BBMarker::create(const osg::BoundingBox& bbox, const osg::Vec4& color, float width)
{
    const double& x0 = bbox.xMin();
    const double& x1 = bbox.xMax();
    const double& y0 = bbox.yMin();
    const double& y1 = bbox.yMax();
    const double& z0 = bbox.zMin();
    const double& z1 = bbox.zMax();
    
    addMarker(x0, y0, z0, width);
    addMarker(x0, y0, z1, width);
    addMarker(x0, y1, z0, width);
    addMarker(x0, y1, z1, width);

    addMarker(x1, y0, z0, width);
    addMarker(x1, y0, z1, width);
    addMarker(x1, y1, z0, width);
    addMarker(x1, y1, z1, width);

    osg::StateSet* ss = getOrCreateStateSet();

    osg::Material* material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    material->setEmission(osg::Material::FRONT_AND_BACK, color);
    ss->setAttribute(material, osg::StateAttribute::OVERRIDE);

    // The code to enable alpha blending
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(false);
    ss->setAttributeAndModes(depth, osg::StateAttribute::ON);

    setDataVariance(osg::Object::STATIC);
}


void BBMarker::addMarker(double x, double y, double z, float width)
{
    addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(x, y, z), width)));
}


AttitudeDragger::AttitudeDragger()
{
    currentParent = 0;

    static const int numDivisions = 30;
    static const float beltWidthRatio = 0.1f;
    static const int axes[3][3] = { { 0, 1, 2 }, { 1, 0, 2 }, { 2, 1, 0 } };
    static const char* axisNames[3] = { "x", "y", "z" };

    // make dragger belts
    for(int i=0; i < 3; ++i){
        const int x = axes[i][0];
        const int y = axes[i][1];
        const int z = axes[i][2];

        osg::Geometry* geometry = new osg::Geometry();
        osg::Vec3Array* vertices = new osg::Vec3Array();
        osg::DrawElementsUShort* face = new osg::DrawElementsUShort(osg::PrimitiveSet::QUADS, 0);

        for(int j=0; j < numDivisions; ++j){

            static const double PI = 3.14159265358979323846;
            const double theta = j * 2.0 * PI / numDivisions;
            osg::Vec3 v1, v2;
            v1[x] = beltWidthRatio;
            v2[x] = -beltWidthRatio;
            v1[y] = v2[y] = cos(theta);
            v1[z] = v2[z] = sin(theta);
            vertices->push_back(v1);
            vertices->push_back(v2);

            const int s = j * 2;
            const int t = ((j + 1) % numDivisions) * 2;
            face->push_back(s);
            face->push_back(t);
            face->push_back(t + 1);
            face->push_back(s + 1);
        }

        geometry->setVertexArray(vertices);
        geometry->addPrimitiveSet(face);

        osg::Geode* geode = new osg::Geode;
        geode->setName(axisNames[i]);
        geode->addDrawable(geometry);

        osg::StateSet* state = geode->getOrCreateStateSet();
        osg::Material* mat = new osg::Material;
        osg::Vec4 color(0.2f, 0.2f, 0.2f, 0.4f);
        color[i] = 1.0f;
        mat->setEmission(osg::Material::FRONT_AND_BACK, color);
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 0.4f));
        state->setAttribute(mat);

        geode->setDataVariance(osg::Object::STATIC);
        
        addChild(geode);
        draggers[i] = geode;
    }

    osg::StateSet* state = getOrCreateStateSet();
    osg::LightModel* lightModel = new osg::LightModel;
    lightModel->setTwoSided(true);
    state->setAttributeAndModes(lightModel);

    osg::Material* material = new osg::Material;
    state->setAttribute(material);
    state->setMode(GL_BLEND, osg::StateAttribute::ON);
    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(false);
    state->setAttributeAndModes(depth, osg::StateAttribute::ON);

    state->setDataVariance(osg::Object::STATIC);
}


void AttitudeDragger::attachTo(osg::Group* parent)
{
    if(currentParent){
        currentParent->removeChild(this);
        currentParent = 0;
    }

    if(parent){
        radius_ = parent->getBound().radius();
        osg::Matrix S;
        S.makeScale(radius_, radius_, radius_);
        setMatrix(S);
        parent->addChild(this);
        currentParent = parent;
    }
}
