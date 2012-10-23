
/**
   @author Shin'ichiro Nakaoka
*/

#include "VrmlToOsgConverter.h"

#include <map>
#include <string>
#include <iostream>
#include <typeinfo>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/FrontFace>
#include <osg/LightModel>
#include <osg/Depth>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool AVOID_SPECULAR_BUG_OF_OPENGL_DRIVER = true;
}


namespace cnoid {

    class VrmlToOsgConverterImpl
    {
    public:
        VrmlToOsgConverterImpl();
        osg::Node* convert(VrmlNode* vrmlNode);
        osg::Node* convertNode(VrmlNode* vnode);
        osg::Node* convertGroupNode(VrmlGroup* vgroup);
        osg::Group* createTransformNode(VrmlTransform* vt);
        osg::Node* convertShapeNode(VrmlNode* vnode);
        osg::Material* createMaterial(VrmlMaterial* vm);
        osg::Geometry* createGeometryFromIndexedFaceSet(VrmlIndexedFaceSet* vface, float alpha);
        
        VrmlMaterialPtr defaultMaterial;
        osg::ref_ptr<osg::StateSet> stateSetForTransformWithScaling;
        osg::ref_ptr<osg::StateSet> stateSetForTransformWithUniformScaling;

        typedef map<VrmlNode*, osg::Material*> VrmlNodeToOsgMaterialMap;
        VrmlNodeToOsgMaterialMap vrmlNodeToOsgMaterialMap;
        
        typedef map<VrmlNode*, osg::Node*> VrmlNodeToOsgNodeMap;
        VrmlNodeToOsgNodeMap vrmlNodeToOsgNodeMap;

        typedef map<VrmlGeometry*, osg::Geometry*> VrmlGeometryToOsgGeometryMap;
        VrmlGeometryToOsgGeometryMap vrmlGeometryToOsgGeometryMap;

        osgUtil::Optimizer optimizer;
    };
}


VrmlToOsgConverter::VrmlToOsgConverter()
{
    impl = new VrmlToOsgConverterImpl();
}


VrmlToOsgConverterImpl::VrmlToOsgConverterImpl()
{
    defaultMaterial = new VrmlMaterial();

    stateSetForTransformWithScaling = new osg::StateSet();
    stateSetForTransformWithScaling->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    stateSetForTransformWithUniformScaling = new osg::StateSet();
    stateSetForTransformWithUniformScaling->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
}


VrmlToOsgConverter::~VrmlToOsgConverter()
{
    delete impl;
}


osg::Node* VrmlToOsgConverter::convert(VrmlNodePtr vrmlNode)
{
    return impl->convert(vrmlNode.get());
}


osg::Node* VrmlToOsgConverterImpl::convert(VrmlNode* vrmlNode)
{
    osg::Node* node = convertNode(vrmlNode);

    if(node){
        // needed ?
        osg::StateSet* state = node->getOrCreateStateSet();
        osg::LightModel* lightModel = new osg::LightModel;
        lightModel->setTwoSided(true);
        state->setAttributeAndModes(lightModel);

        optimizer.optimize(node);
    }

    return node;
}


osg::Node* VrmlToOsgConverterImpl::convertNode(VrmlNode* vnode)
{
    osg::Node* node = 0;

    VrmlNodeToOsgNodeMap::iterator p = vrmlNodeToOsgNodeMap.find(vnode);
    if(p != vrmlNodeToOsgNodeMap.end()){
        node = p->second;
    } else {

        if(VrmlProtoInstance* protoInstance = dynamic_cast<VrmlProtoInstance*>(vnode)){
            vnode = protoInstance->actualNode.get();
        }

        if(vnode){
            if(VrmlGroup* group = dynamic_cast<VrmlGroup*>(vnode)){
                node = convertGroupNode(group);
            } else if(VrmlShape* shape = dynamic_cast<VrmlShape*>(vnode)){
                node = convertShapeNode(shape);
            }

            if(node){
                node->setName(vnode->defName);
                vrmlNodeToOsgNodeMap.insert(make_pair(vnode, node));
            }
        }
    }
        
    return node;
}


osg::Node* VrmlToOsgConverterImpl::convertGroupNode(VrmlGroup* vgroup)
{
    osg::Group* group;

    if(VrmlTransform* transform = dynamic_cast<VrmlTransform*>(vgroup)){
        group = createTransformNode(transform);
    } else {
        group = new osg::Group;
    }
    
    int num = vgroup->children.size();
    for(int i=0; i < num; i++){
        osg::Node* child = convertNode(vgroup->children[i].get());
        if(child){
            group->addChild(child);
        }
    }
    
    if(group->getNumChildren() == 0){
        group->ref();
        group->unref();
        group = 0;
    }
    
    return group;
}


osg::Group* VrmlToOsgConverterImpl::createTransformNode(VrmlTransform* vt)
{
    osg::MatrixTransform* transform = new osg::MatrixTransform;

    const SFRotation::Vector3& a = vt->rotation.axis();
    osg::Matrix R = osg::Matrix::rotate(vt->rotation.angle(), osg::Vec3d(a[0], a[1], a[2]));
    osg::Matrix T;
    T.setTrans(vt->translation[0], vt->translation[1], vt->translation[2]);
    osg::Matrix C;
    C.setTrans(vt->center[0], vt->center[1], vt->center[2]);
    osg::Matrix Cinv;
    Cinv.invert(C);
    const SFRotation::Vector3& sa = vt->scaleOrientation.axis();
    osg::Vec3d scaleAxis(sa[0], sa[1], sa[2]);
    osg::Matrix SR = osg::Matrix::rotate(vt->scaleOrientation.angle(), scaleAxis);
    osg::Matrix SRinv;
    SRinv.invert(SR);

    const SFVec3f& s = vt->scale;
    osg::Matrix S = osg::Matrix::scale(s[0], s[1], s[2]);
    
    T.preMult(C);
    T.preMult(R);
    T.preMult(SR);
    T.preMult(S);
    T.preMult(SRinv);
    T.preMult(Cinv);
    
    transform->setMatrix(T);

    if((s.array() != 1.0).any()){
        if(s[0] == s[1] && s[1] == s[2]){
            transform->setStateSet(stateSetForTransformWithUniformScaling.get());
        } else {
            transform->setStateSet(stateSetForTransformWithScaling.get());
        }
    }

    return transform;
}


osg::Node* VrmlToOsgConverterImpl::convertShapeNode(VrmlNode* vnode)
{
    VrmlShape* vrmlShape = static_cast<VrmlShape*>(vnode);

    VrmlMaterial* vm = defaultMaterial.get();

    if(vrmlShape->appearance && vrmlShape->appearance->material){
        vm = vrmlShape->appearance->material.get();
    }
    float alpha = 1.0 - vm->transparency;

    osg::Geode* geode = 0;
    osg::Geometry* geometry = 0;
    VrmlGeometry* vrmlGeometry = dynamic_node_cast<VrmlGeometry>(vrmlShape->geometry).get();
    
    if(vrmlGeometry){

        VrmlGeometryToOsgGeometryMap::iterator p = vrmlGeometryToOsgGeometryMap.find(vrmlGeometry);
        if(p != vrmlGeometryToOsgGeometryMap.end()){
            geometry = p->second;
        } else {
            if(VrmlIndexedFaceSet* faceSet = dynamic_cast<VrmlIndexedFaceSet*>(vrmlGeometry)){
                geometry = createGeometryFromIndexedFaceSet(faceSet, alpha);
            }
            if(geometry){
                vrmlGeometryToOsgGeometryMap.insert(make_pair(vrmlGeometry, geometry));
            }
        }
    }
    
    if(geometry){
        geode = new osg::Geode;
        geode->addDrawable(geometry);

        osg::Material* material;

        VrmlNodeToOsgMaterialMap::iterator p = vrmlNodeToOsgMaterialMap.find(vm);
        if(p != vrmlNodeToOsgMaterialMap.end()){
            material = p->second;
        } else {
            material = createMaterial(vm);
            vrmlNodeToOsgMaterialMap.insert(make_pair(vm, material));
        }

        osg::StateSet* stateSet = geode->getOrCreateStateSet();
        stateSet->setAttributeAndModes(material);

        if(alpha < 1.0f){
            // Enable blending, select transparent bin.
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
            stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            
            // Enable depth test so that an opaque polygon will occlude a transparent one behind it.
            stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
            
            // Conversely, disable writing to depth buffer so that
            // a transparent polygon will allow polygons behind it to shine thru.
            // OSG renders transparent polygons after opaque ones.
            osg::Depth* depth = new osg::Depth;
            depth->setWriteMask(false);
            stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);
            
            // Disable conflicting modes.
            //stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        }
    }

    return geode;
}


osg::Material* VrmlToOsgConverterImpl::createMaterial(VrmlMaterial* vm)
{
    osg::Material* material = new osg::Material();

    float alpha = 1.0 - vm->transparency;

    osg::Vec4 diffuse(vm->diffuseColor[0], vm->diffuseColor[1], vm->diffuseColor[2], alpha);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, diffuse);
    float a = vm->ambientIntensity;
    osg::Vec4 ambient(diffuse[0] * a, diffuse[1] * a, diffuse[2] * a, alpha);
    material->setAmbient(osg::Material::FRONT_AND_BACK, ambient);
    osg::Vec4 emission(vm->emissiveColor[0], vm->emissiveColor[1], vm->emissiveColor[2], alpha);
    material->setEmission(osg::Material::FRONT_AND_BACK, emission);

    osg::Material::Face specularFace =
        AVOID_SPECULAR_BUG_OF_OPENGL_DRIVER ? osg::Material::FRONT : osg::Material::FRONT_AND_BACK;
    osg::Vec4 specular(vm->specularColor[0], vm->specularColor[1], vm->specularColor[2], 1.0);
    material->setSpecular(specularFace, specular);
    float shininess = (127.0 * vm->shininess) + 1.0;
    material->setShininess(specularFace, shininess);

    return material;
}


osg::Geometry* VrmlToOsgConverterImpl::createGeometryFromIndexedFaceSet(VrmlIndexedFaceSet* vface, float alpha)
{
    osg::Geometry* geometry = new osg::Geometry;
    
    if(!vface->ccw){
        osg::StateSet* stateSet = geometry->getOrCreateStateSet();
        stateSet->setAttributeAndModes(new osg::FrontFace(osg::FrontFace::CLOCKWISE));
    }

    if(!vface->solid){
        
    }
    
    if(vface->coord){
        osg::Vec3Array* vertices = new osg::Vec3Array;
        MFVec3f& point = vface->coord->point;
        int size = point.size();
        for(int i=0; i < size; i++){
            vertices->push_back(osg::Vec3(point[i][0], point[i][1], point[i][2]));
        }
        geometry->setVertexArray(vertices);
    }
    
    geometry->addPrimitiveSet(new osg::DrawArrayLengths(osg::PrimitiveSet::POLYGON));
    osg::IntArray* vertexIndices = new osg::IntArray();
    int numVertex = 0;
    int numIndices = vface->coordIndex.size();
    
    for(int i=0; i < numIndices; i++) {
        int index = vface->coordIndex[i];
        if (index < 0) {
            ((osg::DrawArrayLengths*)geometry->getPrimitiveSet(0))->push_back(numVertex);
            numVertex = 0;
        } else {
            vertexIndices->push_back(index);
            numVertex++;
        }
    }
    geometry->setVertexIndices(vertexIndices);

    /* old code
    unsigned int i = 0;
    unsigned int begin = 0;
    int numIndices = vface->coordIndex.size();
    while(i < numIndices){
        int index = vface->coordIndex[i];
        if(index < 0){
            int numVertices = i - begin;
            if(numVertices >= 3){
                osg::PrimitiveSet::Mode pmode;
                switch(numVertices){
                case 3:  pmode = osg::PrimitiveSet::TRIANGLES; break;
                case 4:  pmode = osg::PrimitiveSet::QUADS;     break;
                default: pmode = osg::PrimitiveSet::POLYGON;   break;
                }
                osg::DrawElementsUShort* drawElements = new osg::DrawElementsUShort(pmode);
                for(unsigned int j=begin; j < i; ++j){
                    drawElements->push_back(vface->coordIndex[j]);
                }
                geometry->addPrimitiveSet(drawElements);
            }
            begin = i + 1;
        }
        i++;
    }
    */

    if(vface->normal){
        osg::Vec3Array* normals = new osg::Vec3Array;
        MFVec3f& vec = vface->normal->vector;
        int size = vec.size();
        for(int i=0; i < size; i++){
            normals->push_back(osg::Vec3(vec[i][0], vec[i][1], vec[i][2]));
        }
        geometry->setNormalArray(normals);
        
        if(vface->normalIndex.empty()){
            geometry->setNormalIndices(geometry->getVertexIndices());
        } else {
            int size = vface->normalIndex.size();
            osg::UIntArray* indices = new osg::UIntArray;
            for(int i=0; i < size; ++i){
                int index = vface->normalIndex[i];
                if(index >= 0){
                    indices->push_back(index);
                }
            }
            geometry->setNormalIndices(indices);
        }
        if(vface->normalPerVertex == true){
            geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        } else {
            geometry->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE);
        }
    }
    
    if(vface->color){
        MFColor& c = vface->color->color;
        if(!c.empty()){
            osg::Vec4Array* colors = new osg::Vec4Array;
            int size = c.size();
            for(int i=0; i < size; i++){
                colors->push_back(osg::Vec4(c[i][0], c[i][1], c[i][2], alpha));
            }
            geometry->setColorArray(colors);
            
            if(vface->colorIndex.empty()){
                geometry->setColorIndices(geometry->getVertexIndices());
            } else {
                int size = vface->colorIndex.size();
                osg::UIntArray* indices = new osg::UIntArray;
                
                // osg::TemplateIndexArray <unsigned int, osg::Array::UIntArrayType,4,4> *indices;
                // indices = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType,4,4>;
                
                for(int i=0; i < size; i++){
                    int index = vface->colorIndex[i];
                    if(index >= 0){
                        indices->push_back(index);
                    }
                }
                geometry->setColorIndices(indices);
            }
            
            if(vface->colorPerVertex == true){
                geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            } else {
                geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
            }
        }
    }
    
    if(!vface->normal){
        osgUtil::SmoothingVisitor::smooth(*geometry);
    }
    
    return geometry;
}
