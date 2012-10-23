
#include "OsgNormalVisualizer.h"

#include <osg/Geometry>
#include <osg/NodeVisitor>

using namespace cnoid;

namespace {
    
    class NormalExtractor : public osg::NodeVisitor
    {
    public:
        NormalExtractor(float normalScale = 1.0, NormalVisualizer::Mode mode = NormalVisualizer::SURFACE);
        
        void apply(osg::Geode& geode);
        
        osg::Vec3Array* getNormalLines() { return normalLines.get(); }
        
    private:
        NormalVisualizer::Mode mode;
        float scale;

        osg::ref_ptr<osg::Vec3Array> normalLines;

        osg::Vec3Array* vertices;
        osg::Vec3Array::iterator vertexIter;
        osg::IntArray::iterator vertexIndexIter;
        osg::Vec3Array* normals;
        osg::Vec3Array::iterator normalIter;
        osg::UIntArray::iterator normalIndexIter;
        
        void extractOverallNormal();
        void extractPrimitiveSetNormal(const osg::PrimitiveSet* primitiveSet);
        void extractPrimitiveNormals(
            const osg::PrimitiveSet* primitiveSet, osg::Geometry::AttributeBinding binding, int numVertices);
        void extractPolygonNormals(const osg::PrimitiveSet* primitiveSet);
    };
}    


NormalVisualizer::NormalVisualizer(Node *node, float scale, Mode mode)
{
    NormalExtractor extractor(scale, mode);
    node->accept(extractor);

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    osg::ref_ptr<osg::Vec3Array> lines = extractor.getNormalLines();
    geom->setVertexArray(lines.get());
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, lines->size()));

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    if(mode == SURFACE){
        colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
    } else if(mode == VERTEX) {
        colors->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0));
    }

    osg::StateSet* state = geom->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    addDrawable(geom.get());
}


NormalExtractor::NormalExtractor(float normalScale, NormalVisualizer::Mode mode)
    : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN),
      mode(mode),
      scale(normalScale)
{
    normalLines = new osg::Vec3Array;
}


void NormalExtractor::apply(osg::Geode& geode)
{
    for(size_t i = 0; i < geode.getNumDrawables(); i++ ){
        osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
        if(geom){
            vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
            if(!vertices) continue;
            normals  = dynamic_cast<osg::Vec3Array*>(geom->getNormalArray());
            if(!normals) continue;
            osg::Geometry::AttributeBinding binding = geom->getNormalBinding();
            if(binding == osg::Geometry::BIND_OFF) continue;

            if(binding == osg::Geometry::BIND_OVERALL){
                extractOverallNormal();

            } else {
                osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
                
                vertexIter = vertices->begin();
                normalIter = normals->begin();

                osg::IntArray* vertexIndices = dynamic_cast<osg::IntArray*>(geom->getVertexIndices());
                if(vertexIndices){
                    vertexIndexIter = vertexIndices->begin();
                }
                osg::UIntArray* normalIndices = dynamic_cast<osg::UIntArray*>(geom->getNormalIndices());
                if(normalIndices){
                    normalIndexIter = normalIndices->begin();
                }
                
                osg::Geometry::PrimitiveSetList::iterator it;
                for(it = primitiveSets.begin(); it != primitiveSets.end(); ++it){
                    osg::PrimitiveSet* primitiveSet = it->get();

                    if(binding == osg::Geometry::BIND_PER_PRIMITIVE_SET){
                        extractPrimitiveSetNormal(primitiveSet);
                    } else {
                        switch(primitiveSet->getMode()){

                        case osg::PrimitiveSet::TRIANGLES:
                            extractPrimitiveNormals(primitiveSet, binding, 3);
                            break;

                        case osg::PrimitiveSet::QUADS:
                            extractPrimitiveNormals(primitiveSet, binding, 4);
                            break;

                        case osg::PrimitiveSet::POLYGON:
                            extractPolygonNormals(primitiveSet);
                            break;

                        default:
                            break;
                        }
                    }
                }
            }
        }
    }
    traverse(geode);
}


void NormalExtractor::extractOverallNormal()
{
    osg::Vec3 v(0.0, 0.0, 0.0);
    const int n = vertices->size();
    for(int i=0; i < n; ++i){
        v += (*vertices)[i];
    }
    v /= n;
    normalLines->push_back(v);
    normalLines->push_back(v + normals->front() * scale);
}


void NormalExtractor::extractPrimitiveSetNormal(const osg::PrimitiveSet* primitiveSet)
{
    osg::Vec3 v(0.0, 0.0, 0.0);
    int n = primitiveSet->getNumIndices();
    for(int i = 0; i < n; ++i){
        v += (*vertices)[i];
    }
    v /= n;
    normalLines->push_back(v);
    normalLines->push_back(v + *normalIter++ * scale);
}


void NormalExtractor::extractPrimitiveNormals
(const osg::PrimitiveSet* primitiveSet, osg::Geometry::AttributeBinding binding, int numVertices)
{
    for(size_t i=0; i < primitiveSet->getNumPrimitives(); ++i){
        
        if(mode == NormalVisualizer::SURFACE || binding == osg::Geometry::BIND_PER_PRIMITIVE){
            osg::Vec3 n(0.0, 0.0, 0.0);
            if(binding == osg::Geometry::BIND_PER_PRIMITIVE){
                n = *(normalIter++);
            } else if(binding == osg::Geometry::BIND_PER_VERTEX){
                for(int j = 0; j < numVertices; ++j){
                    n += *(normalIter++);
                }
                n /= numVertices; 
            }
            
            osg::Vec3 v(0.0, 0.0, 0.0);
            for(int j = 0; j < numVertices; ++j){
                v += *(vertexIter++);
            }
            v /= numVertices;
            
            normalLines->push_back(v);
            normalLines->push_back(v + n * scale);
            
        } else if(mode == NormalVisualizer::VERTEX){
            for(int j = 0; j < numVertices; ++j){
                osg::Vec3& v = *(vertexIter++);
                osg::Vec3& n = *(normalIter++);
                normalLines->push_back(v);
                normalLines->push_back(v + n * scale);
            }
        }
    }
}


void NormalExtractor::extractPolygonNormals(const osg::PrimitiveSet* primitiveSet)
{
    const osg::DrawArrayLengths* polygonSizes = static_cast<const osg::DrawArrayLengths*>(primitiveSet);

    for(size_t i = 0; i < polygonSizes->size(); ++i){
        int numVertices = (*polygonSizes)[i];
        for(int j=0; j < numVertices; ++j){
            osg::Vec3& v = (*vertices)[*vertexIndexIter++];
            osg::Vec3& n = (*normals)[*normalIndexIter++];
            normalLines->push_back(v);
            normalLines->push_back(v + n * scale);
        }
    }
}
