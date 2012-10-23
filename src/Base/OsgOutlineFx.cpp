
#include "OsgOutlineFx.h"

#include <osgFX/Registry>
#include <osg/Stencil>
#include <osg/CullFace>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/NodeCallback>
#include <osgUtil/CullVisitor>

using namespace cnoid;

namespace osgFX
{
    Registry::Proxy proxy(new OsgOutlineFx);
}

namespace {

    class OutlineTechnique : public osgFX::Technique
    {
    public:
        OutlineTechnique(const OsgOutlineFx* outline)
            : Technique(),
              outline(outline){

        }

        bool validate(osg::State&) const {
            return true;
        }

        void updateColor() {
            if(material.valid()){
                const osg::Vec4& color = outline->getColor();
                material->setAmbient(osg::Material::FRONT_AND_BACK, color);
                material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
                material->setEmission(osg::Material::FRONT_AND_BACK, color);
            }
        }

    protected:

        void define_passes() {

            static const unsigned int Override_On = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
            static const unsigned int Override_Off = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE;

            {
                osg::StateSet* state = new osg::StateSet;

                osg::Stencil* stencil  = new osg::Stencil;
                stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0);
                //stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
                stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::REPLACE, osg::Stencil::REPLACE);
                state->setAttributeAndModes(stencil, Override_On);

                addPass(state);
            }

            {
                osg::StateSet* state = new osg::StateSet;

                osg::Stencil* stencil  = new osg::Stencil;
                stencil->setFunction(osg::Stencil::NOTEQUAL, 1, ~0);
                stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
                state->setAttributeAndModes(stencil, Override_On);

                /*
                osg::CullFace* cf = new osg::CullFace;
                cf->setMode(osg::CullFace::FRONT);
                state->setAttributeAndModes(cf, Override_On);
                */

                osg::PolygonMode* pm = new osg::PolygonMode;
                //pm->setMode(osg::PolygonMode::BACK, osg::PolygonMode::LINE);
                //pm->setMode(osg::PolygonMode::FRONT, osg::PolygonMode::LINE);
                pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
                state->setAttributeAndModes(pm, Override_On);

                osg::LineWidth* lw = new osg::LineWidth;
                lw->setWidth(outline->getWidth());
                state->setAttributeAndModes(lw, Override_On);

                //state->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
                
                material = new osg::Material;
                material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
                updateColor();
                state->setAttributeAndModes(material.get(), Override_On);

                state->setMode(GL_BLEND, Override_Off);
                //state->setMode(GL_BLEND, Override_On);
                state->setMode(GL_DEPTH_TEST, Override_Off);
                state->setTextureMode(0, GL_TEXTURE_1D, Override_Off);
                state->setTextureMode(0, GL_TEXTURE_2D, Override_Off);
                state->setTextureMode(0, GL_TEXTURE_3D, Override_Off);

                addPass(state);
            }
        }

    private:
        osg::ref_ptr<const OsgOutlineFx> outline;
        osg::ref_ptr<osg::Material> material;
    };


    class EnableStencilCallback : public osg::NodeCallback
    {
    public:
        EnableStencilCallback() { }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {

            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
            if (cv) {
                osgUtil::RenderStage* render = cv->getRenderStage();
                unsigned int mask = render->getClearMask();
                if ((mask & GL_STENCIL_BUFFER_BIT) == 0) {
                    render->setClearMask(mask | GL_STENCIL_BUFFER_BIT);
                    render->setClearStencil(0);
                }
            }
            traverse(node, nv);
        }
    };
}


OsgOutlineFx::OsgOutlineFx()
    : Effect()
{
    width = 3.0f;
    color.set(1.0f, 1.0f, 1.0f, 1.0f);
    //addCullCallback(new EnableStencilCallback);
    setCullCallback(new EnableStencilCallback);
}


OsgOutlineFx::OsgOutlineFx(double width, const osg::Vec4& color)
    : Effect(),
      width(width),
      color(color)
{
    //addCullCallback(new EnableStencilCallback);
    setCullCallback(new EnableStencilCallback);
}


OsgOutlineFx::OsgOutlineFx(const OsgOutlineFx& org, const osg::CopyOp& op)
    : Effect(org, op)
{
    width = org.width;
    color = org.color;
}


OsgOutlineFx::~OsgOutlineFx()
{

}


void OsgOutlineFx::setColor(const osg::Vec4& color)
{
    this->color = color;
    if(getNumTechniques() > 0){
        OutlineTechnique* tech = dynamic_cast<OutlineTechnique*>(getTechnique(0));
        if(tech){
            tech->updateColor();
        }
    }
}


bool OsgOutlineFx::define_techniques()
{
    addTechnique(new OutlineTechnique(this));
    return true;
}
