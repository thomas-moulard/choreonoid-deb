/**
   \author Shin'ichiro Nakaoka
*/

#include "SceneGL.h"
#include <osg/Geode>

using namespace osg;
using namespace cnoid;

namespace cnoid {

    class SceneGLDrawable : public osg::Drawable
    {
    public:
        SceneGLDrawable()
        {
            owner = 0;
        }

        SceneGLDrawable(const SceneGLDrawable& org, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY)
            : osg::Drawable(org, copyop)
        {
            owner = 0;
        }

        META_Object(cnoidBase, SceneGLDrawable)

        virtual ~SceneGLDrawable() { }

        virtual void drawImplementation(osg::RenderInfo& ri) const {
            owner->drawImplementation(ri);
        }
        
        virtual osg::BoundingBox computeBound() const {
            return owner->computeGLBound();
        }

        virtual void accept(osg::PrimitiveFunctor& functor) const  {
            owner->accept(functor);
        }

        SceneGL* owner;
    };
}


SceneGL::SceneGL()
{
    initializeDrawable(new SceneGLDrawable());
}


SceneGL::SceneGL(const SceneGL& org, const osg::CopyOp& copyop)
    : SceneObject(org, copyop)
{
    initializeDrawable(new SceneGLDrawable(*org.drawable, copyop));
}


void SceneGL::initializeDrawable(SceneGLDrawable* drawable)
{
    this->drawable = drawable;
    drawable->owner = this;
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(drawable);
    addChild(geode);
}


SceneGL::~SceneGL()
{

}


void SceneGL::setUseDisplayList(bool on)
{
    drawable->setUseDisplayList(on);
}


void SceneGL::dirtyDisplayList()
{
    drawable->dirtyDisplayList();
    drawable->dirtyBound();
    requestRedraw();
}


osg::BoundingBox SceneGL::computeGLBound() const
{
    return osg::BoundingBox();
}


/**
   Reimplement this function and specify vertices using the functor object
   when you need the intersection test on the objects
   rendered by custom drawImplementation().
*/
void SceneGL::accept(osg::PrimitiveFunctor& functor) const
{
    drawable->osg::Drawable::accept(functor);
}
