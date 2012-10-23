/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SCENE_GL_H_INCLUDED
#define CNOID_GUIBASE_SCENE_GL_H_INCLUDED

#include <cnoid/SceneObject>
#include <osg/Drawable>
#include "exportdecl.h"

namespace cnoid {

    class SceneGLDrawable;

    /**
       This class is used for drawing a 3D object with raw OpenGL commands.
    */
    class CNOID_EXPORT SceneGL : public SceneObject
    {
      public:
        SceneGL();
        SceneGL(const SceneGL& org, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);
        virtual ~SceneGL();

      protected:
        void setUseDisplayList(bool on);
        void dirtyDisplayList();
        virtual void drawImplementation(osg::RenderInfo& ri) const = 0;
        virtual osg::BoundingBox computeGLBound() const;
        virtual void accept(osg::PrimitiveFunctor& functor) const;

      private:
        void initializeDrawable(SceneGLDrawable* drawable);
        
        osg::ref_ptr<SceneGLDrawable> drawable;

        friend class SceneGLDrawable;
    };

    typedef osg::ref_ptr<SceneGL> SceneGLPtr;
}

#endif
