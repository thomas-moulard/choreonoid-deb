
#ifndef CNOID_GUIBASE_OSG_OUTLINE_FX_H_INCLUDED
#define CNOID_GUIBASE_OSG_OUTLINE_FX_H_INCLUDED

#include <osgFX/Effect>

#include "exportdecl.h"

namespace cnoid
{
    class CNOID_EXPORT OsgOutlineFx : public osgFX::Effect
    {
    public:
        OsgOutlineFx();
        OsgOutlineFx(double width, const osg::Vec4& color = osg::Vec4(1.0, 0.0, 0.0, 1.0));
        OsgOutlineFx(const OsgOutlineFx& copy, const osg::CopyOp& op = osg::CopyOp::SHALLOW_COPY);

        META_Effect(cnoid, OsgOutlineFx, "OsgOutlineFx", "Stencil buffer based object outlining.", "");

        inline float getWidth() const {
            return width;
        }

        inline const osg::Vec4& getColor() const {
            return color;
        }

        void setColor(const osg::Vec4& color);
        
    protected:
        virtual ~OsgOutlineFx();

        bool define_techniques();

    private:
        float width;
        osg::Vec4 color;
    };
};

#endif
