/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_OSG_COLLISION_H_INCLUDED
#define CNOID_BODYPLUGIN_OSG_COLLISION_H_INCLUDED

#include <vector>
#include <osg/Drawable>
#include <cnoid/ColdetLinkPair>
#include "exportdecl.h"

namespace cnoid {

    class OsgCollision : public osg::Drawable
    {
    public:
        OsgCollision();
        OsgCollision(const OsgCollision& org, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);
            
        META_Object(cnoidBody, OsgCollision)
            
        virtual ~OsgCollision() { }

        inline void setColdetPairs(const std::vector<ColdetLinkPairPtr>& pairs) {
            ppairs = &pairs;
        };

    private:
            
        const std::vector<ColdetLinkPairPtr>* ppairs;

        virtual void drawImplementation(osg::RenderInfo& ri) const;
        virtual osg::BoundingBox computeBound() const;
    };
}

#endif
