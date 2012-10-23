/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_COLDET_LINK_PAIR_H_INCLUDED
#define CNOID_BODY_COLDET_LINK_PAIR_H_INCLUDED

#include "Link.h"
#include <cnoid/ColdetModelPair>
#include "exportdecl.h"

namespace cnoid {
    
    class Link;
    
    class CNOID_EXPORT ColdetLinkPair : public ColdetModelPair
    {
      public:
        ColdetLinkPair(Link* link1, Link* link2);
        
        ColdetLinkPair(const ColdetLinkPair& org);
        
        virtual ~ColdetLinkPair();
        
        void updatePositions();
        
        Link* link(int index);
        
      protected:
        Link* links[2];
        
      private:
    };
    
    typedef boost::intrusive_ptr<ColdetLinkPair> ColdetLinkPairPtr;
}

#endif
