/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_PATH_H_INCLUDED
#define CNOID_BODY_LINK_PATH_H_INCLUDED

#include "LinkTraverse.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT LinkPath : public LinkTraverse
    {
      public:
        
        LinkPath();
        LinkPath(Link* base, Link* end);
        LinkPath(Link* end);

        bool find(Link* base, Link* end);
        void find(Link* end);

        inline Link* baseLink() const {
            return links.front();
        }
        
        inline Link* endLink() const {
            return links.back();
        }

      private:

        virtual void find(Link* root, bool doUpward, bool doDownward);

        bool findPathSub(Link* link, Link* prev, Link* end, bool isForwardDirection);
        void findPathFromRootSub(Link* link);
    };

};

#endif
