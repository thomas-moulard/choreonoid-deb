/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LEGGED_BODY_H_INCLUDED
#define CNOID_BODY_LEGGED_BODY_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT LeggedBody : public Body
    {
      public:

        LeggedBody();
        virtual ~LeggedBody();

        virtual BodyPtr duplicate() const;

        inline int numFeet() const { return footInfos.size(); }

        struct FootInfo {
            Link* link;
            Vector3 homeCop;
            Vector3 soleCenter;
            YamlMappingPtr info;
        };

        inline Link* footLink(int index) const { return footInfos[index].link; }
        inline const FootInfo& footInfo(int index) const { return footInfos[index]; }
        
        bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor = false);
        bool setStance(double width, Link* baseLink);

        Vector3 centerOfSole(int footIndex) const;
        Vector3 centerOfSoles() const;

        Vector3 homeCopOfSole(int footIndex) const;
        Vector3 homeCopOfSoles() const;
        
        static bool checkBodyInfoAsLeggedBody(const YamlMappingPtr info);

      protected:

        LeggedBody(const LeggedBody& org);

        virtual void doResetInfo(const YamlMapping& info);
        
      private:

        std::vector<FootInfo> footInfos;

    };

    typedef boost::intrusive_ptr<LeggedBody> LeggedBodyPtr;
}

#endif
