
#ifndef CNOID_CHOREOGRAPHY_PRONUN_SYMBOL_H_INCLUDED
#define CNOID_CHOREOGRAPHY_PRONUN_SYMBOL_H_INCLUDED

#include "Pose.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT PronunSymbol : public PoseUnit
    {
      public:
        PronunSymbol();
        PronunSymbol(const PronunSymbol& org);
        virtual ~PronunSymbol();

        virtual PoseUnit* duplicate();
        virtual bool restore(const YamlMapping& archive, const BodyPtr body);
        virtual void store(YamlMapping& archive, const BodyPtr body) const;

        inline PoseUnitPtr actualPoseUnit(){
            return actualPoseUnit_;
        }
                
      private:
        PoseUnitPtr actualPoseUnit_;
    };

    typedef boost::intrusive_ptr<PronunSymbol> PronunSymbolPtr;
}

#endif
