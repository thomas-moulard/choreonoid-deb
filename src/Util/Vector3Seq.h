/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VECTOR3_SEQ_H_INCLUDED
#define CNOID_UTIL_VECTOR3_SEQ_H_INCLUDED

#include "Seq.h"
#include "EigenUtil.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT Vector3Seq : public Seq<Vector3>
    {
        typedef Seq<Vector3> BaseSeqType;
            
    public:
        Vector3Seq(int nFrames = 0, double frameRate = 100.0);
        Vector3Seq(const Vector3Seq& org);
        virtual ~Vector3Seq();

        virtual bool write(YamlWriter& writer);
        virtual bool read(const YamlMapping& archive);

        virtual bool loadPlainFormat(const std::string& filename);
        virtual bool saveAsPlainFormat(const std::string& filename);

    protected:
        virtual Vector3 defaultValue() const { return Vector3::Zero(); }
    };

    typedef boost::shared_ptr<Vector3Seq> Vector3SeqPtr;
}

#endif
