/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_VALUE_SEQ_H_INCLUDED
#define CNOID_UTIL_MULTI_VALUE_SEQ_H_INCLUDED

#include "MultiSeq.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT MultiValueSeq : public MultiSeq<double>
    {
        typedef MultiSeq<double> BaseSeqType;
            
    public:

        typedef boost::shared_ptr<MultiValueSeq> Ptr;
            
        MultiValueSeq(int numParts = 1, int numFrames = 0, double frameRate = 100.0);
        MultiValueSeq(const MultiValueSeq& org);
        virtual ~MultiValueSeq();
        
        virtual bool write(YamlWriter& writer);
        virtual bool read(const YamlMapping& archive);

        virtual bool loadPlainFormat(const std::string& filename);
        virtual bool saveAsPlainFormat(const std::string& filename);
    };

    typedef MultiValueSeq::Ptr MultiValueSeqPtr;        
}

#endif
