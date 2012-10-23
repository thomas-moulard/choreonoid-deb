/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_AFFINE3_SEQ_H_INCLUDED
#define CNOID_UTIL_MULTI_AFFINE3_SEQ_H_INCLUDED

#include "MultiSeq.h"
#include "EigenTypes.h"
#include "exportdecl.h"

namespace cnoid {

    class YamlWriter;
    class YamlMapping;

    class CNOID_EXPORT MultiAffine3Seq : public MultiSeq<Affine3, Eigen::aligned_allocator<Affine3> >
	{
        typedef MultiSeq<Affine3, Eigen::aligned_allocator<Affine3> > BaseSeqType;

    public:
        typedef boost::shared_ptr<MultiAffine3Seq> Ptr;
            
        MultiAffine3Seq(int numParts = 1, int numFrames = 0, double frameRate = DEFAULT_FRAME_RATE);
        MultiAffine3Seq(const MultiAffine3Seq& org);
        virtual ~MultiAffine3Seq();
        
        virtual bool write(YamlWriter& writer);
        virtual bool read(const YamlMapping& archive);

        virtual bool loadPlainFormat(const std::string& filename);
        bool saveTopPartAsPlainFormat(const std::string& filename);

    protected:
        virtual Affine3 defaultValue() const { return Affine3::Identity(); }

    };

    typedef MultiAffine3Seq::Ptr MultiAffine3SeqPtr;        
}

#endif
