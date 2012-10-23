/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SEQ_H_INCLUDED
#define CNOID_UTIL_SEQ_H_INCLUDED

#include "SeqBase.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

    template <typename ElementType> class Seq : public SeqBase
    {
      public:
        typedef boost::shared_ptr< Seq<ElementType> > Ptr;

      Seq(const char* seqType, int nFrames = 0.0, double frameRate = 100.0)
          : SeqBase(seqType),
            container(nFrames) {
                frameRate_ = frameRate;
      }

      Seq(const Seq<ElementType>& org)
          : SeqBase(org),
            container(org.container) {
            frameRate_ = org.frameRate_;
      }
    
        virtual ~Seq() { }

        virtual double getFrameRate() const {
            return frameRate_;
        }

        inline double frameRate() const {
            return frameRate_;
        }

        virtual void setFrameRate(double frameRate) {
            frameRate_ = frameRate;
        }

        virtual int getNumFrames() const {
            return container.size();
        }

        inline int numFrames() const {
            return container.size();
        }

        virtual void setNumFrames(int n, bool clearNewElements = false) {
            if(clearNewElements){
                container.resize(n, defaultValue());
            } else {
                container.resize(n);
            }
        }

        inline bool empty() const {
            return container.empty();
        }

        inline int frameOfTime(double time) const {
            return (int)(time * frameRate_);
        }

        inline double timeOfFrame(int frame) const {
            return (frame / frameRate_);
        }

        inline ElementType& operator[](int frameIndex) {
            return container[frameIndex];
        }

        inline const ElementType& operator[](int frameIndex) const {
            return container[frameIndex];
        }

        inline ElementType& at(int frameIndex) {
            return container[frameIndex];
        }

        inline const ElementType& at(int frameIndex) const {
            return container[frameIndex];
        }
            
        virtual bool read(const YamlMapping& archive) { return SeqBase::read(archive); }
        virtual bool write(YamlWriter& writer) { return SeqBase::write(writer); }

      protected:

        std::vector<ElementType> container;
        double frameRate_;

        virtual ElementType defaultValue() const { return ElementType(); }
    };
}

#endif
