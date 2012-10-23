/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SEQ_H_INCLUDED
#define CNOID_UTIL_MULTI_SEQ_H_INCLUDED

#include "SeqBase.h"
#include <algorithm>
#include <boost/multi_array.hpp>

namespace cnoid {

    template <typename ElementType, typename Allocator = std::allocator<ElementType> >
    class MultiSeq : public MultiSeqBase
    {
    public:
        typedef boost::shared_ptr< MultiSeq<ElementType, Allocator> > Ptr;
        typedef typename boost::multi_array<ElementType, 2, Allocator> Container;
        typedef typename Container::index_range IndexRange;
        typedef typename Container::template array_view<1>::type View;
            

        MultiSeq(const char* seqType, int numParts = 1, int numFrames = 0.0, double frameRate = 100.0)
            : MultiSeqBase(seqType),
              container(boost::extents[numFrames][numParts]) {
            frameRate_ = frameRate;
            numFrames_ = numFrames;
        }

        MultiSeq(const MultiSeq<ElementType, Allocator>& org)
            : MultiSeqBase(org),
              container(org.container) {
            frameRate_ = org.frameRate_;
            numFrames_ = org.numFrames_;
        }
    
        virtual ~MultiSeq() { }

        virtual void setDimension(int newNumFrames, int newNumParts, bool clearNewElements = false) {

            const int prevNumParts = numParts();
            const int prevNumFrames = numFrames();

            int reservedNumFrames = container.size();
            if(newNumFrames > reservedNumFrames){
                container.resize(boost::extents[newNumFrames][newNumParts]);
            } else {
                container.resize(boost::extents[reservedNumFrames][newNumParts]);
            }

            numFrames_ = newNumFrames;

            if(clearNewElements){
                int i = (prevNumFrames == 0) ? 0 : prevNumParts;
                while(i < newNumParts){
                    View frames = part(i++);
                    std::fill(frames.begin(), frames.end(), defaultValue());
                }
                if(prevNumFrames > 0 && (numFrames_ > prevNumFrames)){
                    for(int i=0; i < prevNumParts; ++i){
                        View frames = part(i);
                        const ElementType& last = frames[prevNumFrames - 1];
                        std::fill(frames.begin() + prevNumFrames, frames.end(), last);
                    }
                }
            }
        }

        virtual double getFrameRate() const {
            return frameRate_;
        }

        inline double frameRate() const {
            return frameRate_;
        }

        virtual void setFrameRate(double frameRate) {
            frameRate_ = frameRate;
        }

        virtual void setNumParts(int newNumParts, bool clearNewElements = false) {
            setDimension(numFrames(), newNumParts, clearNewElements);
        }

        virtual int getNumFrames() const {
            return numFrames_;
        }

        inline int numFrames() const {
            return numFrames_;
        }

        virtual void setNumFrames(int newNumFrames, bool clearNewElements = false) {
            setDimension(newNumFrames, numParts(), clearNewElements);
        }

        virtual int getNumParts() const {
            return container.shape()[1];
        }

        inline int numParts() const {
            return container.shape()[1];
        }

        /**
           @if jp
           シーケンスの時間長を返す。
           @note この時間 *未満* の時間については有効なデータが存在する。
           この時間のデータは存在しないので、そのようなアクセスしてはいけない。
           @endif
        */
        inline double timeLength() const {
            return numFrames_ / frameRate();
        }

        inline int frameOfTime(double time) const {
            return (int)(time * frameRate_);
        }
            
        inline double timeOfFrame(int frame) const {
            return (frame / frameRate_);
        }
            
        inline const ElementType& at(int frame, int part) const {
            return container[frame][part];
        }

        inline ElementType& at(int frame, int part) {
            return container[frame][part];
        }
                
        inline View part(int index) {
            return container[boost::indices[IndexRange(0, numFrames())][index]];
        }

        inline View frame(int index) {
            return container[index][boost::indices[IndexRange(0, numParts())]];
        }

        /*
          Container::reference operator[](index i) {
          return container[i];
          }
          Container::const_reference operator[](index i) const {
          return container[i];
          }
        */

        inline View appendFrame() {
            const size_t n = numFrames_;
            if(n >= container.size()){
                setNumFrames(n * 2);
            }
            numFrames_ = n + 1;
            return frame(n);
        }
                
        template <typename ArrayType> inline void appendFrame(const ArrayType& v) {
            View frame = appendFrame();
            const int m = numParts();
            for(int i=0; i < m; ++i){
                frame[i] = v[i];
            }
        }

        virtual bool read(const YamlMapping& archive) { return MultiSeqBase::read(archive); }
        virtual bool write(YamlWriter& writer) { return MultiSeqBase::write(writer); }

    protected:

        Container container;
        int numFrames_;
        double frameRate_;

        virtual ElementType defaultValue() const { return ElementType(); }
    };
}

#endif
