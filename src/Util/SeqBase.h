/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SEQ_BASE_H_INCLUDED
#define CNOID_UTIL_SEQ_BASE_H_INCLUDED

#include <string>
#include <boost/shared_ptr.hpp>
#include "exportdecl.h"

namespace cnoid {

    class YamlMapping;
    class YamlWriter;
    
    static const double DEFAULT_FRAME_RATE = 100.0;

    class CNOID_EXPORT SeqBase
    {
    public:
        
        SeqBase(const char* seqType);
        SeqBase(const SeqBase& org);
        virtual ~SeqBase();

        inline const std::string& seqType() const {
            return seqType_;
        }

        virtual double getFrameRate() const = 0;
        virtual void setFrameRate(double frameRate) = 0;

        inline double getTimeStep() const {
            return 1.0 / getFrameRate();
        }
    
        inline void setTimeStep(double timeStep){
            return setFrameRate(1.0 / timeStep);
        }

        virtual int getNumFrames() const = 0;
        virtual void setNumFrames(int n, bool clearNewElements = false) = 0;

        inline void setTimeLength(double length, bool clearNewElements = false){
            return setNumFrames(static_cast<int>(length * getFrameRate()), clearNewElements);
        }

        /**
           @if jp
           シーケンスの時間長を返す。
           @note この時間 *未満* の時間については有効なデータが存在する。
           この時間のデータは存在しないので、そのようなアクセスしてはいけない。
           @endif
        */
        inline double getTimeLength() const {
            return getNumFrames() / getFrameRate();
        }
            
        inline const std::string& purpose() {
            return purpose_;
        }

        virtual void setPurpose(const std::string& purpose) {
            purpose_ = purpose;
        }

        virtual bool read(const YamlMapping& archive);
        virtual bool write(YamlWriter& writer);

        inline const std::string& ioErrorMessage() const {
            return ioErrorMessage_;
        }

    protected:
        void setIoErrorMessage(const std::string& message) {
            ioErrorMessage_ = message;
        }

    private:
        const std::string seqType_;
        std::string purpose_;
        std::string ioErrorMessage_;
    };

    typedef boost::shared_ptr<SeqBase> SeqBasePtr;


    class CNOID_EXPORT MultiSeqBase : public SeqBase
    {
    public:
        MultiSeqBase(const char* seqType)
            : SeqBase(seqType) { }
        MultiSeqBase(const SeqBase& org)
            : SeqBase(org) { }
        virtual ~MultiSeqBase() { }

        virtual void setDimension(int numFrames, int numParts, bool claerNewElements = false) = 0;
            
        virtual void setNumParts(int numParts, bool clearNewElements = false) = 0;
        virtual int getNumParts() const = 0;

        virtual bool read(const YamlMapping& archive);
        virtual bool write(YamlWriter& writer);
    };

    typedef boost::shared_ptr<MultiSeqBase> MultiSeqBasePtr;
}

#endif
