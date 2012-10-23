/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_MULTI_SEQ_ITEM_H_INCLUDED
#define CNOID_GUIBASE_MULTI_SEQ_ITEM_H_INCLUDED

#include "Item.h"
#include <cnoid/MultiSeq>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT MultiSeqItemBase : public Item
    {
    public:
        MultiSeqItemBase();
        MultiSeqItemBase(const MultiSeqItemBase& org);
        virtual ~MultiSeqItemBase();

        virtual MultiSeqBasePtr seqBase() = 0;
        virtual void notifyUpdate();
        
    protected:
        void doPutPropertiesSub(PutPropertyFunction& putProperty, MultiSeqBase* seq);
        bool storeSub(Archive& archive);
        bool restoreSub(const Archive& archive);
    };
    
    
    template <typename MultiSeqType> class MultiSeqItem : public MultiSeqItemBase
    {
    public:

        static void initialize(ExtensionManager* ext) { }
        
        typedef boost::intrusive_ptr< MultiSeqItem<MultiSeqType> > Ptr;
        
        MultiSeqItem() : seq_(new MultiSeqType()) { }
        
        MultiSeqItem(typename MultiSeqType::Ptr seq) : seq_(seq) { }

        virtual MultiSeqBasePtr seqBase() { return seq_; }
                
        inline boost::shared_ptr<MultiSeqType> seq() { return seq_; }
        //void resetSeq(boost::shared_ptr<MultiSeqType> seq) { seq_ = seq; }

    protected:
            
        MultiSeqItem(const MultiSeqItem<MultiSeqType>& org)
            : MultiSeqItemBase(org),
              seq_(new MultiSeqType(*org.seq_)) { }

        virtual ItemPtr doDuplicate() const {
            return new MultiSeqItem<MultiSeqType>(*this);
        }

        virtual void doPutProperties(PutPropertyFunction& putProperty) {
            doPutPropertiesSub(putProperty, seq_.get());
        }

        virtual bool store(Archive& archive) {
            return storeSub(archive);
        }
            
        virtual bool restore(const Archive& archive) {
            return restoreSub(archive);
        }

    private:
            
        virtual ~MultiSeqItem() { }

        boost::shared_ptr<MultiSeqType> seq_;
    };
}

#endif
