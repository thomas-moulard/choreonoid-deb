/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_VECTOR3_SEQ_ITEM_H_INCLUDED
#define CNOID_GUIBASE_VECTOR3_SEQ_ITEM_H_INCLUDED

#include "Item.h"
#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT Vector3SeqItem : public Item
    {
    public:
        static void initialize(ExtensionManager* ext);
            
        Vector3SeqItem();
        Vector3SeqItem(Vector3SeqPtr seq);

        inline const Vector3SeqPtr& seq() { return seq_; }

        bool loadPlainFormat(const std::string& filename);
        bool saveAsPlainFormat(const std::string& filename);

        virtual void notifyUpdate();

    protected:
            
        Vector3SeqItem(const Vector3SeqItem& org);
            
        virtual ItemPtr doDuplicate() const;
        virtual void doPutProperties(PutPropertyFunction& putProperty);
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);

    private:
            
        virtual ~Vector3SeqItem();
            
        Vector3SeqPtr seq_;
    };

    typedef boost::intrusive_ptr<Vector3SeqItem> Vector3SeqItemPtr;
}

#endif
