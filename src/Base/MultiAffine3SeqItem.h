/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_MULTI_AFFINE3_SEQ_ITEM_H_INCLUDED
#define CNOID_GUIBASE_MULTI_AFFINE3_SEQ_ITEM_H_INCLUDED

#include "MultiSeqItem.h"
#include <cnoid/MultiAffine3Seq>

namespace cnoid {
    typedef MultiSeqItem<MultiAffine3Seq> MultiAffine3SeqItem;
    typedef MultiAffine3SeqItem::Ptr MultiAffine3SeqItemPtr;

	template<> void MultiSeqItem<MultiAffine3Seq>::initialize(ExtensionManager* ext);
}

#endif
