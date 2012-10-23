/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiSeqItem.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace boost;
using namespace cnoid;


MultiSeqItemBase::MultiSeqItemBase()
{

}


MultiSeqItemBase::MultiSeqItemBase(const MultiSeqItemBase& org)
    : Item(org)
{

}


MultiSeqItemBase::~MultiSeqItemBase()
{

}


void MultiSeqItemBase::notifyUpdate()
{
    setInconsistencyWithLastAccessedFile();
    Item::notifyUpdate();
}


static bool setPropertyNumFrames(MultiSeqBase* seq, int numFrames)
{
    if(numFrames >= 0){
        seq->setNumFrames(numFrames);
        return true;
    }
    return false;
}


static bool setPropertyTimeLength(MultiSeqBase* seq, double timeLength)
{
    if(timeLength >= 0){
        seq->setTimeLength(timeLength);
        return true;
    }
    return false;
}


void MultiSeqItemBase::doPutPropertiesSub(PutPropertyFunction& putProperty, MultiSeqBase* seq)
{
    putProperty(_("Frame rate"), seq->getFrameRate());
    putProperty(_("Num parts"), seq->getNumParts());
    putProperty(_("Num frames"), seq->getNumFrames(), bind(setPropertyNumFrames, seq, _1));
    putProperty(_("Time length"), seq->getTimeLength(), bind(setPropertyTimeLength, seq, _1));
    putProperty(_("Time step"), seq->getTimeStep());
}


bool MultiSeqItemBase::storeSub(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", lastAccessedFileName());
        archive.write("format", lastAccessedFileFormatId());
        return true;
    }
    return false;
}


bool MultiSeqItemBase::restoreSub(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        if(load(filename, formatId)){
            return true;
        }
    }
    return false;
}
