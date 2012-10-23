/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SeqBase.h"
#include "YamlNodes.h"
#include "YamlWriter.h"

using namespace std;
using namespace cnoid;


SeqBase::SeqBase(const char* seqType)
    : seqType_(seqType)
{

}


SeqBase::SeqBase(const SeqBase& org)
    : seqType_(org.seqType_),
      purpose_(org.purpose_)
{

}


SeqBase::~SeqBase()
{

}


bool SeqBase::read(const YamlMapping& archive)
{
    setPurpose(archive["purpose"].toString());
    setFrameRate(archive["frameRate"].toDouble());
    return true;
}


bool SeqBase::write(YamlWriter& writer)
{
    if(!seqType().empty()){
        writer.putKeyValue("type", seqType());
        writer.putKeyValue("purpose", purpose());
        writer.putKeyValue("frameRate", getFrameRate());
        writer.putKeyValue("numFrames", getNumFrames());
        return true;
    }
    return true;
}


bool MultiSeqBase::read(const YamlMapping& archive)
{
    setPurpose(archive["purpose"].toString());
    setFrameRate(archive["frameRate"].toDouble());
    return true;
}


bool MultiSeqBase::write(YamlWriter& writer)
{
    if(!seqType().empty()){
        writer.putKeyValue("type", seqType());
        writer.putKeyValue("purpose", purpose());
        writer.putKeyValue("frameRate", getFrameRate());
        writer.putKeyValue("numParts", getNumParts());
        writer.putKeyValue("numFrames", getNumFrames());
        return true;
    }
    return true;
}
