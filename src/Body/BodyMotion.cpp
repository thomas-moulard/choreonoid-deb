/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "BodyMotion.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/YamlReader>
#include <cnoid/YamlWriter>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    //bool TRACE_FUNCTIONS = false;
}


BodyMotion::BodyMotion()
    : MultiSeqBase("BodyMotion"),
      jointPosSeq_(new MultiValueSeq()),
      linkPosSeq_(new MultiAffine3Seq())
{
    jointPosSeq_->setPurpose("JointPosition");
    linkPosSeq_->setPurpose("LinkPosition");
}


BodyMotion::BodyMotion(const BodyMotion& org)
    : MultiSeqBase(org),
      jointPosSeq_(new MultiValueSeq(*org.jointPosSeq_)),
      linkPosSeq_(new MultiAffine3Seq(*org.linkPosSeq_))
{
    if(org.relativeZmpSeq_){
        relativeZmpSeq_.reset(new Vector3Seq(*org.relativeZmpSeq_));
    }
}


const Vector3SeqPtr& BodyMotion::relativeZmpSeq()
{
    if(!relativeZmpSeq_){
        relativeZmpSeq_.reset(new Vector3Seq());
        relativeZmpSeq_->setPurpose("RelativeZmp");
        relativeZmpSeq_->setNumFrames(numFrames());
        relativeZmpSeq_->setFrameRate(frameRate());
    }
    return relativeZmpSeq_;
}


/*
  This function sets the number of joints
*/
void BodyMotion::setNumParts(int numParts, bool clearNewElements)
{
    jointPosSeq_->setNumParts(numParts, clearNewElements);
}


/**
   This function returns the number of joints
*/
int BodyMotion::getNumParts() const
{
    return jointPosSeq_->numParts();
}
        

double BodyMotion::getFrameRate() const
{
    return frameRate();
}


void BodyMotion::setFrameRate(double frameRate)
{
    jointPosSeq_->setFrameRate(frameRate);
    linkPosSeq_->setFrameRate(frameRate);
    if(relativeZmpSeq_){
        relativeZmpSeq_->setFrameRate(frameRate);
    }
}


int BodyMotion::getNumFrames() const
{
    return numFrames();
}


void BodyMotion::setNumFrames(int n, bool clearNewArea)
{
    jointPosSeq_->setNumFrames(n, clearNewArea);
    linkPosSeq_->setNumFrames(n, clearNewArea);
    if(relativeZmpSeq_){
        relativeZmpSeq_->setNumFrames(n, clearNewArea);
    }
}


void BodyMotion::setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea)
{
    jointPosSeq_->setDimension(numFrames, numJoints, clearNewArea);
    linkPosSeq_->setDimension(numFrames, numLinks, clearNewArea);
    if(relativeZmpSeq_){
        relativeZmpSeq_->setNumFrames(numFrames, clearNewArea);
    }
}


void BodyMotion::setDimension(int numFrames, int numJoints, bool clearNewArea)
{
    jointPosSeq_->setDimension(numFrames, numJoints, clearNewArea);
    linkPosSeq_->setNumFrames(numFrames, clearNewArea);
    if(relativeZmpSeq_){
        relativeZmpSeq_->setNumFrames(numFrames, clearNewArea);
    }
}    


bool BodyMotion::loadStandardYamlFormat(const std::string& filename)
{
    bool result = false;
    
    YamlReader reader;
    reader.expectRegularMultiSequence();
    
    try {
        reader.load(filename);
        const YamlMapping& archive = *reader.document()->toMapping();

        result = read(archive);

    } catch(const YamlNode::Exception& ex){
        setIoErrorMessage(ex.message());
        result = false;
    }

    return result;
}


bool BodyMotion::read(const YamlMapping& archive)
{
    setDimension(0, 1, 1);

    bool result = true;
    bool loaded = false;
    bool zmpLoaded = false;

    try {
        if(archive["type"].toString() != "BodyMotion"){
            result = false;

        } else {
            const YamlSequence& components = *archive["components"].toSequence();
        
            for(int i=0; i < components.size(); ++i){
                const YamlMapping& component = *components[i].toMapping();
                const string& type = component["type"];
                const string& purpose = component["purpose"];
                if(type == "MultiValueSeq" && purpose == "JointPosition"){
                    result &= jointPosSeq_->read(component);
                    if(result){
                        loaded = true;
                    }
                } else if((type == "MultiAffine3Seq" || type == "MultiSe3Seq") && purpose == "LinkPosition"){
                    result &= linkPosSeq_->read(component);
                    if(result){
                        loaded = true;
                    }
                } else if(type == "Vector3Seq" && purpose == "RelativeZmp"){
                    result = relativeZmpSeq()->read(component);
                    if(result){
                        zmpLoaded = true;
                    }
                }
                if(!result){
                    break;
                }
            }
        }
    } catch(const YamlNode::Exception& ex){
        setIoErrorMessage(ex.message());
        result = false;
    }

    if(!result){
        setDimension(0, 1, 1);
    }
    if(!zmpLoaded){
        relativeZmpSeq_.reset();
    }
    
    return (result && loaded);
}


bool BodyMotion::write(YamlWriter& writer)
{
    writer.startSequence();

    jointPosSeq()->write(writer);
    linkPosSeq()->write(writer);
    if(hasRelativeZmpSeq()){
        relativeZmpSeq()->write(writer);
    }

    writer.endSequence();

    return true;
}


bool BodyMotion::saveAsStandardYamlFormat(const std::string& filename)
{
    YamlWriter writer(filename);
    writer.setDoubleFormat("%.9g");

    writer.putComment("Body motion data set format version 1.0 defined by cnoid-Robotics\n");

    writer.startMapping();

    writer.putKeyValue("type", "BodyMotion");
    writer.putKey("components");

    bool result = write(writer);

    writer.endMapping();

    return result;
}


namespace cnoid {

    BodyMotion::Frame& operator<<(const BodyMotion::Frame& frame, const Body& body)
    {
        BodyMotion& motion = const_cast<BodyMotion::Frame&>(frame).motion();
        int numJoints = std::min(body.numJoints(), motion.numJoints());
        MultiValueSeq::View q = motion.jointPosSeq()->frame(frame.frame());
        for(int i=0; i < numJoints; ++i){
            q[i] = body.joint(i)->q;
        }
        int numLinks =  std::min(body.numLinks(), motion.numLinks());
        MultiAffine3Seq::View T = motion.linkPosSeq()->frame(frame.frame());
        for(int i=0; i < numLinks; ++i){
            Link* link = body.link(i);
            T[i].translation() = link->p;
            T[i].linear() = link->R;
        }
        return const_cast<BodyMotion::Frame&>(frame);
    }

    BodyMotion::Frame& operator>>(const BodyMotion::Frame& frame, const Body& body)
    {
        const BodyMotion& motion = frame.motion();
        int numJoints = std::min(body.numJoints(), motion.numJoints());
        const MultiValueSeq::View q = motion.jointPosSeq()->frame(frame.frame());
        for(int i=0; i < numJoints; ++i){
            body.joint(i)->q = q[i];
        }
        int numLinks =  std::min(body.numLinks(), motion.numLinks());
        const MultiAffine3Seq::View T = motion.linkPosSeq()->frame(frame.frame());
        for(int i=0; i < numLinks; ++i){
            Link* link = body.link(i);
            link->p = T[i].translation();
            link->R = T[i].linear();
        }
        return const_cast<BodyMotion::Frame&>(frame);
    }
}
