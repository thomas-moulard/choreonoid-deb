/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyMotionEngine.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/TimeSyncItemEngineManager>
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;

namespace {

    const bool TRACE_FUNCTIONS = false;    

    class BodyMotionEngine : public TimeSyncItemEngine
    {
    public:
        BodyItemPtr bodyItem;
        BodyPtr body;
        MultiValueSeqPtr qSeq;
        MultiAffine3SeqPtr positions;
        Vector3SeqPtr relativeZmpSeq;
        bool calcForwardKinematics;

        BodyMotionEngine() { }

        bool initialize(Item* item) {

            BodyMotionItem* bodyMotionItem;
            MultiValueSeqItem* multiValueSeqItem;
            MultiAffine3SeqItem* multiAffine3SeqItem;;

            if((bodyMotionItem = dynamic_cast<BodyMotionItem*>(item))){
                if(setBodyItem(item)){
                    qSeq = bodyMotionItem->jointPosSeq();
                    positions = bodyMotionItem->linkPosSeq();
                    if(bodyMotionItem->hasRelativeZmpSeqItem()){
                        relativeZmpSeq = bodyMotionItem->relativeZmpSeq();
                    }
                }

            } else if((multiValueSeqItem = dynamic_cast<MultiValueSeqItem*>(item))){
                if(setBodyItem(item)){
                    qSeq = multiValueSeqItem->seq();
                }
                
            } else if((multiAffine3SeqItem = dynamic_cast<MultiAffine3SeqItem*>(item))){
                if(setBodyItem(item)){
                    positions = multiAffine3SeqItem->seq();
                }
            }

            if(bodyItem){
                calcForwardKinematics = !(positions && positions->numParts() > 1);
                item->sigUpdated().connect(bind(&BodyMotionEngine::notifyUpdate, this));
            }
            
            return (bodyItem);
        }

        bool setBodyItem(Item* item){
            bodyItem = item->findOwnerItem<BodyItem>();
            if(bodyItem){
                body = bodyItem->body();
            }
            return (bodyItem);
        }
        
        virtual bool onTimeChanged(double time){

            bool isActive = false;
            bool fkDone = false;
            
            if(qSeq){
                isActive = setJointPositions(time);
            }
            if(positions){
                isActive |= setLinkPositions(time);
                if(positions->numParts() == 1){
                    body->calcForwardKinematics(); // FK from the root
                    fkDone = true;
                }
            }
            if(relativeZmpSeq){
                if(calcForwardKinematics){
                    bodyItem->calcForwardKinematics();
                    fkDone = true;
                }
                isActive |= setRelativeZmp(time);
            }

            bodyItem->notifyKinematicStateChange(!fkDone && calcForwardKinematics);

            return isActive;
        }

        static inline bool clamp(int& frame, int numFrames) {

            if(frame < 0){
                frame = 0;
                return false;
            } else if(frame >= numFrames){
                frame = numFrames - 1;
                return false;
            }
            return true;
        }

        bool setLinkPositions(double time){

            bool isValidTime = false;

            const int numLinks = positions->numParts();
            const int numFrames = positions->numFrames();
            if(numLinks > 0 && numFrames > 0){
                int frame = positions->frameOfTime(time);
                isValidTime = clamp(frame, numFrames);
                for(int i=0; i < numLinks; ++i){
                    Link* link = body->link(i);
                    const Affine3& position = positions->at(frame, i);
                    link->p = position.translation();
                    link->R = position.linear();
                }
            }

            return isValidTime;
        }

        bool setJointPositions(double time){

            bool isValidTime = false;

            const int numJoints = qSeq->numParts();
            int frame = qSeq->frameOfTime(time);
            const int numFrames = qSeq->numFrames();
            if(numJoints > 0 && numFrames > 0){
                isValidTime = clamp(frame, numFrames);
                MultiValueSeq::View q = qSeq->frame(frame);
                for(int i=0; i < numJoints; ++i){
                    Link* link = body->joint(i);
                    link->q = q[i];
                }
            }

            return isValidTime;
        }    

        bool setRelativeZmp(double time){

            bool isValidTime = false;
            int frame = relativeZmpSeq->frameOfTime(time);
            const int numFrames = relativeZmpSeq->numFrames();
            if(numFrames > 0){
                isValidTime = clamp(frame, numFrames);
                Link* rootLink = body->rootLink();
                const Vector3& relativeZmp = (*relativeZmpSeq)[frame];
                const Vector3 absZmp = rootLink->R * relativeZmp + rootLink->p;
                bodyItem->setZmp(absZmp);
            }
            return isValidTime;
        }    

    };

    typedef boost::intrusive_ptr<BodyMotionEngine> BodyMotionEnginePtr;
    

    TimeSyncItemEnginePtr createBodyMotionEngine(Item* sourceItem)
    {
        BodyMotionEnginePtr engine = new BodyMotionEngine();
        if(!engine->initialize(sourceItem)){
            engine.reset(NULL);
        }
        return engine;
    }
}


void cnoid::initializeBodyMotionEngine(ExtensionManager& em)
{
    em.timeSyncItemEngineManger().addEngineFactory(createBodyMotionEngine);
}
