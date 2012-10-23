/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "DynamicsSimulatorItem.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/World>
#include <cnoid/ConstraintForceSolver>
#include <algorithm>
#include <boost/bind.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

// for Windows
#undef min
#undef max

namespace {

    const bool TRACE_FUNCTIONS = false;

    class BodyUnit
    {
    public:
        int frame;
        double frameRate;
        BodyPtr body;
        MultiValueSeqPtr qseqRef;
        MultiValueSeqPtr qseqResultBuf;
        MultiValueSeqPtr qseqResult;
        MultiAffine3SeqItemPtr rootResultItem;
        MultiAffine3SeqPtr rootResultBuf;
        MultiAffine3SeqPtr rootResult;
        
        bool initialize(BodyItemPtr bodyItem, double worldFrameRate);
        bool setReferenceState(int newFrame, double frameRate, bool& putResult);
        void putResultToBuf();
        bool flushBuf();
    };
}


namespace cnoid {
  
    class DSIImpl
    {
    public:
        DynamicsSimulatorItem* self;

        World<ConstraintForceSolver> world;
        double worldFrameRate;
        double staticFriction;
        double slipFriction;
        double culling_thresh;
        double epsilon;

        vector<BodyUnit> bodyUnits;
        vector<BodyUnit*> bodyUnitsToPutResult;
        vector<BodyUnit*> bodyUnitsToNotifyUpdate;

        int currentFrame;
        int frameAtLastBufferWriting;

        DSIImpl(DynamicsSimulatorItem* self);
        DSIImpl(DynamicsSimulatorItem* self, const DSIImpl& org);

        void initialize();
            
        bool doStartSimulation();
        bool setupBodies();
        void addBody(BodyItemPtr bodyItem);
        bool doStepSimulation();
        double doFlushResults();
        void doStopSimulation();
        bool onFrictionPropertyChanged(double mu, int type);
    };
}


void cnoid::initializeDynamicsSimulatorItem(ExtensionManager& ext)
{
    ext.itemManager().registerClass<DynamicsSimulatorItem>(N_("DynamicsSimulatorItem"));
    ext.itemManager().addCreationPanel<DynamicsSimulatorItem>();
}


bool BodyUnit::initialize(BodyItemPtr bodyItem, double worldFrameRate)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::initialize()" << endl;
    }
        
    frame = 0;
    body = bodyItem->body()->duplicate();

    if(body->isStaticModel()){
        return true;
    }

    const int numJoints = body->numJoints();

    ItemList<BodyMotionItem> motionItems = bodyItem->getSubItems<BodyMotionItem>();

    BodyMotionItemPtr motionItem;
    if(motionItems.empty()){
        motionItem = new BodyMotionItem();
        motionItem->setName("sim");
        motionItem->motion()->setFrameRate(worldFrameRate);
        qseqResult = motionItem->jointPosSeq();
        qseqResult->setNumParts(numJoints);
        motionItem->motion()->setNumFrames(1);
        for(int i=0; i < numJoints; ++i){
            qseqResult->at(0, i) = body->joint(i)->q;
        }
        bodyItem->addChildItem(motionItem);
        qseqResultBuf.reset(new MultiValueSeq(numJoints, 0, worldFrameRate));
    } else {
        // prefer the first checked item
        for(size_t i=0; i < motionItems.size(); ++i){
            if(ItemTreeView::mainInstance()->isItemChecked(motionItems[i])){
                motionItem = motionItems[i];
                break;
            }
        }
        if(!motionItem){
            motionItem = motionItems[0];
        }
                
        qseqRef = motionItem->jointPosSeq();
    }
    
    frameRate = motionItem->motion()->frameRate();
    rootResultItem = motionItem->linkPosSeqItem();
    rootResult = motionItem->linkPosSeq();
    rootResultBuf.reset(new MultiAffine3Seq(1, 0, frameRate));
    rootResult->setFrameRate(frameRate);
    rootResult->setNumParts(1);
    rootResult->setNumFrames(1);
    Link* rootLink = body->rootLink();
    Affine3& initialPos = rootResult->at(0, 0);
    initialPos.translation() = rootLink->p;
    initialPos.linear() = rootLink->R;
    
    rootLink->v.setZero();
    rootLink->dv.setZero();
    rootLink->w.setZero();
    rootLink->dw.setZero();
    rootLink->vo.setZero();
    rootLink->dvo.setZero();
    
    // set the high-gain mode
    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        joint->isHighGainMode = true;
        joint->u = 0.0;
        joint->dq = 0.0;
        if(!qseqRef || qseqRef->numFrames() < 2){
            joint->ddq = 0.0;
        } else {
            //double qNext = qseqRef->atSeqFrame(i, 1);
            //double dqNext = (qNext - joint->q) * frameRate;
            //joint->ddq = dqNext * frameRate;
            joint->ddq = 0.0;
        }
    }
    
    body->clearExternalForces();
    body->calcForwardKinematics(true, true);

    return true;
}


bool BodyUnit::setReferenceState(int newFrame, double worldFrameRate, bool& putResult)
{
    putResult = false;
    
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::updateState()" << endl;
    }

    if(!qseqRef){
        putResult = true;
        frame = newFrame;
        return true;
    }

    if(newFrame >= qseqRef->numFrames()){
        return false;
    }

    if(newFrame <= frame){
        return true;
    }

    frame = newFrame;
    

    MultiValueSeq::View qRef = qseqRef->frame(frame);
    const int n = std::min(body->numJoints(), qseqRef->numParts());
    for(int i=0; i < n; ++i){
        Link* joint = body->joint(i);
        double qNext = qRef[i];
        double dqNext = (qNext - joint->q) * worldFrameRate;
        joint->ddq = (dqNext - joint->dq) * worldFrameRate;
        joint->dq = dqNext;
        joint->q = qNext;
    }

    putResult = true;

    return true;
}


void BodyUnit::putResultToBuf()
{
    const int bufFrame = rootResultBuf->numFrames();
    
    if(qseqResultBuf){
        qseqResultBuf->setNumFrames(bufFrame + 1);
        MultiValueSeq::View qResult = qseqResultBuf->frame(bufFrame);
        const int n = body->numJoints();
        for(int i=0; i < n; ++i){
            qResult[i] = body->joint(i)->q;
        }
    }

    rootResultBuf->setNumFrames(bufFrame + 1);
    Link* rootLink = body->rootLink();
    Affine3& pos = rootResultBuf->at(bufFrame, 0);
    pos.translation() = rootLink->p;
    pos.linear() = rootLink->R;
}


bool BodyUnit::flushBuf()
{
    const int frame = rootResult->numFrames();
    const int numBufFrames = rootResultBuf->numFrames();

    if(numBufFrames > 0){
    
        if(qseqResult){
            const int n = body->numJoints();
            qseqResult->setNumFrames(frame + numBufFrames);
            for(int i=0; i < numBufFrames; ++i){
                MultiValueSeq::View qResult = qseqResult->frame(frame + i);
                MultiValueSeq::View qResultBuf = qseqResultBuf->frame(i);
                for(int j=0; j < n; ++j){
                    qResult[j] = qResultBuf[j];
                }
            }
            qseqResultBuf->setNumFrames(0);
        }
        
        rootResult->setNumFrames(frame + numBufFrames);
        for(int i=0; i < numBufFrames; ++i){
            rootResult->at(frame + i, 0) = rootResultBuf->at(i, 0);
        }
        rootResultBuf->setNumFrames(0);
    }

    return (numBufFrames > 0);
}
    
    

DynamicsSimulatorItem::DynamicsSimulatorItem()
{
    impl = new DSIImpl(this);
}


DSIImpl::DSIImpl(DynamicsSimulatorItem* self)
    : self(self)
{
    initialize();
}


DynamicsSimulatorItem::DynamicsSimulatorItem(const DynamicsSimulatorItem& org)
    : SimulatorItem(org),
      impl(new DSIImpl(this, *org.impl))
{

}


DSIImpl::DSIImpl(DynamicsSimulatorItem* self, const DSIImpl& org)
    : self(self)
{
    initialize();

    staticFriction = org.staticFriction;
    slipFriction = org.slipFriction;
}


void DSIImpl::initialize()
{
    world.setRungeKuttaMethod();
    world.setGravityAcceleration(Vector3(0.0, 0.0, 9.8));
    world.enableSensors(true);

    staticFriction = 1.0;
    slipFriction = 1.0;
    culling_thresh = 0.01;
    epsilon = 0.0;
}


DynamicsSimulatorItem::~DynamicsSimulatorItem()
{
    delete impl;
}


ItemPtr DynamicsSimulatorItem::doDuplicate() const
{
    return new DynamicsSimulatorItem(*this);
}


QWidget* DynamicsSimulatorItem::settingPanel()
{
    return 0;
}


bool DynamicsSimulatorItem::doStartSimulation()
{
    return impl->doStartSimulation();
}


bool DSIImpl::doStartSimulation()
{
    worldFrameRate = TimeBar::instance()->frameRate();
    world.clearBodies();
    world.constraintForceSolver.clearCollisionCheckLinkPairs();

    bool result = setupBodies();

    if(result){
        currentFrame = 0;
        frameAtLastBufferWriting = 0;
        world.setTimeStep(1.0 / worldFrameRate);
        world.setCurrentTime(0.0);
        world.initialize();
    }
    
    return result;
}


bool DSIImpl::setupBodies()
{
    if(TRACE_FUNCTIONS){
        cout << "DSIImpl::setupBodies()" << endl;
    }
    
    bodyUnits.clear();
    
    WorldItemPtr worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        ItemList<BodyItem> bodyItems = worldItem->getBodyItems();
        for(size_t i=0; i < bodyItems.size(); ++i){
            BodyUnit unit;
            if(unit.initialize(bodyItems[i], worldFrameRate)){
                bodyUnits.push_back(unit);
                world.addBody(bodyUnits[i].body);
            }
        }
    }

    // set all the combinations of collision check link pairs
    for(size_t b1 = 0; b1 < bodyUnits.size(); ++b1){
        BodyPtr& body1 = bodyUnits[b1].body;
        for(size_t b2 = b1 + 1; b2 < bodyUnits.size(); ++b2){
            BodyPtr& body2 = bodyUnits[b2].body;
            for(int l1 = 0; l1 < body1->numLinks(); ++l1){
                Link* link1 = body1->link(l1);
                for(int l2 = 0; l2 < body2->numLinks(); ++l2){
                    Link* link2 = body2->link(l2);
                    if(link1 != link2){
                        world.constraintForceSolver.addCollisionCheckLinkPair
                            (b1, link1, b2, link2, staticFriction, slipFriction, culling_thresh, epsilon);
                    }
                }
            }
        }
    }

    // remove static-model bodies
    vector<BodyUnit>::iterator p = bodyUnits.begin();
    while(p != bodyUnits.end()){
        if(p->body->isStaticModel()){
            p = bodyUnits.erase(p);
        } else {
            ++p;
        }
    }
        
    return !bodyUnits.empty();
}


bool DynamicsSimulatorItem::doStepSimulation()
{
    return impl->doStepSimulation();
}


bool DSIImpl::doStepSimulation()
{
    currentFrame++;

    world.constraintForceSolver.clearExternalForces();

    bool doContinue = false;
    bodyUnitsToPutResult.clear();
    
    for(size_t i=0; i < bodyUnits.size(); ++i){
        BodyUnit& unit = bodyUnits[i];
        int frame = static_cast<int>(currentFrame * unit.frameRate / worldFrameRate);
        bool putResult;
        if(unit.setReferenceState(frame, worldFrameRate, putResult)){
            if(putResult){
                bodyUnitsToPutResult.push_back(&unit);
            }
            doContinue = true;
        }
    }

    if(doContinue){

        static int counter = 0;

        world.calcNextState();

        self->lockResults();
        
        for(size_t i=0; i < bodyUnitsToPutResult.size(); ++i){
            bodyUnitsToPutResult[i]->putResultToBuf();
        }
        frameAtLastBufferWriting = currentFrame;

        if(++counter == 10){
          self->requestToFlushResults();
          counter = 0;
        }

        self->unlockResults();
    }

    return doContinue;
}


double DynamicsSimulatorItem::doFlushResults()
{
    return impl->doFlushResults();
}


double DSIImpl::doFlushResults()
{
    bodyUnitsToNotifyUpdate.clear();

    self->lockResults();
    
    for(size_t i=0; i < bodyUnits.size(); ++i){
        if(bodyUnits[i].flushBuf()){
            bodyUnitsToNotifyUpdate.push_back(&bodyUnits[i]);
        }
    }

    int frame = frameAtLastBufferWriting;
    
    self->unlockResults();

    for(size_t i=0; i < bodyUnitsToNotifyUpdate.size(); ++i){
        bodyUnitsToNotifyUpdate[i]->rootResultItem->notifyUpdate();
    }

    return (frame / worldFrameRate);
}


double DynamicsSimulatorItem::doStopSimulation()
{
    return (impl->currentFrame / impl->worldFrameRate);
}


bool DSIImpl::onFrictionPropertyChanged(double mu, int type)
{
    if(mu > 0.0){
        if(type == 0){
            staticFriction = mu;
        } else if(type == 1){
            slipFriction = mu;
        }
        return true;
    }
    return false;
}


void DynamicsSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Static friction"), impl->staticFriction,
                (bind(&DSIImpl::onFrictionPropertyChanged, impl, _1, 0)));
    putProperty(_("Slip friction"), impl->slipFriction,
                (bind(&DSIImpl::onFrictionPropertyChanged, impl, _1, 1)));
}


bool DynamicsSimulatorItem::store(Archive& archive)
{
    archive.write("staticFriction", impl->staticFriction);
    archive.write("slipFriction", impl->slipFriction);
    return true;
}


bool DynamicsSimulatorItem::restore(const Archive& archive)
{
    archive.read("staticFriction", impl->staticFriction);
    archive.read("slipFriction", impl->slipFriction);
    return true;
}
