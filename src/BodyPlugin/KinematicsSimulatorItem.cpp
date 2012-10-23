/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "KinematicsSimulatorItem.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/Body>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

    const bool TRACE_FUNCTIONS = false;

    class BodyUnit
    {
    public:
        int frame;
        double frameRate;
        BodyPtr body;
        MultiValueSeqPtr qseqRef;
        Vector3SeqPtr zmpSeq;
        MultiValueSeqPtr qseqResultBuf;
        MultiValueSeqPtr qseqResult;
        MultiAffine3SeqItemPtr rootResultItem;
        MultiAffine3SeqPtr rootResultBuf;
        MultiAffine3SeqPtr rootResult;
        Link* baseLink;
        LinkTraverse fkTraverse;
        vector<Link*> footLinks;
        vector<double> footLinkOriginHeights;

        bool initialize(BodyItemPtr bodyItem);
        bool updatePosition(int newFrame);
        void putResultToBuf();
        void setBaseLink(Link* link, double originHeight);
        void setBaseLinkByZmp(int frame);
        bool flushBuf();
    };
}


namespace cnoid {
        
    class KSIImpl
    {
    public:
        KSIImpl(KinematicsSimulatorItem* self);
        KSIImpl(KinematicsSimulatorItem* self, const KSIImpl& org);
            
        bool doStartSimulation();
        bool setupBodies();
        void addBody(BodyItemPtr bodyItem);
        bool doStepSimulation();
        double doFlushResults();        

        KinematicsSimulatorItem* self;
            
        vector<BodyUnit> bodyUnits;
        vector<BodyUnit*> bodyUnitsToPutResult;
        vector<BodyUnit*> bodyUnitsToNotifyUpdate;

        int currentFrame;
        int frameAtLastBufferWriting;
        double mainFrameRate;
    };
}


void cnoid::initializeKinematicsSimulatorItem(ExtensionManager& ext)
{
    ext.itemManager().registerClass<KinematicsSimulatorItem>(N_("KinematicsSimulatorItem"));
    ext.itemManager().addCreationPanel<KinematicsSimulatorItem>();
}


bool BodyUnit::initialize(BodyItemPtr bodyItem)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::initialize()" << endl;
    }

    if(bodyItem->body()->isStaticModel()){
        return false;
    }
        
    frame = 0;
    body = bodyItem->body()->duplicate();
    baseLink = 0;

    ItemList<BodyMotionItem> motionItems = bodyItem->getSubItems<BodyMotionItem>();
    if(!motionItems.empty()){
        BodyMotionItemPtr motionItem;
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
        
        frameRate = motionItem->motion()->frameRate();
        qseqRef = motionItem->jointPosSeq();
        if(qseqResult){
            qseqResultBuf.reset(new MultiValueSeq(qseqResult->numParts(), 0, frameRate));
        }

        if(motionItem->hasRelativeZmpSeqItem()){
            zmpSeq = motionItem->relativeZmpSeq();
        }

        rootResultItem = motionItem->linkPosSeqItem();
        rootResultBuf.reset(new MultiAffine3Seq(1, 0, frameRate));
        rootResult = motionItem->linkPosSeq();
        rootResult->setFrameRate(frameRate);
        rootResult->setDimension(1, 1);
    }

    const YamlMapping& info = *bodyItem->body()->info();
    const YamlSequence& footLinkInfos = *info.findSequence("footLinks");
    if(footLinkInfos.isValid()){
        for(int i=0; i < footLinkInfos.size(); ++i){
            const YamlMapping& footLinkInfo = *footLinkInfos[i].toMapping();
            Link* link = body->link(footLinkInfo["link"].toString());
            Vector3 soleCenter;
            if(link && read(footLinkInfo, "soleCenter", soleCenter)){
                footLinks.push_back(link);
                footLinkOriginHeights.push_back(-soleCenter[2]);
            }
        }
    }

    if(!footLinks.empty()){
        if(zmpSeq){
            setBaseLinkByZmp(0);
        } else {
            if(bodyItem->currentBaseLink()){
                int currentBaseLinkIndex = bodyItem->currentBaseLink()->index;
                for(size_t i=0; i < footLinks.size(); ++i){
                    Link* footLink = footLinks[i];
                    if(footLink->index == currentBaseLinkIndex){
                        setBaseLink(footLink, footLinkOriginHeights[i]);
                    }
                }
            } else{
                setBaseLink(footLinks[0], footLinkOriginHeights[0]);
            }
        }
    }

    if(!baseLink){
        if(bodyItem->currentBaseLink()){
            baseLink = body->link(bodyItem->currentBaseLink()->index);
        } else {
            baseLink = body->rootLink();
        }
        fkTraverse.find(baseLink, true, true);
    }

    if(rootResult){
        Link* rootLink = body->rootLink();
        Affine3& initialPos = rootResult->at(0, 0);
        initialPos.translation() = rootLink->p;
        initialPos.linear() = rootLink->R;
    }

    return (qseqRef != 0);
}


void BodyUnit::setBaseLink(Link* link, double originHeight)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::setBaseLink()" << endl;
    }

    baseLink = link;

    link->p[2] = originHeight;

    Matrix3& R = link->R;
    Matrix3::ColXpr x = R.col(0);
    Matrix3::ColXpr y = R.col(1);
    Matrix3::ColXpr z = R.col(2);
    z.normalize();
    y = z.cross(x).normalized();
    x = y.cross(z);

    fkTraverse.find(link, true, true);
    fkTraverse.calcForwardKinematics();
}


void BodyUnit::setBaseLinkByZmp(int frame)
{
    Link* root = body->rootLink();
    const Vector3 zmp = root->R * zmpSeq->at(frame) + root->p;
    int footLinkOnZmpId = -1;
    double l2min = std::numeric_limits<double>::max();
    for(size_t i=0; i < footLinks.size(); ++i){
        double l2 = (footLinks[i]->p - zmp).squaredNorm();
        if(l2 < l2min){
            footLinkOnZmpId = i;
            l2min = l2;
        }
    }
    if(footLinkOnZmpId >= 0 && footLinks[footLinkOnZmpId] != baseLink){
        setBaseLink(footLinks[footLinkOnZmpId], footLinkOriginHeights[footLinkOnZmpId]);
    }
}


bool BodyUnit::updatePosition(int newFrame)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::updatePosition()" << endl;
    }

    if(newFrame >= qseqRef->numFrames()){
        return false;
    }

    frame = newFrame;
    
    const int n = std::min(body->numJoints(), qseqRef->numParts());

    MultiValueSeq::View qRef = qseqRef->frame(frame);
    for(int i=0; i < n; ++i){
        body->joint(i)->q = qRef[i];
    }

    fkTraverse.calcForwardKinematics();

    if(zmpSeq){
        setBaseLinkByZmp(frame);
    } else {
        for(size_t i=0; i < footLinks.size(); ++i){
            Link* link = footLinks[i];
            if(link != baseLink){
                if(link->p[2] < footLinkOriginHeights[i]){
                    setBaseLink(link, footLinkOriginHeights[i]);
                }
            }
        }
    }

    return true;
}

void BodyUnit::putResultToBuf()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::putResultToBuf()" << endl;
    }

    const int bufFrame = rootResultBuf->numFrames();

    if(qseqResultBuf){
        qseqResultBuf->setNumFrames(bufFrame + 1);
        MultiValueSeq::View qResultBuf = qseqResultBuf->frame(bufFrame);
        const int n = qResultBuf.size();
        for(int i=0; i < n; ++i){
            qResultBuf[i] = body->joint(i)->q;
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
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::flushBuf()" << endl;
    }

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
    

KinematicsSimulatorItem::KinematicsSimulatorItem()
{
    impl = new KSIImpl(this);
}


KSIImpl::KSIImpl(KinematicsSimulatorItem* self)
    : self(self)
{

}


KinematicsSimulatorItem::KinematicsSimulatorItem(const KinematicsSimulatorItem& org)
    : SimulatorItem(org),
      impl(new KSIImpl(this, *org.impl))
{
    
}


KSIImpl::KSIImpl(KinematicsSimulatorItem* self, const KSIImpl& org)
    : self(self)
{

}


KinematicsSimulatorItem::~KinematicsSimulatorItem()
{
    delete impl;
}


ItemPtr KinematicsSimulatorItem::doDuplicate() const
{
    return new KinematicsSimulatorItem(*this);
}


QWidget* KinematicsSimulatorItem::settingPanel()
{
    return 0;
}


bool KinematicsSimulatorItem::doStartSimulation()
{
    return impl->doStartSimulation();
}


bool KSIImpl::doStartSimulation()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyUnit::doStartSimulation()" << endl;
    }

    bool result = setupBodies();

    if(result){
        currentFrame = 0;
        frameAtLastBufferWriting = 0;
    }
    
    return result;
}


bool KSIImpl::setupBodies()
{
    if(TRACE_FUNCTIONS){
        cout << "KSIImpl::setupBodies()" << endl;
    }
    
    mainFrameRate = 0;
    bodyUnits.clear();
    
    WorldItemPtr worldItem = self->findOwnerItem<WorldItem>();
    if(worldItem){
        ItemList<BodyItem> bodyItems = worldItem->getBodyItems();
        for(size_t i=0; i < bodyItems.size(); ++i){
            BodyUnit unit;
            if(unit.initialize(bodyItems[i])){
                bodyUnits.push_back(unit);
                if(unit.frameRate > mainFrameRate){
                    mainFrameRate = unit.frameRate;
                }
            }
        }
    }

    return (!bodyUnits.empty() && mainFrameRate > 0);
}


bool KinematicsSimulatorItem::doStepSimulation()
{
    return impl->doStepSimulation();
}


bool KSIImpl::doStepSimulation()
{
    if(TRACE_FUNCTIONS){
        cout << "KSIImpl::doStepSimulation()" << endl;
    }
    
    currentFrame++;

    bodyUnitsToPutResult.clear();
    
    for(size_t i=0; i < bodyUnits.size(); ++i){
        BodyUnit& unit = bodyUnits[i];
        int frame = static_cast<int>(currentFrame * unit.frameRate / mainFrameRate);
        if(frame > unit.frame){
            if(unit.updatePosition(frame)){
                bodyUnitsToPutResult.push_back(&unit);
            }
        }
    }

    bool doContinue = !bodyUnitsToPutResult.empty();

    if(doContinue){
        self->lockResults();
        
        for(size_t i=0; i < bodyUnitsToPutResult.size(); ++i){
            bodyUnitsToPutResult[i]->putResultToBuf();
        }
        frameAtLastBufferWriting = currentFrame;

        self->requestToFlushResults();
        
        self->unlockResults();
    }

    return doContinue;
}


double KinematicsSimulatorItem::doFlushResults()
{
    return impl->doFlushResults();
}


double KSIImpl::doFlushResults()
{
    if(TRACE_FUNCTIONS){
        cout << "KSIImpl::doFlushResults()" << endl;
    }
    
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

    return (frame / mainFrameRate);
}


double KinematicsSimulatorItem::doStopSimulation()
{
    if(TRACE_FUNCTIONS){
        cout << "KinematicsSimulatorItem::doStopSimulation()" << endl;
    }

    return (impl->currentFrame / impl->mainFrameRate);
}


bool KinematicsSimulatorItem::store(Archive& archive)
{
    return true;
}


bool KinematicsSimulatorItem::restore(const Archive& archive)
{
    return true;
}
