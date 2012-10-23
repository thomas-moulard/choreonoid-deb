/**
   @author Shin'ichiro NAKAOKA
*/

#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "KinematicFaultChecker.h"
#include "LinkSelectionView.h"
#include <cnoid/Archive>
#include <cnoid/MainWindow>
#include <cnoid/MenuManager>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <cnoid/Separator>
#include <cnoid/EigenUtil>
#include <QDialog>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QFrame>
#include <QLabel>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <map>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    bool USE_DUPLICATED_BODY = false;

    KinematicFaultChecker* checkerInstance = 0;

#ifdef _MSC_VER
	inline long lround(double x) {
		return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
	}
#endif
}

namespace cnoid {

    class KinematicFaultCheckerImpl : public QDialog
    {
    public:
        MessageView& mes;
        std::ostream& os;
        
        CheckBox positionCheck;
        DoubleSpinBox angleMarginSpin;
        DoubleSpinBox translationMarginSpin;
        CheckBox velocityCheck;

        QButtonGroup radioGroup;
        RadioButton allJointsRadio;
        RadioButton selectedJointsRadio;
        RadioButton nonSelectedJointsRadio;
        
        DoubleSpinBox velocityLimitRatioSpin;
        CheckBox collisionCheck;

        CheckBox onlyTimeBarRangeCheck;

        int numFaults;
        vector<int> lastPosFaultFrames;
        vector<int> lastVelFaultFrames;
        typedef std::map<ColdetLinkPair*, int> LastCollisionFrameMap;
        LastCollisionFrameMap lastCollisionFrames;

        double frameRate;
        double angleMargin;
        double translationMargin;
        double velocityLimitRatio;

        KinematicFaultCheckerImpl();
        bool store(Archive& archive);
        void restore(const Archive& archive);
        void apply();
        int checkFaults(
            BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
            bool checkPosition, bool checkVelocity, bool checkCollision,
            dynamic_bitset<> linkSelection, double beginningTime, double endingTime);
        void putJointPositionFault(int frame, Link* joint, std::ostream& os);
        void putJointVelocityFault(int frame, Link* joint, std::ostream& os);
        void putSelfCollision(int frame, ColdetLinkPair* linkPair, std::ostream& os);
    };
}


void KinematicFaultChecker::initialize(ExtensionManager& ext)
{
    if(!checkerInstance){
        checkerInstance = ext.manage(new KinematicFaultChecker());

        MenuManager& mm = ext.menuManager();
        mm.setPath("/Tools");
        mm.addItem(_("Kinematic Fault Checker"))
            ->sigTriggered().connect(bind(&KinematicFaultCheckerImpl::show, checkerInstance->impl));
        
        ext.connectProjectArchiver(
            "KinematicFaultChecker",
            bind(&KinematicFaultCheckerImpl::store, checkerInstance->impl, _1),
            bind(&KinematicFaultCheckerImpl::restore, checkerInstance->impl, _1));
    }
}


KinematicFaultChecker* KinematicFaultChecker::instance()
{
    return checkerInstance;
}


KinematicFaultChecker::KinematicFaultChecker()
{
    impl = new KinematicFaultCheckerImpl();
}


KinematicFaultChecker::~KinematicFaultChecker()
{
    delete impl;
}


KinematicFaultCheckerImpl::KinematicFaultCheckerImpl()
    : QDialog(MainWindow::instance()),
      mes(*MessageView::mainInstance()),
      os(mes.cout())
{
    setWindowTitle(_("Kinematic Fault Checker"));
    
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);
    
    QHBoxLayout* hbox = new QHBoxLayout();
    
    positionCheck.setText(_("Joint position check"));
    positionCheck.setChecked(true);
    hbox->addWidget(&positionCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Angle margin")));
    angleMarginSpin.setDecimals(2);
    angleMarginSpin.setRange(-99.99, 99.99);
    angleMarginSpin.setSingleStep(0.01);
    hbox->addWidget(&angleMarginSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Translation margin")));
    translationMarginSpin.setDecimals(4);
    translationMarginSpin.setRange(-9.9999, 9.9999);
    translationMarginSpin.setSingleStep(0.0001);
    hbox->addWidget(&translationMarginSpin);
    hbox->addWidget(new QLabel("[m]"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();
    
    velocityCheck.setText(_("Joint velocity check"));
    velocityCheck.setChecked(true);
    hbox->addWidget(&velocityCheck);
    hbox->addSpacing(10);

    hbox->addWidget(new QLabel(_("Limit ratio")));
    velocityLimitRatioSpin.setDecimals(0);
    velocityLimitRatioSpin.setRange(1.0, 100.0);
    velocityLimitRatioSpin.setValue(100.0);
    hbox->addWidget(&velocityLimitRatioSpin);
    hbox->addWidget(new QLabel("%"));

    hbox->addStretch();
    vbox->addLayout(hbox);
    hbox = new QHBoxLayout();

    radioGroup.addButton(&allJointsRadio);
    radioGroup.addButton(&selectedJointsRadio);
    radioGroup.addButton(&nonSelectedJointsRadio);
    
    allJointsRadio.setText(_("All joints"));
    allJointsRadio.setChecked(true);
    hbox->addWidget(&allJointsRadio);

    selectedJointsRadio.setText(_("Selected joints"));
    hbox->addWidget(&selectedJointsRadio);

    nonSelectedJointsRadio.setText(_("Non-selected joints"));
    hbox->addWidget(&nonSelectedJointsRadio);

    hbox->addStretch();
    vbox->addLayout(hbox);
    vbox->addWidget(new HSeparator);
    hbox = new QHBoxLayout();
    
    collisionCheck.setText(_("Self-collision check"));
    collisionCheck.setChecked(true);
    hbox->addWidget(&collisionCheck);

    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);
    
    hbox = new QHBoxLayout();
    onlyTimeBarRangeCheck.setText(_("Time bar's range only"));
    onlyTimeBarRangeCheck.setChecked(false);
    hbox->addWidget(&onlyTimeBarRangeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect(bind(&KinematicFaultCheckerImpl::apply, this));
    
    vbox->addWidget(buttonBox);
}


bool KinematicFaultCheckerImpl::store(Archive& archive)
{
    archive.write("checkJointPositions", positionCheck.isChecked());
    archive.write("angleMargin", angleMarginSpin.value());
    archive.write("translationMargin", translationMarginSpin.value());
    archive.write("checkJointVelocities", velocityCheck.isChecked());
    archive.write("velocityLimitRatio", velocityLimitRatioSpin.value());
    archive.write("targetJoints",
                  (allJointsRadio.isChecked() ? "all" :
                   (selectedJointsRadio.isChecked() ? "selected" : "non-selected")));
    archive.write("checkSelfCollisions", collisionCheck.isChecked());
    archive.write("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked());
    return true;
}


void KinematicFaultCheckerImpl::restore(const Archive& archive)
{
    positionCheck.setChecked(archive.get("checkJointPositions", positionCheck.isChecked()));
    angleMarginSpin.setValue(archive.get("angleMargin", angleMarginSpin.value()));
    translationMarginSpin.setValue(archive.get("translationMargin", translationMarginSpin.value()));
    velocityCheck.setChecked(archive.get("checkJointVelocities", velocityCheck.isChecked()));
    velocityLimitRatioSpin.setValue(archive.get("velocityLimitRatio", velocityLimitRatioSpin.value()));
    string target;
    if(archive.read("targetJoints", target)){
        if(target == "all"){
            allJointsRadio.setChecked(true);
        } else if(target == "selected"){
            selectedJointsRadio.setChecked(true);
        } else if(target == "non-selected"){
            nonSelectedJointsRadio.setChecked(true);
        }
    }
    collisionCheck.setChecked(archive.get("checkSelfCollisions", collisionCheck.isChecked()));
    onlyTimeBarRangeCheck.setChecked(archive.get("onlyTimeBarRange", onlyTimeBarRangeCheck.isChecked()));
}


void KinematicFaultCheckerImpl::apply()
{
    bool processed = false;
        
    ItemList<BodyMotionItem> items = ItemTreeView::mainInstance()->selectedItems<BodyMotionItem>();
    if(items.empty()){
        mes.notify(_("No BodyMotionItems are selected."));
    } else {
        for(size_t i=0; i < items.size(); ++i){
            BodyMotionItem* motionItem = items[i];
            BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>();
            if(!bodyItem){
                mes.notify(str(format(_("%1% is not owned by any BodyItem. Check skiped.")) % motionItem->name()));
            } else {
                mes.putln();
                mes.notify(str(format(_("Applying the Kinematic Fault Checker to %1% ..."))
                               % motionItem->headItem()->name()));
                
                dynamic_bitset<> linkSelection;
                if(selectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->getLinkSelection(bodyItem);
                } else if(nonSelectedJointsRadio.isChecked()){
                    linkSelection = LinkSelectionView::mainInstance()->getLinkSelection(bodyItem);
                    linkSelection.flip();
                } else {
                    linkSelection.resize(bodyItem->body()->numLinks(), true);
                }
                
                double beginningTime = 0.0;
                double endingTime = motionItem->motion()->getTimeLength();
                    std::numeric_limits<double>::max();
                if(onlyTimeBarRangeCheck.isChecked()){
                    TimeBar* timeBar = TimeBar::instance();
                    beginningTime = timeBar->minTime();
                    endingTime = timeBar->maxTime();
                }
                
                int n = checkFaults(bodyItem, motionItem, mes.cout(),
                                    positionCheck.isChecked(),
                                    velocityCheck.isChecked(),
                                    collisionCheck.isChecked(),
                                    linkSelection,
                                    beginningTime, endingTime);
                
                if(n > 0){
                    if(n == 1){
                        mes.notify(_("A fault has been detected."));
                    } else {
                        mes.notify(str(format(_("%1% faults have been detected.")) % n));
                    }
                } else {
                    mes.notify(_("No faults have been detected."));
                }
                processed = true;
            }
        }
    }
}


/**
   @return Number of detected faults
*/
int KinematicFaultChecker::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os, double beginningTime, double endingTime)
{
    dynamic_bitset<> linkSelection(bodyItem->body()->numLinks());
    linkSelection.set();
    return impl->checkFaults(
        bodyItem, motionItem, os, true, true, true, linkSelection, beginningTime, endingTime);
}


int KinematicFaultCheckerImpl::checkFaults
(BodyItem* bodyItem, BodyMotionItem* motionItem, std::ostream& os,
 bool checkPosition, bool checkVelocity, bool checkCollision, dynamic_bitset<> linkSelection,
 double beginningTime, double endingTime)
{
    numFaults = 0;

    BodyPtr body = bodyItem->body();
    BodyMotionPtr motion = motionItem->motion();
    MultiValueSeqPtr qseq = motion->jointPosSeq();;
    MultiAffine3SeqPtr pseq = motion->linkPosSeq();
    
    if((!checkPosition && !checkVelocity && !checkCollision) || body->isStaticModel() || !qseq->getNumFrames()){
        return numFaults;
    }
    
    vector<ColdetLinkPairPtr>* pColdetLinkPairs;

    BodyItem::KinematicState orgKinematicState;
    
    if(USE_DUPLICATED_BODY){
        body = body->duplicate();
        pColdetLinkPairs = new vector<ColdetLinkPairPtr>(bodyItem->selfColdetPairs);
    } else {
        bodyItem->storeKinematicState(orgKinematicState);
        pColdetLinkPairs = &bodyItem->selfColdetPairs;
    }

    const int numJoints = std::min(body->numJoints(), qseq->numParts());
    const int numLinks = std::min(body->numLinks(), pseq->numParts());
    const int numPairs = pColdetLinkPairs->size();

    frameRate = motion->frameRate();
    double stepRatio2 = 2.0 / frameRate;
    angleMargin = radian(angleMarginSpin.value());
    translationMargin = translationMarginSpin.value();
    velocityLimitRatio = velocityLimitRatioSpin.value() / 100.0;

    int beginningFrame = std::max(0, (int)(beginningTime * frameRate));
    int endingFrame = std::min((motion->numFrames() - 1), (int)lround(endingTime * frameRate));

    lastPosFaultFrames.clear();
    lastPosFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastVelFaultFrames.clear();
    lastVelFaultFrames.resize(numJoints, std::numeric_limits<int>::min());
    lastCollisionFrames.clear();

    if(checkCollision){
        Link* root = body->rootLink();
        root->p.setZero();
        root->R.setIdentity();
    }
        
    for(int frame = beginningFrame; frame <= endingFrame; ++frame){

        int prevFrame = (frame == beginningFrame) ? beginningFrame : frame - 1;
        int nextFrame = (frame == endingFrame) ? endingFrame : frame + 1;

        for(int i=0; i < numJoints; ++i){
            Link* joint = body->joint(i);
            double q = qseq->at(frame, i);
            joint->q = q;
            if(joint->index >= 0 && linkSelection[joint->index]){
                if(checkPosition){
                    bool fault = false;
                    if(joint->jointType == Link::ROTATIONAL_JOINT){
                        fault = (q > (joint->ulimit - angleMargin) || q < (joint->llimit + angleMargin));
                    } else if(joint->jointType == Link::SLIDE_JOINT){
                        fault = (q > (joint->ulimit - translationMargin) || q < (joint->llimit + translationMargin));
                    }
                    if(fault){
                        putJointPositionFault(frame, joint, os);
                    }
                }
                if(checkVelocity){
                    double dq = (qseq->at(nextFrame, i) - qseq->at(prevFrame, i)) / stepRatio2;
                    joint->dq = dq;
                    if(dq > (joint->uvlimit * velocityLimitRatio) || dq < (joint->lvlimit * velocityLimitRatio)){
                        putJointVelocityFault(frame, joint, os);
                    }
                }
            }
        }

        if(checkCollision){

            body->calcForwardKinematics();

            for(int i=0; i < numLinks; ++i){
                Link* link = body->link(i);
                const Affine3& p = pseq->at(frame, i);
                link->p = p.translation();
                link->R = p.linear();
            }
        
            for(int i=0; i < numPairs; ++i){
                ColdetLinkPair* linkPair = (*pColdetLinkPairs)[i].get();
                linkPair->updatePositions();
                linkPair->detectCollisions();
                if(linkPair->checkCollision()){
                    putSelfCollision(frame, linkPair, os);
                }
            }
        }
    }

    if(USE_DUPLICATED_BODY){
        delete pColdetLinkPairs;
    } else {
        bodyItem->restoreKinematicState(orgKinematicState);
    }

    return numFaults;
}


void KinematicFaultCheckerImpl::putJointPositionFault(int frame, Link* joint, std::ostream& os)
{
    static format f1(_("%1$7.3f [s]: Position limit over of %2% (%3% is beyond the range (%4% , %5%) with margin %6%.)"));
    static format f2(_("%1$7.3f [s]: Position limit over of %2% (%3% is beyond the range (%4% , %5%).)"));
    
    if(frame > lastPosFaultFrames[joint->jointId] + 1){
        double q, l, u, m;
        if(joint->jointType == Link::ROTATIONAL_JOINT){
            q = degree(joint->q);
            l = degree(joint->llimit);
            u = degree(joint->ulimit);
            m = degree(angleMargin);
        } else {
            q = joint->q;
            l = joint->llimit;
            u = joint->ulimit;
            m = translationMargin;
        }

        if(m != 0.0){
            os << (f1 % (frame / frameRate) % joint->name() % q % l % u % m) << endl;
        } else {
            os << (f2 % (frame / frameRate) % joint->name() % q % l % u) << endl;
        }

        numFaults++;
    }
    lastPosFaultFrames[joint->jointId] = frame;
}


void KinematicFaultCheckerImpl::putJointVelocityFault(int frame, Link* joint, std::ostream& os)
{
    static format f(_("%1$7.3f [s]: Velocity limit over of %2% (%3% is %4$.0f %% of the range (%5% , %6%).)"));
    
    if(frame > lastVelFaultFrames[joint->jointId] + 1){
        double dq, l, u;
        if(joint->jointType == Link::ROTATIONAL_JOINT){
            dq = degree(joint->dq);
            l = degree(joint->lvlimit);
            u = degree(joint->uvlimit);
        } else {
            dq = joint->dq;
            l = joint->lvlimit;
            u = joint->uvlimit;
        }

        double r = (dq < 0.0) ? (dq / l) : (dq / u);
        r *= 100.0;

        os << (f % (frame / frameRate) % joint->name() % dq % r % l % u) << endl;

        numFaults++;
    }
    lastVelFaultFrames[joint->jointId] = frame;
}


void KinematicFaultCheckerImpl::putSelfCollision(int frame, ColdetLinkPair* linkPair, std::ostream& os)
{
    static format f(_("%1$7.3f [s]: Collision between %2% and %3%"));
    
    bool putMessage = false;
    LastCollisionFrameMap::iterator p = lastCollisionFrames.find(linkPair);
    if(p == lastCollisionFrames.end()){
        putMessage = true;
        lastCollisionFrames[linkPair] = frame;
    } else {
        if(frame > p->second + 1){
            putMessage = true;
        }
        p->second = frame;
    }

    if(putMessage){
        os << (f % (frame / frameRate) % linkPair->link(0)->name() % linkPair->link(1)->name()) << endl;
        numFaults++;
    }
}
