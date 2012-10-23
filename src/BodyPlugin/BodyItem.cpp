/**
   @file
   @author Shin'ichiro Nakaoka
   @todo Eliminate the dependency on WorldItem and KinematicsBar
*/

#include "BodyItem.h"
#include "WorldItem.h"
#include "KinematicsBar.h"
#include <iostream>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <cnoid/LeggedBody>
#include <cnoid/YamlReader>
#include <cnoid/EigenYaml>
#include <cnoid/Archive>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/PinDragIK>
#include <cnoid/PenetrationBlocker>

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    const bool TRACE_FUNCTIONS = false;

    BodyItem::KinematicState kinematicStateCopy;

    /// \todo move this to hrpUtil ?
    inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

    bool loadBodyItem(BodyItem* item, const std::string& filename, std::ostream& os)
    {
        bool loaded = false;

        if(item->loadModelFile(filename)){
            if(item->name().empty()){
                item->setName(item->body()->modelName());
            }
            loaded = true;
        } else {
            os << item->errorMessage();
        }

        return loaded;
    }
    
    void onSigOptionsParsed(boost::program_options::variables_map& variables)
    {
        if(variables.count("hrpmodel")){
            vector<string> modelFileNames = variables["hrpmodel"].as< vector<string> >();
            for(size_t i=0; i < modelFileNames.size(); ++i){
                BodyItemPtr item(new BodyItem());
                if(item->load(modelFileNames[i], "OpenHRP-VRML-MODEL")){
                    RootItem::mainInstance()->addChildItem(item);
                }
            }
        }
    }
}
    

void cnoid::initializeBodyItem(ExtensionManager& ext)
{
    ext.itemManager().registerClass<BodyItem>(N_("BodyItem"));
    ext.itemManager().addLoader<BodyItem>(
        _("OpenHRP model file"), "OpenHRP-VRML-MODEL", "wrl;yaml", bind(loadBodyItem, _1, _2, _3));
    ext.optionManager().addOption("hrpmodel", program_options::value< vector<string> >(), "load an OpenHRP model file");
    ext.optionManager().sigOptionsParsed().connect(onSigOptionsParsed);

    kinematicStateCopy.p.setZero();
    kinematicStateCopy.R.setIdentity();
    kinematicStateCopy.zmp.setZero();
}


BodyItem::BodyItem()
    : body_(new Body()),
      sigKinematicStateChanged_(bind(&BodyItem::emitSigKinematicStateChanged, this)),
      sigKinematicStateEdited_(bind(&BodyItem::emitSigKinematicStateEdited, this)),
      updateSelfCollisionsCaller(bind(&BodyItem::updateSelfCollisions, this, false), IDLE_PRIORITY_NORMAL)
{
    isSelfCollisionDetectionEnabled_ = true;
    init();
}


BodyItem::BodyItem(const BodyItem& org)
    : Item(org),
      body_(org.body_->duplicate()),
      modelNodeSet_(org.modelNodeSet_),
      modelFilePath_(org.modelFilePath_),
      sigKinematicStateChanged_(bind(&BodyItem::emitSigKinematicStateChanged, this)),
      updateSelfCollisionsCaller(bind(&BodyItem::updateSelfCollisions, this, false), IDLE_PRIORITY_NORMAL)
{
    isSelfCollisionDetectionEnabled_ = org.isSelfCollisionDetectionEnabled_;
    init();
    setCurrentBaseLink(body_->link(org.currentBaseLink()->index));
}


void BodyItem::init()
{
    kinematicsBar = KinematicsBar::instance();
    isFkRequested = isVelFkRequested = isAccFkRequested = false;
    currentHistoryIndex = 0;
    isCurrentKinematicStateInHistory = false;
    needToAppendKinematicStateToHistory = false;
    isCallingSlotsOnKinematicStateEdited = false;
    isSelfCollisionUpdateNeeded = false;
    isColdetModelPositionUpdateNeeded = false;
    initBody();

    sigPositionChanged().connect(bind(&BodyItem::onPositionChanged, this));
}


BodyItem::~BodyItem()
{

}


void BodyItem::initBody()
{
    setCurrentBaseLink(body_->rootLink());
    if(pinDragIK_){
        pinDragIK_.reset();
    }

    zmp_.setZero();

    int n = body_->numLinks();
    worldColdetPairsOfLink_.resize(n);
    worldCollisionLinkBitSet.resize(n);
    selfCollisionLinkBitSet.resize(n);

    updateSelfColdetPairs();
}


void BodyItem::onPositionChanged()
{
    worldItem_ = findOwnerItem<WorldItem>();
    if(!worldItem_){
        for(size_t i=0; i < worldColdetPairsOfLink_.size(); ++i){
            worldColdetPairsOfLink_[i].clear();
        }
    }
}


bool BodyItem::loadModelFile(const std::string& filename)
{
    errorMessage_.clear();

    BodyLoader bodyLoader;

    MessageView::mainInstance()->beginStdioRedirect();
    BodyPtr newBody = bodyLoader.loadModelFile(filename, true, true, true);
    MessageView::mainInstance()->endStdioRedirect();
            
    if(!newBody){
        errorMessage_ = bodyLoader.errorMessage();
        modelNodeSet_.reset();
        modelFilePath_.clear();

    } else {
        body_ = newBody;
        body_->setName(name());
        modelNodeSet_ = bodyLoader.modelNodeSet();
        modelFilePath_ = filename;
    }

    initBody();

    return (newBody);
}


void BodyItem::setName(const std::string& name)
{
    if(body_){
        body_->setName(name);
    }
    Item::setName(name);
}


void BodyItem::setCurrentBaseLink(Link* link)
{
    if(link != currentBaseLink_){
        if(link){
            fkTraverse.find(link, true, true);
        } else {
            fkTraverse.find(body_->rootLink());
        }
    }
    currentBaseLink_ = link;
}


/**
   Forward kinematics from the current base link is done.
*/
void BodyItem::calcForwardKinematics(bool calcVelocity, bool calcAcceleration)
{
    fkTraverse.calcForwardKinematics(calcVelocity, calcAcceleration);
}


void BodyItem::copyKinematicState()
{
    storeKinematicState(kinematicStateCopy);

}


void BodyItem::pasteKinematicState()
{
    restoreKinematicState(kinematicStateCopy);
    notifyKinematicStateChange(false);    
}


void BodyItem::storeKinematicState(KinematicState& state)
{
    const int n = body_->numJoints();
    state.q.resize(n);
    for(int i=0; i < n; ++i){
        state.q[i] = body_->joint(i)->q;
    }
    state.p = body_->rootLink()->p;
    state.R = body_->rootLink()->R;
    state.zmp = zmp_;
}


/**
   @return false if the restored state is same as the current state
*/
bool BodyItem::restoreKinematicState(const KinematicState& state)
{
    bool modified = false;

    int n = std::min(static_cast<int>(state.q.size()), body_->numJoints());
    for(int i=0; i < n; ++i){
        double& q = body_->joint(i)->q;
        if(q != state.q[i]){
            q = state.q[i];
            modified = true;
        }
    }

    Link* root = body_->rootLink();
    if(!modified){
        modified = (root->p != state.p || root->R != state.R || zmp_ != state.zmp);
    }
    if(modified){
        root->p = state.p;
        root->R = state.R;
        zmp_ = state.zmp;
        body_->calcForwardKinematics();
    }

    return modified;
}


void BodyItem::beginKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::beginKinematicStateEdit()" << endl;
    }

    if(!isCurrentKinematicStateInHistory){
        appendKinematicStateToHistory();
    }
}


void BodyItem::acceptKinematicStateEdit()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::acceptKinematicStateEdit()" << endl;
    }

    //appendKinematicStateToHistory();
    needToAppendKinematicStateToHistory = true;
    sigKinematicStateEdited_.request();
}


void BodyItem::appendKinematicStateToHistory()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::appendKinematicStateToHistory()" << endl;
    }

    KinematicStatePtr state(new KinematicState);
    storeKinematicState(*state);

    if(kinematicStateHistory.empty() || (currentHistoryIndex == kinematicStateHistory.size() - 1)){
        kinematicStateHistory.push_back(state);
        currentHistoryIndex = kinematicStateHistory.size() - 1;
    } else {
        ++currentHistoryIndex;
        kinematicStateHistory.resize(currentHistoryIndex + 1);
        kinematicStateHistory[currentHistoryIndex] = state;
    }
        
    if(kinematicStateHistory.size() > 20){
        kinematicStateHistory.pop_front();
        currentHistoryIndex--;
    }

    isCurrentKinematicStateInHistory = true;
}


bool BodyItem::undoKinematicState()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::undoKinematicState()" << endl;
    }

    bool done = false;
    bool modified = false;

    if(!isCurrentKinematicStateInHistory){
        if(currentHistoryIndex < kinematicStateHistory.size()){
            done = true;
            modified = restoreKinematicState(*kinematicStateHistory[currentHistoryIndex]);
        }
    } else {
        if(currentHistoryIndex > 0){
            done = true;
            modified = restoreKinematicState(*kinematicStateHistory[--currentHistoryIndex]);
        }
    }

    if(done){
        if(modified){
            notifyKinematicStateChange(false);
            isCurrentKinematicStateInHistory = true;
            sigKinematicStateEdited_.request();
        } else {
            isCurrentKinematicStateInHistory = true;
            done = undoKinematicState();
        }
    }

    return done;
}


bool BodyItem::redoKinematicState()
{
    if(TRACE_FUNCTIONS){
        cout << "BodyItem::redoKinematicState()" << endl;
    }
    
    if(currentHistoryIndex + 1 < kinematicStateHistory.size()){
        restoreKinematicState(*kinematicStateHistory[++currentHistoryIndex]);
        notifyKinematicStateChange(false);
        isCurrentKinematicStateInHistory = true;
        sigKinematicStateEdited_.request();
        return true;
    }
    return false;
}
        

PinDragIKptr BodyItem::pinDragIK()
{
    if(!pinDragIK_){
        pinDragIK_.reset(new PinDragIK(body_));
    }
    return pinDragIK_;
}


InverseKinematicsPtr BodyItem::getCurrentIK(Link* targetLink)
{
    InverseKinematicsPtr ik;

    if(KinematicsBar::instance()->mode() == KinematicsBar::AUTO_MODE){
        ik = body_->getDefaultIK(targetLink);
    }

    if(!ik){
        pinDragIK(); // create if not created
        if(pinDragIK_->numPinnedLinks() > 0 || !currentBaseLink_){
            pinDragIK_->setTargetLink(targetLink, true);
            if(pinDragIK_->initialize()){
                ik = pinDragIK_;
            }
        }
    }
    if(!ik){
        if(currentBaseLink_){
            ik = body_->getJointPath(currentBaseLink_, targetLink);
        }
    }

    return ik;
}


PenetrationBlockerPtr BodyItem::createPenetrationBlocker(Link* link, bool excludeSelfCollisions)
{
    PenetrationBlockerPtr blocker;
    
    if(link->body == body_ && worldItem_){
        std::vector<ColdetLinkPairPtr>& pairs = worldColdetPairsOfLink(link->index);
        if(!pairs.empty()){
            blocker.reset(new PenetrationBlocker(link));
            for(size_t i=0; i < pairs.size(); ++i){
                Link* oppnentLink = pairs[i]->link(0);
                if(oppnentLink == link){
                    oppnentLink = pairs[i]->link(1);
                }
                if(excludeSelfCollisions){
                    if(oppnentLink->body == body_){
                        continue;
                    }
                }
                blocker->addOpponentLink(oppnentLink);
            }
            blocker->setDepth(kinematicsBar->penetrationBlockDepth());
        }
    }
    return blocker;
}


void BodyItem::moveToOrigin()
{
    beginKinematicStateEdit();
    
    Vector3 p;
    Matrix3 R;
    body_->getDefaultRootPosition(p, R);
    Link* rootLink = body_->rootLink();
    rootLink->p = p;
    rootLink->R = R;
    body_->calcForwardKinematics();
    
    notifyKinematicStateChange(false);
    acceptKinematicStateEdit();
}


void BodyItem::setPresetPose(PresetPoseID id)
{
    int jointIndex = 0;

    beginKinematicStateEdit();
    
    if(id == STANDARD_POSE){
        const YamlSequence& pose = *body_->info()->findSequence("standardPose");
        if(pose.isValid()){
            const int n = std::min(pose.size(), body_->numJoints());
            while(jointIndex < n){
                body_->joint(jointIndex)->q = radian(pose[jointIndex].toDouble());
                jointIndex++;
            }
        }
    }

    const int n = body_->numJoints();
    while(jointIndex < n){
        body_->joint(jointIndex++)->q = 0.0;
    }

    fkTraverse.calcForwardKinematics();
    notifyKinematicStateChange(false);
    acceptKinematicStateEdit();
}


const Vector3& BodyItem::centerOfMass()
{
    if(!updateFlags.test(UF_CM)){
        body_->calcCM();
        updateFlags.set(UF_CM);
    }

    return body_->lastCM();
}


/**
   \todo use getDefaultIK() if the kinematics bar is in the AUTO mode.
*/
bool BodyItem::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    bool result = false;
    
    LeggedBody* legged = dynamic_cast<LeggedBody*>(body_.get());

    if(legged){
        
        KinematicState orgKinematicState;
        storeKinematicState(orgKinematicState);
        beginKinematicStateEdit();
        
        result = legged->doLegIkToMoveCm(c, onlyProjectionToFloor);

        if(result){
            notifyKinematicStateChange();
            acceptKinematicStateEdit();
            updateFlags.set(UF_CM);
        } else {
            restoreKinematicState(orgKinematicState);
        }
    }

    return result;
}


bool BodyItem::setStance(double width)
{
    bool result = false;
    
    LeggedBody* legged = dynamic_cast<LeggedBody*>(body_.get());

    if(legged){
        
        KinematicState orgKinematicState;
        storeKinematicState(orgKinematicState);
        beginKinematicStateEdit();
        
        result = legged->setStance(width, currentBaseLink_);

        if(result){
            notifyKinematicStateChange();
            acceptKinematicStateEdit();
        } else {
            restoreKinematicState(orgKinematicState);
        }
    }

    return result;
}
                

boost::optional<Vector3> BodyItem::getParticularPosition(PositionType position)
{
    boost::optional<Vector3> pos;
    
    if(position == ZMP){
        pos = zmp_;

    } else {
        if(position == CM_PROJECTION){
            pos = centerOfMass();

        } else {
            LeggedBody* legged = dynamic_cast<LeggedBody*>(body_.get());
            if(legged){
                if(position == HOME_COP){
                    pos = legged->homeCopOfSoles();
                } else if(position == RIGHT_HOME_COP || position == LEFT_HOME_COP) {
                    if(legged->numFeet() == 2){
                        pos = legged->homeCopOfSole((position == RIGHT_HOME_COP) ? 0 : 1);
                    }
                }

            }
        }
        if(pos){
            (*pos).z() = 0.0;
        }
    }

    return pos;
}


void BodyItem::editZmp(const Vector3& zmp)
{
    beginKinematicStateEdit();
    setZmp(zmp);
    notifyKinematicStateChange(false);
    acceptKinematicStateEdit();
}


void BodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    if(!isCallingSlotsOnKinematicStateEdited){
        isCurrentKinematicStateInHistory = false;
    }

    if(requestFK){
        isFkRequested |= requestFK;
        isVelFkRequested |= requestVelFK;
        isAccFkRequested |= requestAccFK;
    }
    updateFlags.reset();

    if(isSelfCollisionDetectionEnabled_){
        isSelfCollisionUpdateNeeded = true;
    }
    isColdetModelPositionUpdateNeeded = true;
    
    sigKinematicStateChanged_.request();
}


void BodyItem::notifyKinematicStateChange
(boost::signals::connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    sigKinematicStateChanged_.requestBlocking(connectionToBlock);
    notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK);
}


void BodyItem::emitSigKinematicStateChanged()
{
    if(isFkRequested){
        fkTraverse.calcForwardKinematics(isVelFkRequested, isAccFkRequested);
        isFkRequested = isVelFkRequested = isAccFkRequested = false;
    }

    sigKinematicStateChanged_.signal()();

    if(isSelfCollisionUpdateNeeded){
        updateSelfCollisionsCaller.setPriority(kinematicsBar->collisionDetectionPriority());
        updateSelfCollisionsCaller.request();
    }
    
    if(needToAppendKinematicStateToHistory){
        appendKinematicStateToHistory();
        needToAppendKinematicStateToHistory = false;
    }
}


void BodyItem::emitSigKinematicStateEdited()
{
    isCallingSlotsOnKinematicStateEdited = true;
    sigKinematicStateEdited_.signal()();
    isCallingSlotsOnKinematicStateEdited = false;
    
    if(!sigKinematicStateEdited_.isBeingRequested() && needToAppendKinematicStateToHistory){
        appendKinematicStateToHistory();
        needToAppendKinematicStateToHistory = false;
    }
}


void BodyItem::enableSelfCollisionDetection(bool on)
{
    if(isSelfCollisionDetectionEnabled_ && !on){
        isSelfCollisionDetectionEnabled_ = false;
        clearSelfCollisions();
        notifyUpdate();

    } else if(!isSelfCollisionDetectionEnabled_ && on){
        isSelfCollisionDetectionEnabled_ = true;
        updateSelfColdetPairs();
        notifyUpdate();
    }
}


void BodyItem::updateSelfColdetPairs()
{
    selfColdetPairs.clear();

    int n = body_->numLinks();
    int excludeTreeDepth = 1;
    dynamic_bitset<> exclusions(n);

    const YamlMapping& selfCollisionInfo = *body_->info()->findMapping("selfCollisionDetection");

    if(selfCollisionInfo.isValid()){
        excludeTreeDepth = selfCollisionInfo.get("excludeTreeDepth", 1);
        const YamlSequence& excludeLinks = *selfCollisionInfo.findSequence("excludeLinks");
        for(int i=0; i < excludeLinks.size(); ++i){
            Link* link = body_->link(excludeLinks[i].toString());
            if(link){
                exclusions[link->index] = true;
            }
        }
    }

    for(int i=0; i < n; ++i){
        Link* link1 = body_->link(i);
        if(!exclusions[link1->index]){
            for(int j=i+1; j < n; ++j){
                Link* link2 = body_->link(j);
                if(!exclusions[link2->index]){
                    bool skip = false;
                    Link* parent1 = link1;
                    Link* parent2 = link2;
                    for(int k=0; k < excludeTreeDepth; ++k){
                        if(parent1){
                            parent1 = parent1->parent;
                        }
                        if(parent2){
                            parent2 = parent2->parent;
                        }
                        if(!parent1 && !parent2){
                            break;
                        }
                        if(parent1 == link2 || parent2 == link1){
                            skip = true;
                            break;
                        }
                    }
                    if(!skip){
                        selfColdetPairs.push_back(new ColdetLinkPair(link1, link2));
                    }
                }
            }
        }
    }

    if(isSelfCollisionDetectionEnabled_){
        updateSelfCollisions(true);
    } else {
        clearSelfCollisions();
    }
}


void BodyItem::updateColdetModelPositions(bool force)
{
    if(isColdetModelPositionUpdateNeeded || force){
        const int n = body_->numLinks();
        for(int i=0; i < n; ++i){
            body_->link(i)->updateColdetModelPosition();
        }
        isColdetModelPositionUpdateNeeded = false;
    }
}


/**
   @ret true if a colliding link set changes
*/
bool BodyItem::updateSelfCollisions(bool force)
{
    if(isSelfCollisionUpdateNeeded || force){
        
        bool collisionLinkSetChanged = false;
        
        if(!selfColdetPairs.empty()){

            updateColdetModelPositions();

            selfCollisionLinkBitSet.reset();
    
            for(size_t i=0; i < selfColdetPairs.size(); ++i){
                ColdetLinkPair& linkPair = *selfColdetPairs[i];
                bool prevEmpty = linkPair.collisions().empty();
                bool empty = linkPair.detectCollisions().empty();
                if(prevEmpty != empty){
                    collisionLinkSetChanged = true;
                }
                if(!empty){
                    selfCollisionLinkBitSet.set(linkPair.link(0)->index);
                    selfCollisionLinkBitSet.set(linkPair.link(1)->index);
                }
            }

            if(collisionLinkSetChanged){
                sigSelfCollisionLinkSetChanged_();
            }
            sigSelfCollisionsUpdated_();
        }
        isSelfCollisionUpdateNeeded = false;

        return collisionLinkSetChanged;
    }
    return false;
}


ItemPtr BodyItem::doDuplicate() const
{
    return new BodyItem(*this);
}


void BodyItem::clearSelfCollisions()
{
    selfCollisionLinkBitSet.reset();
    sigSelfCollisionLinkSetChanged_();
    sigSelfCollisionsUpdated_();
}


bool BodyItem::onSelfCollisionDetectionPropertyChanged(bool on)
{
    enableSelfCollisionDetection(on);
    return true;
}


void BodyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Model name"), body_->modelName());
    putProperty(_("Num links"), body_->numLinks());
    putProperty(_("Num joints"), body_->numJoints());
    putProperty(_("Root link"), body_->rootLink()->name());
    putProperty(_("Base link"), currentBaseLink_ ? currentBaseLink_->name() : "none");
    putProperty(_("Mass"), body_->totalMass());
    putProperty(_("Static model ?"), body_->isStaticModel());
    putProperty(_("Model file"), filesystem::path(modelFilePath_).leaf());

    putProperty(_("Self-collision"), isSelfCollisionDetectionEnabled_,
                (bind(&BodyItem::onSelfCollisionDetectionPropertyChanged, this, _1)));

    //          This does not call the function, why ?
    //         (bind(&BodyItem::enableSelfCollisionDetection, this, _1), constant(true)));
}


bool BodyItem::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");
    
    archive.writeRelocatablePath("modelFile", modelFilePath_);
    archive.write("currentBaseLink", (currentBaseLink_ ? currentBaseLink_->name() : ""), YAML_DOUBLE_QUOTED);
    write(archive, "rootPosition", body_->rootLink()->p);
    write(archive, "rootAttitude", body_->rootLink()->R);

    YamlSequence& qs = *archive.createFlowStyleSequence("jointPositions");
    int n = body_->numJoints();
    for(int i=0; i < n; ++i){
        qs.append(body_->joint(i)->q, 10, n);
    }

    archive.write("selfCollisionDetection", isSelfCollisionDetectionEnabled_);
    
    return true;
}


bool BodyItem::restore(const Archive& archive)
{
    bool restored = false;
    
    string modelFile;
    if(archive.readRelocatablePath("modelFile", modelFile)){
        restored = modelFile.empty() || load(modelFile);
    }

    if(restored){
        read(archive, "rootPosition", body_->rootLink()->p);
        read(archive, "rootAttitude", body_->rootLink()->R);
        
        const YamlSequence& qs = *archive.findSequence("jointPositions");
        if(qs.isValid()){
            for(int i=0; i < qs.size(); ++i){
                body_->joint(i)->q = qs[i].toDouble();
            }
        }

        body_->calcForwardKinematics();
        setCurrentBaseLink(body_->link(archive.get("currentBaseLink", "")));

        notifyKinematicStateChange();

        enableSelfCollisionDetection(archive.get("selfCollisionDetection", true));
    }

    return restored;
}
