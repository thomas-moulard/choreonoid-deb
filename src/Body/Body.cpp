/**
   \file
   \brief The implementation of the body class 
   \author Shin'ichiro Nakaoka
*/

#include "Body.h"
#include "BodyCustomizerInterface.h"
#include "Link.h"
#include "LinkGroup.h"
#include "JointPath.h"
#include "Sensor.h"
#include "CompositeIK.h"
#include <cnoid/EigenUtil>
#include <cnoid/ColdetModel>
#include <cstdlib>

using namespace cnoid;
using namespace std;

static const bool PUT_DEBUG_MESSAGE = true;

static bool pluginsInDefaultDirectoriesLoaded = false;

#ifndef uint
typedef unsigned int uint;
#endif

namespace cnoid {
	
    class CustomizedJointPath : public JointPath
    {
        Body* body;
        int ikTypeId;
        bool isCustomizedIkPathReversed;
        virtual void onJointPathUpdated();
    public:
        CustomizedJointPath(Body* body, Link* baseLink, Link* targetLink);
        virtual ~CustomizedJointPath();
        virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R);
        virtual bool hasAnalyticalIK() const;
    };
}


Body::~Body()
{
    if(customizerHandle){
        customizerInterface->destroy(customizerHandle);
    }
	
    delete rootLink_;

    // delete sensors
    for(int sensorType =0; sensorType < numSensorTypes(); ++sensorType){
        int n = numSensors(sensorType);
        for(int i=0; i < n; ++i){
            Sensor* s = sensor(sensorType, i);
            if(s){
                Sensor::destroy(s);
            }
        }
    }
}


void Body::initialize()
{
    rootLink_ = 0;

    customizerHandle = 0;
    customizerInterface = 0;
    bodyHandleEntity.body = this;
    bodyHandle = &bodyHandleEntity;
}
	

Body::Body()
  : allSensors(Sensor::NUM_SENSOR_TYPES)
{
    initialize();
	
    rootLink_ = new Link;
    rootLink_->body = this;

    lastCM_.setZero();

    info_ = new YamlMapping();

    defaultRootPosition.setZero();
    defaultRootAttitude.setIdentity();
}


Body::Body(const Body& org)
    : name_(org.name_),
      modelName_(org.modelName_),
      allSensors(Sensor::NUM_SENSOR_TYPES),
      info_(org.info_)
{
    initialize();

    lastCM_ = org.lastCM_;
	
    setRootLink(new Link(*org.rootLink()));

    defaultRootPosition = org.defaultRootPosition;
    defaultRootAttitude = org.defaultRootAttitude;
	
    // copy sensors
    std::map<Link*, int> linkToIndexMap;

    for(int i=0; i < org.linkTraverse_.numLinks(); ++i){
        Link* lnk = org.linkTraverse_[i];
        linkToIndexMap[lnk] = i;
    }

    int n = org.numSensorTypes();
    for(int sensorType = 0; sensorType < n; ++sensorType){
        for(int i=0; i < org.numSensors(sensorType); ++i){
            Sensor* orgSensor = org.sensor(sensorType, i);
            int linkIndex = linkToIndexMap[orgSensor->link];
            Link* newLink = linkTraverse_[linkIndex];
            Sensor* cloneSensor = createSensor(newLink, sensorType, orgSensor->id, orgSensor->name);
            *cloneSensor = *orgSensor;
        }
    }

    // do deep copy of linkConnections
    for(size_t i=0; i < org.linkConnections.size(); ++i){
        const LinkConnection& orgConnection = org.linkConnections[i];
        LinkConnection connection(orgConnection);
        for(int j=0; j < 2; ++j){
            connection.link[j] = link(orgConnection.link[j]->index);
        }
    }

    if(org.customizerInterface){
        installCustomizer(org.customizerInterface);
    }
}


BodyPtr Body::duplicate() const
{
    return new Body(*this);
}


void Body::setRootLink(Link* link)
{
    if(rootLink_){
        delete rootLink_;
    }
    rootLink_ = link;

    updateLinkTree();
}


void Body::setDefaultRootPosition(const Vector3& p, const Matrix3& R)
{
    defaultRootPosition = p;
    defaultRootAttitude = R;
}


void Body::getDefaultRootPosition(Vector3& out_p, Matrix3& out_R)
{
    out_p = defaultRootPosition;
    out_R = defaultRootAttitude;
}


Link* Body::createEmptyJoint(int jointId)
{
    Link* empty = new Link;
    empty->body = this;
    empty->jointId = jointId;
    empty->p.setZero();
    empty->R.setIdentity();
    empty->v.setZero();
    empty->w.setZero();
    empty->dv.setZero();
    empty->dw.setZero();
    empty->q = 0.0;
    empty->dq = 0.0;
    empty->ddq = 0.0;
    empty->u = 0.0;
    empty->a.setZero();
    empty->d.setZero();
    empty->b.setZero();
    empty->Rs.setIdentity();
    empty->m = 0.0;
    empty->I.setZero();
    empty->c.setZero();
    empty->wc.setZero();
    empty->vo.setZero();
    empty->dvo.setZero();
    empty->fext.setZero();
    empty->tauext.setZero();
    empty->Jm2 = 0.0;
    empty->ulimit = 0.0;
    empty->llimit = 0.0;
    empty->uvlimit = 0.0;
    empty->lvlimit = 0.0;
    empty->defaultJointValue = 0.0;
    empty->Ir = 0.0;
    empty->gearRatio = 1.0;

    return empty;
}


void Body::updateLinkTree()
{
    nameToLinkMap.clear();
    linkTraverse_.find(rootLink());

    int n = linkTraverse_.numLinks();
    jointIdToLinkArray.clear();
    jointIdToLinkArray.resize(n, 0);
    int maxJointID = -1;
    
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        link->body = this;
        link->index = i;
        nameToLinkMap[link->name()] = link;

        int id = link->jointId;
        if(id >= 0){
            int size = jointIdToLinkArray.size();
            if(id >= size){
                jointIdToLinkArray.resize(id + 1, 0);
            }
            jointIdToLinkArray[id] = link;
            if(id > maxJointID){
                maxJointID = id;
            }
        }
    }

    jointIdToLinkArray.resize(maxJointID + 1);

    for(size_t i=0; i < jointIdToLinkArray.size(); ++i){
        if(!jointIdToLinkArray[i]){
            jointIdToLinkArray[i] = createEmptyJoint(i);
            std::cerr << "Warning: Model " << modelName_ <<
                " has empty joint ID in the valid IDs." << std::endl;
        }
    }

    calcTotalMass();

    isStaticModel_ = (rootLink_->jointType == Link::FIXED_JOINT && numJoints() == 0);
}


void Body::initializeConfiguration()
{
    rootLink_->p = defaultRootPosition;
    rootLink_->setAttitude(defaultRootAttitude);

    rootLink_->v.setZero();
    rootLink_->dv.setZero();
    rootLink_->w.setZero();
    rootLink_->dw.setZero();
    rootLink_->vo.setZero();
    rootLink_->dvo.setZero();
    
    int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        link->u = 0.0;
        link->q = 0.0;
        link->dq = 0.0;
        link->ddq = 0.0;
    }
 
    calcForwardKinematics(true, true);

    clearExternalForces();
}


void Body::resetInfo(YamlMappingPtr info)
{
    info_ = info;
    doResetInfo(*info);
}


void Body::doResetInfo(const YamlMapping& info)
{
    linkGroup_ = LinkGroup::create(this, *info.findSequence("linkGroup"));
}


/**
   This function returns a link object whose name of Joint node matches a given name.
   Null is returned when the body has no joint of the given name.
*/
Link* Body::link(const std::string& name) const
{
    NameToLinkMap::const_iterator p = nameToLinkMap.find(name);
    return (p != nameToLinkMap.end()) ? p->second : 0;
}


double Body::calcTotalMass()
{
    totalMass_ = 0.0;

    int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        totalMass_ += linkTraverse_[i]->m;
    }

    return totalMass_;
}


Vector3 Body::calcCM()
{
    totalMass_ = 0.0;
    Vector3 mc = Vector3::Zero();
    int n = linkTraverse_.numLinks();
    
    for(int i=0; i < n; i++){
        Link* link = linkTraverse_[i];
        link->wc.noalias() = link->p + link->R * link->c;
        mc.noalias() += link->m * link->wc;
        totalMass_ += link->m;
    }

    lastCM_ = mc / totalMass_;

    return lastCM_;
}


/**
   assuming Link::v,w is already computed by calcForwardKinematics(true);
   assuming Link::wc is already computed by calcCM();
*/
void Body::calcTotalMomentum(Vector3& out_P, Vector3& out_L)
{
    out_P.setZero();
    out_L.setZero();

    Vector3 dwc;    // Center of mass speed in world frame
    Vector3 P;	    // Linear momentum of the link
    Vector3 L;	    // Angular momentum with respect to the world frame origin 
    Vector3 Llocal; // Angular momentum with respect to the center of mass of the link

    int n = linkTraverse_.numLinks();
    
    for(int i=0; i < n; i++){
        Link* link = linkTraverse_[i];
        dwc = link->v + link->w.cross(link->R * link->c);
        P   = link->m * dwc;

        //L   = cross(link->wc, P) + link->R * link->I * trans(link->R) * link->w; 
        Llocal.noalias() = link->I * link->R.transpose() * link->w;
        L     .noalias() = link->wc.cross(P) + link->R * Llocal; 

        out_P += P;
        out_L += L;
    }
}


void Body::calcForwardKinematics(bool calcVelocity, bool calcAcceleration)
{
    linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
}


Sensor* Body::createSensor(Link* link, int sensorType, int id, const std::string& name)
{
    Sensor* sensor = 0;

    if(sensorType < Sensor::NUM_SENSOR_TYPES && id >= 0){

        SensorArray& sensors = allSensors[sensorType];
        int n = sensors.size();
        if(id >= n){
            sensors.resize(id + 1, 0);
        }
        sensor = sensors[id];
        if(sensor){
            std::cerr << "duplicated sensor Id is specified(id = "
                      << id << ", name = " << name << ")" << std::endl;
                
            nameToSensorMap.erase(sensor->name);
        } else {
            sensor = Sensor::create(sensorType);
        }
        if(sensor){
            sensor->id = id;
            sensors[id] = sensor;
            sensor->link = link;
            sensor->name = name;
            nameToSensorMap[name] = sensor;
        }
    }
		
    return sensor;
}

void Body::addSensor(Sensor* sensor, int sensorType, int id ){
    if(sensorType < Sensor::NUM_SENSOR_TYPES && id >= 0){
        SensorArray& sensors = allSensors[sensorType];
        int n = sensors.size();
        if(id >= n){
            sensors.resize(id + 1, 0);
        }
        Sensor* sameId = sensors[id];
        if(sameId){
            std::cerr << "duplicated sensor Id is specified(id = "
                      << id << ", name = " << sensor->name << ")" << std::endl;
                
            nameToSensorMap.erase(sameId->name);
        }
        sensors[id] = sensor;
        nameToSensorMap[sensor->name] = sensor;
    }
}


void Body::clearSensorValues()
{
    for(int i=0; i < numSensorTypes(); ++i){
        for(int j=0; j < numSensors(i); ++j){
            sensor(i, j)->clear();
        }
    }
}


JointPathPtr Body::getJointPath(Link* baseLink, Link* targetLink)
{
    if(customizerInterface && customizerInterface->initializeAnalyticIk){
        return JointPathPtr(new CustomizedJointPath(this, baseLink, targetLink));
    } else {
        return JointPathPtr(new JointPath(baseLink, targetLink));
    }
}


InverseKinematicsPtr Body::getDefaultIK(Link* targetLink)
{
    InverseKinematicsPtr ik;
    
    const YamlMapping& setupMap = *info_->findMapping("defaultIKsetup");

    if(targetLink && setupMap.isValid()){
        const YamlSequence& setup = *setupMap.findSequence(targetLink->name());
        if(setup.isValid() && !setup.empty()){
            Link* baseLink = link(setup[0].toString());
            if(baseLink){
                if(setup.size() == 1){
                    ik = getJointPath(baseLink, targetLink);
                } else {
                    CompositeIKptr compositeIK(new CompositeIK(this, targetLink));
                    ik = compositeIK;
                    for(int i=0; i < setup.size(); ++i){
                        Link* baseLink = link(setup[i].toString());
                        if(baseLink){
                            if(!compositeIK->addBaseLink(baseLink)){
                                ik.reset();
                                break;
                            }
                        }
                    }
                    /*
                    PinDragIKptr pinDragIK(new PinDragIK(this));
                    pinDragIK->setBaseLink(baseLink);
                    pinDragIK->setTargetLink(targetLink, true);
                    for(int i=1; i < setup.size(); ++i){
                        Link* pinLink = link(setup[i].toString());
                        if(pinLink){
                            pinDragIK->setPin(pinLink, PinDragIK::TRANSLATION | PinDragIK::ROTATION);
                        }
                    }
                    pinDragIK->initialize();
                    ik = pinDragIK;
                    */
                }
            }
        }
    }

    return ik;
}


void Body::setVirtualJointForces()
{
    if(customizerInterface && customizerInterface->setVirtualJointForces){
        customizerInterface->setVirtualJointForces(customizerHandle);
    }
}


void Body::clearExternalForces()
{
    int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        link->fext.setZero();
        link->tauext.setZero();
    }
}


void Body::updateLinkColdetModelPositions()
{
    const int n = linkTraverse_.numLinks();
    for(int i=0; i < n; ++i){
        Link* link = linkTraverse_[i];
        if(link->coldetModel){
            link->coldetModel->setPosition(link->segmentAttitude(), link->p);
        }
    }
}


void Body::putInformation(std::ostream &out)
{
    out << "Body: model name = " << modelName_ << " name = " << name_ << "\n\n";

    int n = numLinks();
    for(int i=0; i < n; ++i){
        out << *link(i);
    }
    out << std::endl;
}


/**
   The function installs the pre-loaded customizer corresponding to the model name.
*/
bool Body::installCustomizer()
{
    if(!pluginsInDefaultDirectoriesLoaded){
        loadBodyCustomizers(bodyInterface());
        pluginsInDefaultDirectoriesLoaded = true;
    }
		
    BodyCustomizerInterface* interface = findBodyCustomizer(modelName_);

    return interface ? installCustomizer(interface) : false;
}


bool Body::installCustomizer(BodyCustomizerInterface * customizerInterface)
{
    if(this->customizerInterface){
        if(customizerHandle){
            this->customizerInterface->destroy(customizerHandle);
            customizerHandle = 0;
        }
        this->customizerInterface = 0;
    }
	
    if(customizerInterface){
        customizerHandle = customizerInterface->create(bodyHandle, modelName_.c_str());
        if(customizerHandle){
            this->customizerInterface = customizerInterface;
        }
    }

    return (customizerHandle != 0);
}


std::ostream& operator<< (std::ostream& out, Body& body)
{
    body.putInformation(out);
    return out;
}


static inline Link* extractLink(BodyHandle bodyHandle, int linkIndex)
{
    return static_cast<BodyHandleEntity*>(bodyHandle)->body->link(linkIndex);
}


static int getLinkIndexFromName(BodyHandle bodyHandle, const char* linkName)
{
    Body* body = static_cast<BodyHandleEntity*>(bodyHandle)->body;
    Link* link = body->link(linkName);
    return (link ? link->index : -1);
}


static const char* getLinkName(BodyHandle bodyHandle, int linkIndex)
{
    return extractLink(bodyHandle, linkIndex)->name().c_str();
}


static double* getJointValuePtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle,linkIndex)->q);
}


static double* getJointVelocityPtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle, linkIndex)->dq);
}


static double* getJointTorqueForcePtr(BodyHandle bodyHandle, int linkIndex)
{
    return &(extractLink(bodyHandle, linkIndex)->u);
}


BodyInterface* Body::bodyInterface()
{
    static BodyInterface interface = {
        BODY_INTERFACE_VERSION,
        getLinkIndexFromName,
        getLinkName,
        getJointValuePtr,
        getJointVelocityPtr,
        getJointTorqueForcePtr,
    };

    return &interface;
}


CustomizedJointPath::CustomizedJointPath(Body* body, Link* baseLink, Link* targetLink) :
    JointPath(baseLink, targetLink),
    body(body)
{
    onJointPathUpdated();
}


CustomizedJointPath::~CustomizedJointPath()
{

}


void CustomizedJointPath::onJointPathUpdated()
{
    ikTypeId = body->customizerInterface->initializeAnalyticIk(
        body->customizerHandle, baseLink()->index, endLink()->index);
    if(ikTypeId){
        isCustomizedIkPathReversed = false;
    } else {
        // try reversed path
        ikTypeId = body->customizerInterface->initializeAnalyticIk(
            body->customizerHandle, endLink()->index, baseLink()->index);
        if(ikTypeId){
            isCustomizedIkPathReversed = true;
        }
    }
}


bool CustomizedJointPath::calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R)
{
    bool solved;
	
    if(ikTypeId == 0 || isBestEffortIKMode){

        solved = JointPath::calcInverseKinematics(end_p, end_R);

    } else {

#if 0
        std::vector<double> qorg(numJoints());
        for(int i=0; i < numJoints(); ++i){
            qorg[i] = joint(i)->q;
        }
#endif

        const Link* targetLink = endLink();
        const Link* baseLink_ = baseLink();

        Vector3 p_relative;
        Matrix3 R_relative;
        if(!isCustomizedIkPathReversed){
            p_relative.noalias() = baseLink_->R.transpose() * (end_p - baseLink_->p);
            R_relative.noalias() = baseLink_->R.transpose() * end_R;
        } else {
            p_relative.noalias() = end_R.transpose() * (baseLink_->p - end_p);
            R_relative.noalias() = end_R.transpose() * baseLink_->R;
        }
        solved = body->customizerInterface->
            calcAnalyticIk(body->customizerHandle, ikTypeId, p_relative, R_relative);

        if(solved){

            calcForwardKinematics();

#if 0
            double errsqr =
                (end_p - targetLink->p).squaredNorm() +
                omegaFromRot(targetLink->R.transpose() * end_R).squaredNorm();

            if(errsqr < maxIKerrorSqr){
                solved = true;
            } else {
                solved = false;
                for(int i=0; i < numJoints(); ++i){
                    joint(i)->q = qorg[i];
                }
                calcForwardKinematics();
            }
#endif
        }
    }

    return solved;
}


bool CustomizedJointPath::hasAnalyticalIK() const
{
    return (ikTypeId != 0);
}
