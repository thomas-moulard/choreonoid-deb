/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ModelNodeSet.h"
#include <bitset>
#include <iostream>
#include <algorithm>
#include <boost/format.hpp>
#include <cnoid/EasyScanner>
#include <cnoid/VrmlParser>

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {

    typedef void (ModelNodeSetImpl::*ProtoCheckFunc)(void);
    
    struct ProtoInfo
    {
	ProtoInfo() { }
	ProtoInfo(int id, ProtoCheckFunc func) : id(id), protoCheckFunc(func) { }
	int id;
	ProtoCheckFunc protoCheckFunc;
    };
    
    typedef map<string, ProtoInfo> ProtoNameToInfoMap;
    ProtoNameToInfoMap protoNameToInfoMap;
}


namespace cnoid {

    class ModelNodeSetImpl
    {
    public:
        ModelNodeSetImpl(ModelNodeSet* self);

        bool loadModelFile(const std::string& filename);

        ModelNodeSet* self;
        
        int numJointNodes;
        VrmlProtoInstancePtr humanoidNode;
        JointNodeSetPtr rootJointNodeSet;
        int messageIndent;

        VrmlProtoPtr protoToCheck;

        enum {
            PROTO_UNDEFINED = 0,
            PROTO_HUMANOID,
            PROTO_JOINT,
            PROTO_SEGMENT,
            PROTO_SENSOR,
            PROTO_HARDWARECOMPONENT,
            NUM_PROTOS
        };

        typedef std::bitset<NUM_PROTOS> ProtoIdSet;
    
        void extractHumanoidNode(VrmlParser& parser);

        void throwExceptionOfIllegalField(const std::string& name, const char* label);

        template <typename TValue>
        VrmlVariantField* addField(const std::string& name, const TValue& defaultValue) {
            VrmlVariantField* field = protoToCheck->findField(name);
            if(!field){
                field = &protoToCheck->field(name);
                (*field) = defaultValue;
            } else if(field->type() != typeid(TValue)){
                throwExceptionOfIllegalField(name, labelOfVrmlFieldType<TValue>());
            }
            return field;
        }

        template <typename TValue>
        VrmlVariantField* addField(const std::string& name) {
            return addField(name, TValue());
        }

        template <typename TValue>
        void requireField(const std::string& name){
            VrmlVariantField* field = protoToCheck->findField(name);
            if(!field || field->type() != typeid(TValue)){
                throwExceptionOfIllegalField(name, labelOfVrmlFieldType<TValue>());
            }
        }
        
        void checkHumanoidProto();
        void checkJointProto();
        void checkSegmentProto();
        void checkSensorProtoCommon();
        void checkHardwareComponentProto();
        void extractJointNodes();
        JointNodeSetPtr addJointNodeSet(VrmlProtoInstancePtr jointNode);
        void extractChildNodes
            (JointNodeSetPtr jointNodeSet, MFNode& childNodes, const ProtoIdSet acceptableProtoIds, const Affine3& T);

        void putMessage(const std::string& message);
    };
}


ModelNodeSet::ModelNodeSet()
{
    impl = new ModelNodeSetImpl(this);
}


ModelNodeSetImpl::ModelNodeSetImpl(ModelNodeSet* self) : self(self)
{
    numJointNodes = 0;
    messageIndent = 0;

    if(protoNameToInfoMap.empty()){

        protoNameToInfoMap["Humanoid"]
            = ProtoInfo(PROTO_HUMANOID, &ModelNodeSetImpl::checkHumanoidProto);

        protoNameToInfoMap["Joint"]
            = ProtoInfo(PROTO_JOINT, &ModelNodeSetImpl::checkJointProto);
        
        protoNameToInfoMap["Segment"]
            = ProtoInfo(PROTO_SEGMENT, &ModelNodeSetImpl::checkSegmentProto);
        
        protoNameToInfoMap["ForceSensor"]
            = ProtoInfo(PROTO_SENSOR, &ModelNodeSetImpl::checkSensorProtoCommon);
        
        protoNameToInfoMap["Gyro"]
            = ProtoInfo(PROTO_SENSOR, &ModelNodeSetImpl::checkSensorProtoCommon);
        
        protoNameToInfoMap["AccelerationSensor"]
            = ProtoInfo(PROTO_SENSOR, &ModelNodeSetImpl::checkSensorProtoCommon);
        
        protoNameToInfoMap["VisionSensor"]
            = ProtoInfo(PROTO_SENSOR, &ModelNodeSetImpl::checkSensorProtoCommon);
        
        protoNameToInfoMap["RangeSensor"]
            = ProtoInfo(PROTO_SENSOR, &ModelNodeSetImpl::checkSensorProtoCommon);
        
        protoNameToInfoMap["HardwareComponent"]
            = ProtoInfo(PROTO_HARDWARECOMPONENT, &ModelNodeSetImpl::checkHardwareComponentProto);
    }
}


ModelNodeSet::~ModelNodeSet()
{
    delete impl;
}


int ModelNodeSet::numJointNodes()
{
    return impl->numJointNodes;
}


VrmlProtoInstancePtr ModelNodeSet::humanoidNode()
{
    return impl->humanoidNode;
}


JointNodeSetPtr ModelNodeSet::rootJointNodeSet()
{
    return impl->rootJointNodeSet;
}


bool ModelNodeSet::loadModelFile(const std::string& filename)
{
    return impl->loadModelFile(filename);
}


bool ModelNodeSetImpl::loadModelFile(const std::string& filename)
{
    numJointNodes = 0;
    humanoidNode = 0;
    rootJointNodeSet.reset();
    messageIndent = 0;

    try {
	VrmlParser parser;
	parser.load(filename);
	extractHumanoidNode(parser);

    } catch(EasyScanner::Exception& ex){
	    throw ModelNodeSet::Exception(ex.getFullMessage());
    }

    return (humanoidNode && rootJointNodeSet);
}


void ModelNodeSetImpl::extractHumanoidNode(VrmlParser& parser)
{
    while(VrmlNodePtr node = parser.readNode()){
		
        if(node->isCategoryOf(PROTO_DEF_NODE)){
			
            protoToCheck = static_pointer_cast<VrmlProto>(node);
			
            ProtoNameToInfoMap::iterator p = protoNameToInfoMap.find(protoToCheck->protoName);
            if(p != protoNameToInfoMap.end()){
                ProtoInfo& info = p->second;
                (this->*info.protoCheckFunc)();
            }
			
        } else if(node->isCategoryOf(PROTO_INSTANCE_NODE)){
			
            VrmlProtoInstancePtr instance = static_pointer_cast<VrmlProtoInstance>(node);
            if(instance->proto->protoName == "Humanoid") {
                humanoidNode = instance;
            }
        }
    }
	
    if(humanoidNode){
        putMessage("Humanoid node");
        extractJointNodes();
    } else {
        throw ModelNodeSet::Exception("Humanoid node is not found");
    }
}


void ModelNodeSetImpl::throwExceptionOfIllegalField(const std::string& name, const char* label)
{
    format message("Proto \"%1%\" must have the \"%2%\" field of %3% type");
    throw ModelNodeSet::Exception(str(message % protoToCheck->protoName % name % label));
}


void ModelNodeSetImpl::checkHumanoidProto()
{
    // necessary fields
    requireField<SFVec3f>("center");
    requireField<MFNode>("humanoidBody");
    requireField<SFRotation>("rotation");
    requireField<SFVec3f>("translation");

    // optional fields
    addField<MFString>("info");
    addField<SFString>("name");
    addField<SFString>("version");
    addField<SFRotation>("scaleOrientation");
    addField<SFVec3f>("scale", SFVec3f::Constant(1.0));
}


void ModelNodeSetImpl::checkJointProto()
{
    // necessary fields
    requireField<SFVec3f>("center");
    requireField<MFNode>("children");
    requireField<SFRotation>("rotation");
    requireField<SFVec3f>("translation");
    requireField<SFString>("jointType");
    requireField<SFInt32>("jointId");

    VrmlVariantField* field;

    field = protoToCheck->findField("jointAxis");
    if(!field){
        throw ModelNodeSet::Exception
            ("Prototype of Humanoid must have the \"jointAxis\" field");
    }
    if(field->type() != typeid(SFString) && field->type() != typeid(SFVec3f)){
        throw ModelNodeSet::Exception
            ("The type of \"jointAxis\" field in \"Humanoid\" prototype must be SFString or SFVec3f");
    }

    // optional fields
    addField<MFFloat>("llimit");
    addField<MFFloat>("ulimit");
    addField<MFFloat>("lvlimit");
    addField<MFFloat>("uvlimit");
    addField<SFRotation>("limitOrientation");
    addField<SFString>("name");

    addField<SFFloat>("gearRatio", 1.0);
    addField<SFFloat>("rotorInertia", 0.0);
    addField<SFFloat>("rotorResistor", 0.0);
    addField<SFFloat>("torqueConst", 1.0);
    addField<SFFloat>("encoderPulse", 1.0);

    addField<SFFloat>("jointValue", 0.0);
    addField<SFVec3f>("scale", SFVec3f::Constant(1.0));

    if(protoToCheck->findField("equivalentInertia")){
        putMessage("The \"equivalentInertia\" field of the Joint node is obsolete.");
    }
}


void ModelNodeSetImpl::checkSegmentProto()
{
    requireField<SFVec3f>("centerOfMass");
    requireField<SFFloat>("mass");
    requireField<MFFloat>("momentsOfInertia");
    addField<SFString>("name");
}


void ModelNodeSetImpl::checkSensorProtoCommon()
{
    requireField<SFInt32>("sensorId");
    requireField<SFVec3f>("translation");
    requireField<SFRotation>("rotation");
}


void ModelNodeSetImpl::checkHardwareComponentProto()
{
    requireField<SFInt32>("id");
    requireField<SFVec3f>("translation");
    requireField<SFRotation>("rotation");
    requireField<SFString>("url");
}


void ModelNodeSetImpl::extractJointNodes()
{
    MFNode& nodes = get<MFNode>(humanoidNode->fields["humanoidBody"]);

    if(nodes.size() > 1){
        throw ModelNodeSet::Exception
            ("The Humanoid node must have a unique Joint node in its \"humanoidBody\" field.");

    } else if(nodes.size() == 1){
        if(nodes[0]->isCategoryOf(PROTO_INSTANCE_NODE)){
	    VrmlProtoInstancePtr jointNode = dynamic_pointer_cast<VrmlProtoInstance>(nodes[0]);
	    if(jointNode && jointNode->proto->protoName == "Joint"){
                rootJointNodeSet = addJointNodeSet(jointNode);
	    }
	}
    }

    if(!rootJointNodeSet){
        throw ModelNodeSet::Exception
            ("The Humanoid node does not have a Joint node in its \"humanoidBody\" field.");
    }
}


JointNodeSetPtr ModelNodeSetImpl::addJointNodeSet(VrmlProtoInstancePtr jointNode)
{
    numJointNodes++;

    putMessage(string("Joint node") + jointNode->defName);

    JointNodeSetPtr jointNodeSet(new JointNodeSet());

    jointNodeSet->jointNode = jointNode;

    MFNode& childNodes = get<MFNode>(jointNode->fields["children"]);

    ProtoIdSet acceptableProtoIds;
    acceptableProtoIds.set(PROTO_JOINT);
    acceptableProtoIds.set(PROTO_SEGMENT);
    acceptableProtoIds.set(PROTO_SENSOR);
    acceptableProtoIds.set(PROTO_HARDWARECOMPONENT);
    
    Affine3 T(Affine3::Identity());
    extractChildNodes(jointNodeSet, childNodes, acceptableProtoIds, T);

    return jointNodeSet;
}

void ModelNodeSetImpl::extractChildNodes
(JointNodeSetPtr jointNodeSet, MFNode& childNodes, const ProtoIdSet acceptableProtoIds, const Affine3& T)
{
    for(size_t i = 0; i < childNodes.size(); i++){
        VrmlNode* childNode = childNodes[i].get();
        const Affine3* pT;
        if(childNode->isCategoryOf(GROUPING_NODE)){
            VrmlGroup* groupNode = static_cast<VrmlGroup*>(childNode);
            VrmlTransform* transformNode = dynamic_cast<VrmlTransform*>(groupNode);
            Affine3 T2;
            if(transformNode){
                T2 = T * transformNode->toAffine3d();
                pT = &T2;
            } else {
                pT = &T;
            }
            extractChildNodes(jointNodeSet, groupNode->getChildren(), acceptableProtoIds, *pT);

        } else if(childNode->isCategoryOf(PROTO_INSTANCE_NODE)){

            VrmlProtoInstance* protoInstance = static_cast<VrmlProtoInstance*>(childNode);
            int id = PROTO_UNDEFINED;
            bool doTraverseChildren = false;
            ProtoIdSet acceptableChildProtoIds(acceptableProtoIds);

            const string& protoName = protoInstance->proto->protoName;
            ProtoNameToInfoMap::iterator p = protoNameToInfoMap.find(protoName);

            if(p == protoNameToInfoMap.end()){
                doTraverseChildren = true;
            } else {
                id = p->second.id;
                if(!acceptableProtoIds.test(id)){
                    throw ModelNodeSet::Exception(protoName + " node is not in a correct place.");
                }
            }

            messageIndent += 2;

            switch(id){
                
            case PROTO_JOINT:
                if(T.matrix() != Affine3::MatrixType::Identity()){
                    throw ModelNodeSet::Exception(protoName + " node is not in a correct place.");
                }
                jointNodeSet->childJointNodeSets.push_back(addJointNodeSet(protoInstance));
                break;
                
            case PROTO_SENSOR:
                if(T.matrix() != Affine3::MatrixType::Identity()){
                    throw ModelNodeSet::Exception(protoName + " node is not in a correct place.");
                }
                jointNodeSet->sensorNodes.push_back(protoInstance);
                putMessage(protoName + protoInstance->defName);
                break;
                
            case PROTO_HARDWARECOMPONENT:
                if(T.matrix() != Affine3::MatrixType::Identity()){
                    throw ModelNodeSet::Exception(protoName + " node is not in a correct place.");
                }
                jointNodeSet->hwcNodes.push_back(protoInstance);
                putMessage(protoName + protoInstance->defName);
                break;
                
            case PROTO_SEGMENT:
                {
                    jointNodeSet->segmentNodes.push_back(protoInstance);
                    jointNodeSet->transforms.push_back(T);
                    putMessage(string("Segment node ") + protoInstance->defName);

                    doTraverseChildren = true;
                    acceptableChildProtoIds.reset();
                    acceptableChildProtoIds.set(PROTO_SENSOR);
                }
                break;
                
            default:
                break;
            }

            if(doTraverseChildren){
                MFNode& childNodes = get<MFNode>(protoInstance->fields["children"]);
                extractChildNodes(jointNodeSet, childNodes, acceptableChildProtoIds, T);
            }

            messageIndent -= 2;
        }
    }
}


void ModelNodeSetImpl::putMessage(const std::string& message)
{
    if(!self->sigMessage.empty()) {
        string space(messageIndent, ' ');
        self->sigMessage(space + message + "\n");
    }
}
