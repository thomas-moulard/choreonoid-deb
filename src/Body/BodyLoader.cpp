/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyLoader.h"
#include "Link.h"
#include "Sensor.h"
#include "ModelNodeSet.h"
#include <cnoid/LeggedBody>
#include <cnoid/EigenTypes>
#include <cnoid/YamlReader>
#include <cnoid/TriangleMeshShaper>
#include <cnoid/ColdetModel>
#include <Eigen/StdVector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

    class BodyLoaderImpl
    {
    public:
        BodyLoaderImpl();
        ~BodyLoaderImpl();

        void setDivisionNumber(int n);
        
        BodyPtr loadModelFile(
            const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel);

        ModelNodeSetPtr modelNodeSet;
        TriangleMeshShaper triangleMeshShaper;
        BodyPtr body;
        bool createColdetModel;
        string errorMessage;
        YamlMappingPtr info;

        boost::signal<void(const std::string& message)> sigMessage;

        typedef map<string, Sensor::SensorType> SensorTypeMap;
        static SensorTypeMap sensorTypeMap;

        struct MeshInfo
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            MeshInfo(VrmlIndexedFaceSet* triangleMesh, const Affine3& TX) :
                triangleMesh(triangleMesh), TX(TX) { }
            VrmlIndexedFaceSet* triangleMesh;
            Affine3 TX;
        };
        vector<MeshInfo, Eigen::aligned_allocator<MeshInfo> > meshes;
        int numTotalVertices;
        int numTotalTriangles;

        void createBodyFromModelNodeSet();
        Link* createLink(JointNodeSetPtr jointNodeSet, const Matrix3& parentRs);
        void setJointProperties(Link* link, VrmlProtoInstancePtr jointNode, const Matrix3& parentRs);
        void setSegmentProperties(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, const JointNodeSet::Affine3Array& transforms);
        void createSensors(Link* link, std::vector<VrmlProtoInstancePtr>& sensorNodes);
        void setColdetModel(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, const JointNodeSet::Affine3Array& transforms);
        bool collectMeshes(VrmlShape* shapeNode, const Affine3& T);
        void extractShapeNodes(VrmlNode* node, const function<bool(VrmlShape* shapeNode, const Affine3& T)>& callback);
        bool extractShapeNodeTraverse(
            VrmlNode* node, const Affine3& T, const function<bool(VrmlShape* shapeNode, const Affine3& T)>& callback);
    };
}


BodyLoaderImpl::SensorTypeMap BodyLoaderImpl::sensorTypeMap;


namespace {

    double getLimitValue(VrmlVariantField& field, double defaultValue)
    {
        MFFloat& values = get<MFFloat>(field);
        if(values.empty()){
            return defaultValue;
        }
        return values[0];
    }


    void copyVrmlField(VrmlVariantField& field, string& out_s)
    {
        switch(field.which()){
        case SFSTRING:
            out_s = get<SFString>(field);
            break;
        case MFSTRING:
        {
            MFString& strings = get<MFString>(field);
            out_s = "";
            for(size_t i=0; i < strings.size(); i++){
                out_s += strings[i] + "\n";
            }
        }
        break;
        default:
            break;
        }
    }


    void copyVrmlField(VrmlVariantField& field, int& out_value)
    {
        out_value = get<SFInt32>(field);
    }


    void copyVrmlField(VrmlVariantField& field, double& out_value)
    {
        out_value = get<SFFloat>(field);
    }


    void copyVrmlField(VrmlVariantField& field, Vector3& out_v)
    {
        out_v = get<SFVec3f>(field);
    }


    void copyVrmlField(VrmlVariantField& field, Matrix3& out_R)
    {
        if(field.which() == SFROTATION){
            out_R = get<SFRotation>(field).toRotationMatrix();

        } else if(field.which() == MFFLOAT){
            MFFloat& mf = get<MFFloat>(field);
            if(mf.size() >= 9){
                out_R <<
                    mf[0], mf[1], mf[2],
                    mf[3], mf[4], mf[5],
                    mf[6], mf[7], mf[8];
            }
        }
    }
}


BodyLoader::BodyLoader()
{
    impl = new BodyLoaderImpl();
}


BodyLoaderImpl::BodyLoaderImpl()
{
    if(sensorTypeMap.empty()){
	sensorTypeMap["ForceSensor"]        = Sensor::FORCE;
	sensorTypeMap["Gyro"]               = Sensor::RATE_GYRO;
	sensorTypeMap["AccelerationSensor"] = Sensor::ACCELERATION;
	sensorTypeMap["PressureSensor"]     = Sensor::PRESSURE;
	sensorTypeMap["PhotoInterrupter"]   = Sensor::PHOTO_INTERRUPTER;
	sensorTypeMap["VisionSensor"]       = Sensor::VISION;
	sensorTypeMap["TorqueSensor"]       = Sensor::TORQUE;
    }

    triangleMeshShaper.sigMessage.connect(sigMessage);
}


BodyLoader::~BodyLoader()
{
    delete impl;
}


BodyLoaderImpl::~BodyLoaderImpl()
{

}


ModelNodeSetPtr BodyLoader::modelNodeSet()
{
    return impl->modelNodeSet;
}


const std::string& BodyLoader::errorMessage()
{
    return impl->errorMessage;
}


SignalProxy< boost::signal<void(const std::string& message)> > BodyLoader::sigMessage()
{
    return impl->sigMessage;
}


/**
   The function for setting the division number of primitive geometries such as
   cone, cylinder and sphere.
*/
void BodyLoader::setDivisionNumber(int n)
{
    impl->setDivisionNumber(n);
}


void BodyLoaderImpl::setDivisionNumber(int n)
{
    triangleMeshShaper.setDivisionNumber(std::max(4, n));
}


BodyPtr BodyLoader::loadModelFile
(const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel)
{
    return impl->loadModelFile(filename, doTriangulation, doNormalGeneration, createColdetModel);
}


BodyPtr BodyLoaderImpl::loadModelFile
(const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel)
{
    body = 0;
    info = 0;
    errorMessage.clear();
    
    filesystem::path orgpath(filename);
    string ext = filesystem::extension(orgpath);
    
    string vrmlFile;

    try {
        if(ext == ".wrl"){
            vrmlFile = filename;
        } else if(ext == ".yaml"){
            YamlReader parser;
            if(parser.load(filename)){
                info = parser.document()->toMapping();
                filesystem::path vrmlFilePath(info->get("modelFile"));
                if(vrmlFilePath.has_root_path()){
                    vrmlFile = vrmlFilePath.file_string();
                } else {
                    vrmlFile = (orgpath.branch_path() / vrmlFilePath).file_string();
                }
            }
        }

        if(!vrmlFile.empty()){

            if(info){
                int divisionNumber;
                if(info->read("divisionNumberOfPrimitiveGeometries", divisionNumber)){
                    setDivisionNumber(divisionNumber);
                }
            }

            modelNodeSet.reset(new ModelNodeSet());
            
            if(modelNodeSet->loadModelFile(vrmlFile)){
                if(doTriangulation || createColdetModel){
                    triangleMeshShaper.setNormalGenerationMode(doNormalGeneration);
                    triangleMeshShaper.apply(modelNodeSet->humanoidNode());
                }
                this->createColdetModel = createColdetModel;

                createBodyFromModelNodeSet();
            }
        }
        
    } catch(const YamlNode::Exception& ex){
        errorMessage = ex.message();
    } catch(ModelNodeSet::Exception& ex){
        errorMessage = ex.what();
    }
    
    return body;
}


void BodyLoaderImpl::createBodyFromModelNodeSet()
{
    JointNodeSetPtr rootJointNodeSet = modelNodeSet->rootJointNodeSet();

    if(rootJointNodeSet){

        if(info){
            if(LeggedBody::checkBodyInfoAsLeggedBody(info)){
                body = new LeggedBody();
            }
        }
        if(!body){
            body = new Body();
        }
            
	body->setModelName(modelNodeSet->humanoidNode()->defName);
    
	Matrix3 Rs = Matrix3::Identity();
	Link* rootLink = createLink(rootJointNodeSet, Rs);
	body->setRootLink(rootLink);

	TProtoFieldMap& f = rootJointNodeSet->jointNode->fields;

	Vector3 defaultRootPos;
	copyVrmlField(f["translation"], defaultRootPos);
	Matrix3 defaultRootR;
	copyVrmlField(f["rotation"], defaultRootR);
	
	body->setDefaultRootPosition(defaultRootPos, defaultRootR);

	body->installCustomizer();
	body->initializeConfiguration();

        if(info){
            body->resetInfo(info);
        }
    }
}


Link* BodyLoaderImpl::createLink(JointNodeSetPtr jointNodeSet, const Matrix3& parentRs)
{
    Link* link = new Link();

    setJointProperties(link, jointNodeSet->jointNode, parentRs);

    vector<VrmlProtoInstancePtr> segmentNodes = jointNodeSet->segmentNodes;

    const JointNodeSet::Affine3Array& transforms = jointNodeSet->transforms;

    setSegmentProperties(link, segmentNodes, transforms);

    if(createColdetModel && !segmentNodes.empty()){
        setColdetModel(link, segmentNodes, transforms);
    }
    
    // The following code adds child links from the back of the child array
    // in order to keep the original order of the children.
    // ( addChild() function of the Link class prepends a child to the child list )
    int numChildren = jointNodeSet->childJointNodeSets.size();
    for(int i = numChildren - 1; i >= 0; --i){
	JointNodeSetPtr childJointNodeSet = jointNodeSet->childJointNodeSets[i];
	Link* childLink = createLink(childJointNodeSet, link->Rs);
	link->addChild(childLink);
    }
    
    createSensors(link, jointNodeSet->sensorNodes);

    return link;
}


void BodyLoaderImpl::setJointProperties(Link* link, VrmlProtoInstancePtr jointNode, const Matrix3& parentRs)
{
    link->setName(jointNode->defName);

    TProtoFieldMap& jf = jointNode->fields;

    copyVrmlField(jf["jointId"], link->jointId);

    Vector3 b;
    copyVrmlField(jf["translation"], b);
    link->b.noalias() = parentRs * b;

    Matrix3 R;
    copyVrmlField(jf["rotation"], R);
    link->Rs.noalias() = parentRs * R;

    Vector3 jointAxis = Vector3::Zero();
    VrmlVariantField& jointAxisField = jf["jointAxis"];
    switch(jointAxisField.which()){
    case SFSTRING:
    {
        SFString& axisLabel = get<SFString>(jointAxisField);
        if(axisLabel == "X"){
            jointAxis = Vector3::UnitX();
        } else if(axisLabel == "Y"){
            jointAxis = Vector3::UnitY();
        } else if(axisLabel == "Z"){
            jointAxis = Vector3::UnitZ();
        }
    }
    break;
    case SFVEC3F:
	copyVrmlField(jointAxisField, jointAxis);
	break;
    default:
	break;
    }

    string jointType;
    copyVrmlField(jf["jointType"], jointType);
    
    if(jointType == "fixed" ){
	link->jointType = Link::FIXED_JOINT;
    } else if(jointType == "free" ){
	link->jointType = Link::FREE_JOINT;
    } else if(jointType == "rotate" ){
	link->jointType = Link::ROTATIONAL_JOINT;
    } else if(jointType == "slide" ){
	link->jointType = Link::SLIDE_JOINT;
    } else {
	link->jointType = Link::FREE_JOINT;
    }

    link->a.setZero();
    link->d.setZero();

    if(link->jointType == Link::ROTATIONAL_JOINT){
	link->a.noalias() = link->Rs * jointAxis;
    } else if(link->jointType == Link::SLIDE_JOINT){
	link->d.noalias() = link->Rs * jointAxis;
    }

    copyVrmlField(jf["jointValue"], link->defaultJointValue);

    copyVrmlField(jf["rotorInertia"], link->Ir);
    copyVrmlField(jf["gearRatio"], link->gearRatio);
    copyVrmlField(jf["torqueConst"], link->torqueConst);
    copyVrmlField(jf["encoderPulse"], link->encoderPulse);
    copyVrmlField(jf["rotorResistor"], link->rotorResistor);

    VrmlVariantField* field = jointNode->findField("equivalentInertia");
    if(field){
	link->Jm2 = get<SFFloat>(*field);
    } else {
	link->Jm2 = link->gearRatio * link->gearRatio * link->Ir;
    }    

    double max = numeric_limits<double>::max();

    link->ulimit  = getLimitValue(jf["ulimit"],  +max);
    link->llimit  = getLimitValue(jf["llimit"],  -max);
    link->uvlimit = getLimitValue(jf["uvlimit"], +max);
    link->lvlimit = getLimitValue(jf["lvlimit"], -max);
}    


void BodyLoaderImpl::setSegmentProperties
(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, const JointNodeSet::Affine3Array& transforms)
{
    int numSegment = segmentNodes.size();
    link->m = 0.0;

    //  Mass = Σmass                 //
    //  C = (Σmass * T * c) / Mass   //
    //  I = Σ(R * I * Rt + G)       //
    //  R = Tの回転行列               //
    //  G = y*y+z*z, -x*y, -x*z, -y*x, z*z+x*x, -y*z, -z*x, -z*y, x*x+y*y    //
    //  (x, y, z ) = T * c - C        //
    std::vector<Vector3> centerOfMassArray;
    std::vector<double> massArray;
    Vector3 linkc = Vector3::Zero();
    Matrix3 linkI = Matrix3::Zero();
    for(int i = 0 ; i < numSegment ; ++i){
        const Affine3& T = transforms[i];
        TProtoFieldMap& sf = segmentNodes[i]->fields;
        Vector3 c;
        copyVrmlField(sf["centerOfMass"], c);
        double m;
        copyVrmlField(sf["mass"], m);
        Matrix3 I;
        copyVrmlField(sf["momentsOfInertia"], I);

        //Matrix3 R(T.linear());
        const Vector3 c1 = T.linear() * c + T.translation();
        centerOfMassArray.push_back(c1);
        massArray.push_back(m);
        linkc = c1 * m + linkc * link->m;
        link->m += m;
        linkc /= link->m;
        linkI.noalias() += T.linear() * I * T.linear().transpose();
    }

    for(int i = 0 ; i < numSegment ; ++i){
        const Vector3& c = centerOfMassArray[i];
        double x = c(0) - linkc(0);
        double y = c(1) - linkc(1);
        double z = c(2) - linkc(2);
        double m = massArray[i];

        linkI(0,0) += m * (y*y + z*z);
        linkI(0,1) += -m * x * y;
        linkI(0,2) += -m * x * z;
        linkI(1,0) += -m * y * x;
        linkI(1,1) += m * (z*z + x*x);
        linkI(1,2) += -m * y * z;
        linkI(2,0) += -m * z * x;
        linkI(2,1) += -m * z * y;
        linkI(2,2) += m * (x*x + y*y);
    }

    link->c.noalias() = link->Rs * linkc;
    link->I.noalias() = link->Rs * linkI * link->Rs.transpose();
}
    

void BodyLoaderImpl::createSensors(Link* link, std::vector<VrmlProtoInstancePtr>& sensorNodes)
{
    int numSensors = sensorNodes.size();

    for(int i=0; i < numSensors; ++i){

	VrmlProtoInstancePtr sensorNode = sensorNodes[i];
	TProtoFieldMap& f = sensorNode->fields;

	const string& name = sensorNode->defName;
	int id;
	copyVrmlField(f["sensorId"], id);

	if(id < 0){
	    throw ModelNodeSet::Exception
		(string("sensor ID is not given to sensor ") + name + "of model " + body->modelName());
	} else {

	    int sensorType = Sensor::COMMON;
	    
	    SensorTypeMap::iterator p = sensorTypeMap.find(sensorNode->proto->protoName);
	    if(p != sensorTypeMap.end()){
		sensorType = p->second;
	    } else {
		throw ModelNodeSet::Exception("Unknown type sensor node");
	    }
	    
	    Sensor* sensor = body->createSensor(link, sensorType, id, name);

	    if(sensor){

		Vector3 p;
		copyVrmlField(f["translation"], p);
		sensor->localPos.noalias() = link->Rs * p;

		Matrix3 R;
		copyVrmlField(f["rotation"], R);
		sensor->localR.noalias() = link->Rs * R;
	    }
	}
    }
}


void BodyLoaderImpl::setColdetModel
(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, const JointNodeSet::Affine3Array& transforms)
{
    meshes.clear();
    numTotalVertices = 0;
    numTotalTriangles = 0;

    int numSegment = segmentNodes.size();
    for(int i = 0 ; i < numSegment ; ++i){
        extractShapeNodeTraverse(segmentNodes[i].get(), transforms[i], bind(&BodyLoaderImpl::collectMeshes, this, _1, _2));
    }

    ColdetModelPtr coldetModel(new ColdetModel());

    coldetModel->setNumVertices(numTotalVertices);
    coldetModel->setNumTriangles(numTotalTriangles);

    int vertexIndex = 0;
    int triangleIndex = 0;
    
    for(size_t i=0; i < meshes.size(); ++i){

        int vertexTop = vertexIndex;

        Affine3 T = Affine3(link->Rs) * meshes[i].TX;
        
        const MFVec3f& vertices = meshes[i].triangleMesh->coord->point;
        size_t numVertices = vertices.size();
        
        for(size_t j=0; j < numVertices; ++j){
            Vector3 v = T * vertices[j];
            coldetModel->setVertex(vertexIndex++, (float)v[0], (float)v[1], (float)v[2]);
        }

        const MFInt32& indices = meshes[i].triangleMesh->coordIndex;
        const size_t numTriangles = indices.size() / 4;

        for(size_t j=0; j < numTriangles; ++j){
            coldetModel->setTriangle(
                triangleIndex++, vertexTop + indices[j*4], vertexTop + indices[j*4+1], vertexTop + indices[j*4+2]);
        }
    }
    
    coldetModel->setName(link->name());
    coldetModel->build();
    link->coldetModel = coldetModel;
}


bool BodyLoaderImpl::collectMeshes(VrmlShape* shapeNode, const Affine3& T)
{
    VrmlIndexedFaceSet* triangleMesh = dynamic_node_cast<VrmlIndexedFaceSet>(shapeNode->geometry).get();
    if(triangleMesh){
        meshes.push_back(MeshInfo(triangleMesh, T));
        numTotalVertices += triangleMesh->coord->point.size();
        numTotalTriangles += triangleMesh->coordIndex.size() / 4;
    }

    return true;
}


/**
   @todo move this code into the ModelNodeSet class ?
*/

bool BodyLoaderImpl::extractShapeNodeTraverse
(VrmlNode* node, const Affine3& T, const function<bool(VrmlShape* shapeNode, const Affine3& T)>& callback)
{
    bool doContinue = true;
    
    if(node->isCategoryOf(PROTO_INSTANCE_NODE)){
        VrmlProtoInstance* protoInstance = static_cast<VrmlProtoInstance*>(node);
        if(protoInstance->actualNode){
            doContinue = extractShapeNodeTraverse(protoInstance->actualNode.get(), T, callback);
        }

    } else if(node->isCategoryOf(GROUPING_NODE)) {

        VrmlGroup* groupNode = static_cast<VrmlGroup*>(node);
        
        Affine3 T2;
        VrmlTransform* transformNode = dynamic_cast<VrmlTransform*>(groupNode);
        if(transformNode){
            T2 = T * transformNode->toAffine3d();
        }

        VrmlSwitch* switchNode = dynamic_cast<VrmlSwitch*>(node);
        if(switchNode){
            int whichChoice = switchNode->whichChoice;
            if(whichChoice >= 0 && whichChoice < switchNode->countChildren()){
                extractShapeNodeTraverse(switchNode->getChild(whichChoice), (transformNode ? T2 : T), callback);
            }
        } else {
            for(int i=0; i < groupNode->countChildren(); ++i){
                if(!extractShapeNodeTraverse(groupNode->getChild(i), (transformNode ? T2 : T), callback)){
                    doContinue = false;
                    break;
                }
            }
        }

    } else if(node->isCategoryOf(SHAPE_NODE)) {
        doContinue = callback(static_cast<VrmlShape*>(node), T);
    }

    return doContinue;
}
