/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "VrmlNodes.h"

using namespace cnoid;

const char* cnoid::labelOfVrmlFieldTypeId(int type)
{
    switch(type){
    case SFINT32: return "SFInt32";
    case MFINT32: return "MFInt32";
    case SFFLOAT: return "SFFloat";
    case MFFLOAT: return "MFFloat";
    case SFVEC2F: return "SFVec3f";
    case MFVEC2F: return "MFVec2f";
    case SFVEC3F: return "SFVec3f";
    case MFVEC3F: return "MFVec3f";
    case SFROTATION: return "SFRotation";
    case MFROTATION: return "MFRotation";
    case SFTIME: return "SFTime";
    case MFTIME: return "MFTime";
    case SFCOLOR: return "SFColor";
    case MFCOLOR: return "MFColor";
    case SFSTRING: return "SFString";
    case MFSTRING: return "MFString";
    case SFNODE: return "SFNode";
    case MFNODE: return "MFNode";
    case SFBOOL: return "SFBool";
    case SFIMAGE: return "SFImage";
    default: return "Unknown Field Type";
        break;
    }
}


VrmlNode::VrmlNode()
{
    refCounter = 0;
}


VrmlNode::~VrmlNode()
{

}


bool VrmlNode::isCategoryOf(VrmlNodeCategory category)
{
    return (category == ANY_NODE) ? true : categorySet.test(category);
}



VrmlUnsupportedNode::VrmlUnsupportedNode(const std::string& nodeTypeName) :
    nodeTypeName(nodeTypeName)
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
}


VrmlViewpoint::VrmlViewpoint()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
    
    fieldOfView = 0.785398;
    jump = true;
    orientation.axis() = SFVec3f::UnitZ();
    orientation.angle() = 0.0;
    position << 0.0, 0.0, 10.0;
}


VrmlNavigationInfo::VrmlNavigationInfo() :
    avatarSize(3)
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);

    avatarSize[0] = 0.25;
    avatarSize[1] = 1.6;
    avatarSize[2] = 0.75;

    headlight = true;
    speed = 1.0;
    visibilityLimit = 0.0;

    type.push_back("WALK");
}


VrmlBackground::VrmlBackground()
{
    categorySet.set(TOP_NODE);
    categorySet.set(BINDABLE_NODE);
    categorySet.set(CHILD_NODE);
}


AbstractVrmlGroup::AbstractVrmlGroup()
{
    categorySet.set(TOP_NODE);
    categorySet.set(GROUPING_NODE);
    categorySet.set(CHILD_NODE);
}


void AbstractVrmlGroup::removeChild(int childIndex)
{
    replaceChild(childIndex, 0);
}


VrmlGroup::VrmlGroup()
{
    bboxCenter.setZero();
    bboxSize.fill(-1.0);
}


MFNode& VrmlGroup::getChildren()
{
    return children;
}


int VrmlGroup::countChildren()
{
    return children.size();
}


VrmlNode* VrmlGroup::getChild(int index)
{
    return children[index].get();
}


void VrmlGroup::replaceChild(int childIndex, VrmlNode* childNode)
{
    if(!childNode){
        children.erase(children.begin() + childIndex);
    } else {
        children[childIndex] = childNode;
    }
}


VrmlTransform::VrmlTransform()
{
    center.setZero();
    rotation.axis() = SFVec3f::UnitZ();
    rotation.angle() = 0.0;
    scale.setOnes();
    scaleOrientation.axis() = SFVec3f::UnitZ();
    scaleOrientation.angle() = 0.0;
    translation.setZero();
}


Eigen::Affine3d VrmlTransform::toAffine3d()
{
    const Eigen::Translation3d C(center);
    const SFRotation& SR = scaleOrientation;
    const Eigen::AlignedScaling3d S(scale);
    const SFRotation& R = rotation;
    const Eigen::Translation3d T(translation);

    return T * C * R * SR * S * SR.inverse() * C.inverse();
}


VrmlInline::VrmlInline()
{
    categorySet.set(INLINE_NODE);
}


VrmlShape::VrmlShape()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);
    categorySet.set(SHAPE_NODE);
}


VrmlAppearance::VrmlAppearance()
{
    categorySet.set(APPEARANCE_NODE);
}


VrmlMaterial::VrmlMaterial()
{
    categorySet.set(MATERIAL_NODE);

    diffuseColor << 0.0f, 0.0f, 0.8f;
    emissiveColor.setZero();
    specularColor.setZero();
    ambientIntensity = 0.2;
    shininess = 0.2;
    transparency = 0.0;
}


VrmlTexture::VrmlTexture()
{
    categorySet.set(TEXTURE_NODE);
}


VrmlImageTexture::VrmlImageTexture()
{
    repeatS = true; 
    repeatT = true; 
}


VrmlTextureTransform::VrmlTextureTransform()
{
    categorySet.set(TEXTURE_TRANSFORM_NODE);

    center.setZero();
    scale.setOnes();
    translation.setZero();
    rotation = 0.0;
}


VrmlGeometry::VrmlGeometry()
{
    categorySet.set(GEOMETRY_NODE);
}


VrmlBox::VrmlBox()
{
    size.fill(2.0);
}


VrmlCone::VrmlCone()
{
    bottom = true;
    bottomRadius = 1.0;
    height = 2.0;
    side = true;
}


VrmlCylinder::VrmlCylinder()
{
    height = 2.0;
    radius = 1.0;
    bottom = true;
    side = true;
    top = true;
}


VrmlSphere::VrmlSphere()
{
    radius = 1.0; 
}


VrmlFontStyle::VrmlFontStyle()
{
    categorySet.set(FONT_STYLE_NODE);
  
    family.push_back("SERIF");
    horizontal = true;
    justify.push_back("BEGIN");
    leftToRight = true;
    size = 1.0;
    spacing = 1.0;
    style = "PLAIN";
    topToBottom = true;
}


VrmlText::VrmlText()
{
    maxExtent = 0.0;
}


VrmlIndexedLineSet::VrmlIndexedLineSet()
{
    colorPerVertex = true;
}


VrmlIndexedFaceSet::VrmlIndexedFaceSet()
{
    ccw = true;
    convex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
}


VrmlColor::VrmlColor()
{
    categorySet.set(COLOR_NODE);
}


VrmlCoordinate::VrmlCoordinate()
{
    categorySet.set(COORDINATE_NODE);
}


VrmlTextureCoordinate::VrmlTextureCoordinate()
{
    categorySet.set(TEXTURE_COORDINATE_NODE);
}


VrmlNormal::VrmlNormal()
{
    categorySet.set(NORMAL_NODE);
}


VrmlCylinderSensor::VrmlCylinderSensor()
{
    categorySet.set(CHILD_NODE);
    categorySet.set(SENSOR_NODE);
  
    autoOffset = true;
    diskAngle = 0.262;
    enabled = true;
    maxAngle = -1;
    minAngle = 0;
    offset = 0;
}


VrmlPointSet::VrmlPointSet()
{
    coord = NULL;
    color = NULL;
}


VrmlPixelTexture::VrmlPixelTexture()
{
    image.width  = 0;
    image.height = 0;
    image.numComponents = 0;
    image.pixels.clear();
	
    repeatS = true;
    repeatT = true;
}


VrmlMovieTexture::VrmlMovieTexture()
{
    // url
    loop = false;
    speed = 0;
    startTime = 0.0;
    stopTime = 0.0;
    repeatS = true;
    repeatT = true;
}


VrmlElevationGrid::VrmlElevationGrid()
{
    xDimension = 0;
    zDimension = 0;
    xSpacing = 0.0;
    zSpacing = 0.0;
    // height	// MFFloat
    ccw = true;
    colorPerVertex = true;
    creaseAngle = 0.0;
    normalPerVertex = true;
    solid = true;
    color = NULL;
    normal = NULL;
    texCoord = NULL;
}


VrmlExtrusion::VrmlExtrusion()
{
    // crossSection
    // spine
    beginCap = true;
    endCap   = true;
    solid    = true;
    ccw      = true;
    convex   = true;
    creaseAngle = 0;
    orientation.push_back(SFRotation(0.0, SFVec3f::UnitZ()));
    scale.push_back(SFVec2f(1.0, 1.0));
}


VrmlSwitch::VrmlSwitch()
{
    whichChoice = -1;
}


MFNode& VrmlSwitch::getChildren()
{
    return choice;
}


int VrmlSwitch::countChildren()
{
    return choice.size();
}


VrmlNode* VrmlSwitch::getChild(int index)
{
    return choice[index].get();
}


void VrmlSwitch::replaceChild(int childIndex, VrmlNode* childNode)
{
    if(!childNode){
        choice.erase(choice.begin() + childIndex);
        if(whichChoice == childIndex){
            whichChoice = -1;
        } else if(whichChoice > childIndex){
            whichChoice -= 1;
        }
    } else {
        choice[childIndex] = childNode;
    }
}


VrmlLOD::VrmlLOD()
{
    center.setZero();
}


MFNode& VrmlLOD::getChildren()
{
    return level;
}


int VrmlLOD::countChildren()
{
    return level.size();
}


VrmlNode* VrmlLOD::getChild(int index)
{
    return level[index].get();
}


void VrmlLOD::replaceChild(int childIndex, VrmlNode* childNode)
{
    if(!childNode){
        level.erase(level.begin() + childIndex);
        if(!level.empty()){
            int rangeIndexToRemove = (childIndex > 0) ? (childIndex - 1) : 0;
            range.erase(range.begin() + rangeIndexToRemove);
        }
    } else {
        level[childIndex] = childNode;
    }
}


VrmlCollision::VrmlCollision()
{
    collide = true;
}


VrmlAnchor::VrmlAnchor()
{

}


VrmlBillboard::VrmlBillboard()
{
    axisOfRotation.setZero();
}


VrmlFog::VrmlFog()
{
    color.setZero();
    visibilityRange = 0.0;
    fogType = "LINEAR";
}


VrmlWorldInfo::VrmlWorldInfo()
{
    categorySet.set(TOP_NODE);
}


VrmlPointLight::VrmlPointLight()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);

    location.setZero();
    on = true;
    intensity = 1.0;
    color.setOnes();
    radius = 100.0;
    ambientIntensity = 0.0;
    attenuation << 1.0, 0.0, 0.0;
}


VrmlDirectionalLight::VrmlDirectionalLight()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);

    ambientIntensity = 0.0;
    color.setOnes();
    direction << 0.0, 0.0, -1.0;
    intensity = 1.0;
    on = true;
}


VrmlSpotLight::VrmlSpotLight()
{
    categorySet.set(TOP_NODE);
    categorySet.set(CHILD_NODE);

    location.setZero();
    direction << 0.0, 0.0, -1.0;
    on = true;
    color.setOnes();
    intensity = 1.0;
    radius = 100.0;
    ambientIntensity = 0.0;
    attenuation << 1.0, 0.0, 0.0;
    beamWidth = 1.570796;
    cutOffAngle = 0.785398;
}


VrmlProto::VrmlProto(const std::string& n) : protoName(n)
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_DEF_NODE);
}


VrmlProtoInstance::VrmlProtoInstance(VrmlProtoPtr proto0) :
    proto(proto0),
    fields(proto0->fields) 
{
    categorySet.set(TOP_NODE);
    categorySet.set(PROTO_INSTANCE_NODE);;
    categorySet.set(CHILD_NODE);
}
