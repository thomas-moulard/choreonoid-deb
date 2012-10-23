/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRMLNODES_H_INCLUDED
#define CNOID_UTIL_VRMLNODES_H_INCLUDED

#include <map>
#include <string>
#include <bitset>
#include <boost/variant.hpp>
#include <boost/intrusive_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "exportdecl.h"

namespace cnoid {

    typedef bool SFBool;
    typedef int  SFInt32;
    typedef double SFFloat;
    typedef std::string SFString;

    // Define SFTime as struct to clearly distinguish its type from SFFloat
    struct SFTime {
        double value;
        inline SFTime() { value = 0.0; }
        inline SFTime(double time) { value = time; }
        inline double operator=(double time) { return (value = time); }
    };

    typedef Eigen::Vector2d SFVec2f;
    typedef Eigen::Vector3d SFVec3f;
    typedef Eigen::Array3f SFColor;
    typedef Eigen::AngleAxisd SFRotation;

    typedef struct {
        int width;
        int height;
        int numComponents;
        std::vector<unsigned char> pixels;
    } SFImage;

    typedef std::vector<SFInt32> MFInt32;
    typedef std::vector<SFFloat> MFFloat;
    typedef std::vector<SFVec2f, Eigen::aligned_allocator<SFVec2f> > MFVec2f;
    typedef std::vector<SFVec3f> MFVec3f;
    typedef std::vector<SFRotation> MFRotation;
    typedef std::vector<SFTime> MFTime;
    typedef std::vector<SFColor> MFColor;
    typedef std::vector<SFString> MFString;

    // see 4.6.3 - 4.6.10 of the VRML97 specification
    enum VrmlNodeCategory {

        ANY_NODE = -1,

        PROTO_DEF_NODE = 0,
        PROTO_INSTANCE_NODE,

        TOP_NODE,
        BINDABLE_NODE,
        GROUPING_NODE,
        CHILD_NODE,

        APPEARANCE_NODE,
        MATERIAL_NODE,
        TEXTURE_NODE,
        TEXTURE_TRANSFORM_NODE,

        SHAPE_NODE,
        GEOMETRY_NODE,
        COORDINATE_NODE,
        COLOR_NODE,
        NORMAL_NODE,
        TEXTURE_COORDINATE_NODE,

        FONT_STYLE_NODE,

        SENSOR_NODE,
        INLINE_NODE,

        NUM_VRML_NODE_CATEGORIES
    };

    class VrmlNode;

    inline void intrusive_ptr_add_ref(VrmlNode* obj);
    inline void intrusive_ptr_release(VrmlNode* obj);

    //! Abstract base class of all vrml nodes.
    class CNOID_EXPORT VrmlNode
    {
      public:

	VrmlNode();
	virtual ~VrmlNode();

	std::string defName;

	bool isCategoryOf(VrmlNodeCategory category);

      protected:
	std::bitset<NUM_VRML_NODE_CATEGORIES> categorySet;

      private:
	int refCounter;

	friend void intrusive_ptr_add_ref(VrmlNode* obj);
	friend void intrusive_ptr_release(VrmlNode* obj);
    };

    inline void intrusive_ptr_add_ref(VrmlNode* obj){
	obj->refCounter++;
    }
    
    inline void intrusive_ptr_release(VrmlNode* obj){
	obj->refCounter--;
	if(obj->refCounter <= 0){
	    delete obj;
	}
    }

    typedef boost::intrusive_ptr<VrmlNode> VrmlNodePtr;

    typedef VrmlNodePtr SFNode;
    typedef std::vector<SFNode> MFNode;


    class CNOID_EXPORT  VrmlUnsupportedNode : public VrmlNode
    {
      public:
        VrmlUnsupportedNode(const std::string& nodeTypeName);
        std::string nodeTypeName;
    };
    typedef boost::intrusive_ptr<VrmlUnsupportedNode> VrmlUnsupportedNodePtr;


    //! VRML Viewpoint node
    class CNOID_EXPORT  VrmlViewpoint : public VrmlNode
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        VrmlViewpoint();

	SFRotation orientation;
	SFFloat fieldOfView;
	SFBool jump;
	SFVec3f position;
	SFString description;
    };
    typedef boost::intrusive_ptr<VrmlViewpoint> VrmlViewpointPtr;


    //! VRML NavigationInfo node
    class CNOID_EXPORT  VrmlNavigationInfo : public VrmlNode
    {
      public:
	VrmlNavigationInfo();

	MFFloat avatarSize;
	SFBool headlight;
	SFFloat speed;
	MFString type;
	SFFloat visibilityLimit;
    };
    typedef boost::intrusive_ptr<VrmlNavigationInfo> VrmlNavigationInfoPtr;


    //! VRML Background node
    class CNOID_EXPORT  VrmlBackground : public VrmlNode
    {
      public:
        VrmlBackground();
      
        MFFloat groundAngle;
        MFColor groundColor;
        MFFloat skyAngle;
        MFColor skyColor;
        MFString backUrl;
        MFString bottomUrl;
        MFString frontUrl;
        MFString leftUrl;
        MFString rightUrl;
        MFString topUrl;
    };
    typedef boost::intrusive_ptr<VrmlBackground> VrmlBackgroundPtr;


    class CNOID_EXPORT  AbstractVrmlGroup : public VrmlNode
    {
      public:
	AbstractVrmlGroup();
        
        virtual MFNode& getChildren() = 0;
        virtual int countChildren() = 0;
        virtual VrmlNode* getChild(int index) = 0;
        virtual void replaceChild(int childIndex, VrmlNode* childNode) = 0;
        
        void removeChild(int childIndex);
    };
    typedef boost::intrusive_ptr<AbstractVrmlGroup> AbstractVrmlGroupPtr;
    
    
    //! VRML Group node
    class CNOID_EXPORT VrmlGroup : public AbstractVrmlGroup
    {
      public:
	VrmlGroup();

        virtual MFNode& getChildren();
        virtual int countChildren();
        virtual VrmlNode* getChild(int index);
        virtual void replaceChild(int childIndex, VrmlNode* childNode);

	SFVec3f bboxCenter;    
	SFVec3f bboxSize;  
	MFNode children;
    };
    typedef boost::intrusive_ptr<VrmlGroup> VrmlGroupPtr;


    //! VRML Transform node
    class CNOID_EXPORT  VrmlTransform : public VrmlGroup
    {
      public:
	VrmlTransform();

        Eigen::Affine3d toAffine3d();

	SFVec3f center;
	SFRotation rotation;
	SFVec3f scale;
	SFRotation scaleOrientation;
	SFVec3f translation;
    };
    typedef boost::intrusive_ptr<VrmlTransform> VrmlTransformPtr;

    //! VRML Inline node
    class CNOID_EXPORT  VrmlInline : public VrmlGroup
    {
      public:
        VrmlInline();
        MFString urls;
    };
    typedef boost::intrusive_ptr<VrmlInline> VrmlInlinePtr;


    class VrmlAppearance;
    typedef boost::intrusive_ptr<VrmlAppearance> VrmlAppearancePtr;

    class VrmlGeometry;
    typedef boost::intrusive_ptr<VrmlGeometry> VrmlGeometryPtr;


    //! VRML Shape node
    class CNOID_EXPORT  VrmlShape : public VrmlNode
    {
      public:
        VrmlShape();
        VrmlAppearancePtr appearance;
        SFNode geometry;
    };
    typedef boost::intrusive_ptr<VrmlShape> VrmlShapePtr;


    class VrmlMaterial;
    typedef boost::intrusive_ptr<VrmlMaterial> VrmlMaterialPtr;

    class VrmlTexture;
    typedef boost::intrusive_ptr<VrmlTexture> VrmlTexturePtr;

    class VrmlTextureTransform;
    typedef boost::intrusive_ptr<VrmlTextureTransform> VrmlTextureTransformPtr;

    //! VRML Appearance node
    class CNOID_EXPORT VrmlAppearance : public VrmlNode
    {
      public:
        VrmlAppearance();
      
        VrmlMaterialPtr material;
        VrmlTexturePtr texture;
        VrmlTextureTransformPtr textureTransform;
    };


    //! VRML Material node
    class CNOID_EXPORT VrmlMaterial : public VrmlNode
    {
      public:
	VrmlMaterial();

	SFFloat ambientIntensity;
	SFColor diffuseColor;
	SFColor emissiveColor;
	SFFloat shininess;
	SFColor specularColor;
	SFFloat transparency;
    };


    //! Base class of VRML Texture nodes
    class CNOID_EXPORT VrmlTexture : public VrmlNode
    {
      public:
        VrmlTexture();
    };

    
    //! VRML ImageTexture node
    class CNOID_EXPORT VrmlImageTexture : public VrmlTexture
    {
      public:
	VrmlImageTexture();

	MFString url;
	SFBool   repeatS;
	SFBool   repeatT;
    };
    typedef boost::intrusive_ptr<VrmlImageTexture> VrmlImageTexturePtr;


    //! VRML TextureTransform node
    class CNOID_EXPORT VrmlTextureTransform : public VrmlNode
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
	VrmlTextureTransform();

	SFVec2f center;
	SFFloat rotation;
	SFVec2f scale;
	SFVec2f translation;
    };

    //! Base class of VRML geometry nodes
    class CNOID_EXPORT VrmlGeometry : public VrmlNode
    {
      public:
        VrmlGeometry();
    };

    //! VRML Box node
    class CNOID_EXPORT VrmlBox : public VrmlGeometry
    {
      public:
	VrmlBox();
	SFVec3f size;
    };
    typedef boost::intrusive_ptr<VrmlBox> VrmlBoxPtr;


    //! VRML Cone node
    class CNOID_EXPORT VrmlCone : public VrmlGeometry
    {
      public:
	VrmlCone();

	SFBool bottom;
	SFFloat bottomRadius;
	SFFloat height;
	SFBool side;
    };
    typedef boost::intrusive_ptr<VrmlCone> VrmlConePtr;


    //! VRML Cylinder node
    class CNOID_EXPORT VrmlCylinder : public VrmlGeometry
    {
      public:
	VrmlCylinder();

	SFBool bottom;
	SFFloat height;
	SFFloat radius;
	SFBool side;
	SFBool top;
    };
    typedef boost::intrusive_ptr<VrmlCylinder> VrmlCylinderPtr;


    //! VRML Sphere node
    class CNOID_EXPORT VrmlSphere : public VrmlGeometry
    {
      public:
	VrmlSphere();
	SFFloat radius;
    };
    typedef boost::intrusive_ptr<VrmlSphere> VrmlSpherePtr;


    //! VRML FontStyle node
    class CNOID_EXPORT VrmlFontStyle : public VrmlNode
    {
      public:
	VrmlFontStyle();

	MFString family;       
	SFBool   horizontal;
	MFString justify;
	SFString language;
	SFBool   leftToRight;
	SFFloat  size;
	SFFloat  spacing;
	SFString style;
	SFBool   topToBottom;
    };
    typedef boost::intrusive_ptr<VrmlFontStyle> VrmlFontStylePtr;


    //! VRML Text node
    class CNOID_EXPORT VrmlText : public VrmlGeometry
    {
      public:
	VrmlText();

	MFString fstring;
	VrmlFontStylePtr fontStyle;
	MFFloat length;
	SFFloat maxExtent;
    };
    typedef boost::intrusive_ptr<VrmlText> VrmlTextPtr;


    class VrmlColor;
    typedef boost::intrusive_ptr<VrmlColor> VrmlColorPtr;

    class VrmlCoordinate;
    typedef boost::intrusive_ptr<VrmlCoordinate> VrmlCoordinatePtr;

    //! VRML IndexedLineSet node
    class CNOID_EXPORT VrmlIndexedLineSet : public VrmlGeometry
    {
      public: 
	VrmlIndexedLineSet();

	VrmlColorPtr color;
	VrmlCoordinatePtr coord;
	MFInt32 colorIndex;
	SFBool colorPerVertex;
	MFInt32 coordIndex;
    };
    typedef boost::intrusive_ptr<VrmlIndexedLineSet> VrmlIndexedLineSetPtr;


    class VrmlNormal;
    typedef boost::intrusive_ptr<VrmlNormal> VrmlNormalPtr;

    class VrmlTextureCoordinate;
    typedef boost::intrusive_ptr<VrmlTextureCoordinate> VrmlTextureCoordinatePtr;


    //! VRML IndexedFaseSet node
    class CNOID_EXPORT VrmlIndexedFaceSet : public VrmlIndexedLineSet
    {
      public:
	VrmlIndexedFaceSet();

	VrmlNormalPtr normal;
	VrmlTextureCoordinatePtr texCoord;
	SFBool ccw;
	SFBool convex;
	SFFloat creaseAngle;
	MFInt32 normalIndex;
	SFBool normalPerVertex;  
	SFBool solid;
	MFInt32 texCoordIndex;
    };
    typedef boost::intrusive_ptr<VrmlIndexedFaceSet> VrmlIndexedFaceSetPtr;


    //! VRML Color node
    class CNOID_EXPORT VrmlColor : public VrmlNode
    {
      public:
        VrmlColor();
      
        MFColor color;
    };


    //! VRML Coordinate node
    class CNOID_EXPORT VrmlCoordinate : public VrmlNode
    {
      public:
        VrmlCoordinate();
        MFVec3f point;
    };


    //! VRML TextureCoordinate node
    class CNOID_EXPORT VrmlTextureCoordinate : public VrmlNode
    {
      public:
        VrmlTextureCoordinate();
        MFVec2f point;
    };


    //! VRML Normal node
    class CNOID_EXPORT VrmlNormal : public VrmlNode
    {
      public:
        VrmlNormal();
        MFVec3f vector;
    };


    //! VRML CylinderSensor node
    class CNOID_EXPORT VrmlCylinderSensor : public VrmlNode
    {
      public:
	VrmlCylinderSensor();

	SFBool  autoOffset;
	SFFloat diskAngle;
	SFBool  enabled;
	SFFloat maxAngle;
	SFFloat minAngle;
	SFFloat offset;
    };
    typedef boost::intrusive_ptr<VrmlCylinderSensor> VrmlCylinderSensorPtr;


    //! VRML PointSet node
    class CNOID_EXPORT VrmlPointSet : public VrmlGeometry
    {
      public:
	VrmlPointSet();

	VrmlCoordinatePtr	coord;
	VrmlColorPtr		color;
    };

    typedef boost::intrusive_ptr<VrmlPointSet> VrmlPointSetPtr;


    //! VRML PixelTexture node
    class CNOID_EXPORT VrmlPixelTexture : public VrmlTexture
    {
      public:
	VrmlPixelTexture();

	SFImage			image;
	SFBool			repeatS;
	SFBool			repeatT;
    };

    typedef boost::intrusive_ptr<VrmlPixelTexture> VrmlPixelTexturePtr;


    //! VRML MovieTexture node
    class CNOID_EXPORT VrmlMovieTexture : public VrmlTexture
    {
      public:
	VrmlMovieTexture();

	MFString		url;
	SFBool			loop;
	SFFloat			speed;
	SFTime			startTime;
	SFTime			stopTime;
	SFBool			repeatS;
	SFBool			repeatT;
    };

    typedef boost::intrusive_ptr<VrmlMovieTexture> VrmlMovieTexturePtr;


    //! VRML ElevationGrid node
    class CNOID_EXPORT VrmlElevationGrid : public VrmlGeometry
    {
      public:
	VrmlElevationGrid();

	SFInt32			xDimension;
	SFInt32			zDimension;
	SFFloat			xSpacing;
	SFFloat			zSpacing;
	MFFloat			height;
	SFBool			ccw;
	SFBool			colorPerVertex;
	SFFloat			creaseAngle;
	SFBool			normalPerVertex;
	SFBool			solid;
	VrmlColorPtr	color;
	VrmlNormalPtr	normal;
	VrmlTextureCoordinatePtr	texCoord;
    };

    typedef boost::intrusive_ptr<VrmlElevationGrid> VrmlElevationGridPtr;


    //! VRML Extrusion node
    class CNOID_EXPORT VrmlExtrusion : public VrmlGeometry
    {
      public:
	VrmlExtrusion();

	MFVec2f			crossSection;
	MFVec3f			spine;
	MFVec2f			scale;
	MFRotation		orientation;
	SFBool			beginCap;
	SFBool			endCap;
	SFBool			solid;
	SFBool			ccw;
	SFBool			convex;
	SFFloat			creaseAngle;
    };

    typedef boost::intrusive_ptr<VrmlExtrusion> VrmlExtrusionPtr;


    class CNOID_EXPORT VrmlSwitch : public AbstractVrmlGroup
    {
      public:
	VrmlSwitch();

        virtual MFNode& getChildren();
        virtual int countChildren();
        virtual VrmlNode* getChild(int index);
        virtual void replaceChild(int childIndex, VrmlNode* childNode);

        MFNode	choice;
	SFInt32	whichChoice;
    };

    typedef boost::intrusive_ptr<VrmlSwitch> VrmlSwitchPtr;


    class CNOID_EXPORT VrmlLOD : public AbstractVrmlGroup
    {
      public:
	VrmlLOD();

        virtual MFNode& getChildren();
        virtual int countChildren();
        virtual VrmlNode* getChild(int index);
        virtual void replaceChild(int childIndex, VrmlNode* childNode);

	MFFloat range;
	SFVec3f center;
	MFNode  level;
    };

    typedef boost::intrusive_ptr<VrmlLOD> VrmlLODPtr;


    class CNOID_EXPORT VrmlCollision : public VrmlGroup
    {
      public:
	VrmlCollision();
	SFBool collide;
	SFNode proxy;
    };

    typedef boost::intrusive_ptr<VrmlCollision> VrmlCollisionPtr;


    class CNOID_EXPORT VrmlAnchor : public VrmlGroup
    {
      public:
	VrmlAnchor();
	SFString description;
	MFString parameter;
	MFString url;
    };

    typedef boost::intrusive_ptr<VrmlAnchor> VrmlAnchorPtr;


    class CNOID_EXPORT VrmlBillboard : public VrmlGroup
    {
      public:
	VrmlBillboard();
	SFVec3f axisOfRotation;
    };

    typedef boost::intrusive_ptr<VrmlBillboard> VrmlBillboardPtr;


    class CNOID_EXPORT VrmlFog : public VrmlNode
    {
      public:
	VrmlFog();
	SFColor  color;
	SFFloat  visibilityRange;
	SFString fogType;
    };

    typedef boost::intrusive_ptr<VrmlFog> VrmlFogPtr;


    class CNOID_EXPORT  VrmlWorldInfo : public VrmlNode
    {
      public:
	VrmlWorldInfo();
	SFString title;
	MFString info;
    };

    typedef boost::intrusive_ptr<VrmlWorldInfo> VrmlWorldInfoPtr;


    class CNOID_EXPORT VrmlPointLight : public VrmlNode
    {
      public:
	VrmlPointLight();
	SFVec3f location;
	SFBool  on;
	SFFloat intensity;
	SFColor color;
	SFFloat radius;
	SFFloat ambientIntensity;
	SFVec3f attenuation;
    };

    typedef boost::intrusive_ptr<VrmlPointLight> VrmlPointLightPtr;


    class CNOID_EXPORT VrmlDirectionalLight : public VrmlNode
    {
      public:
	VrmlDirectionalLight();
	SFFloat ambientIntensity;
	SFColor color;
	SFVec3f direction;
	SFFloat intensity;
	SFBool  on;
    };

    typedef boost::intrusive_ptr<VrmlDirectionalLight> VrmlDirectionalLightPtr;


    class CNOID_EXPORT VrmlSpotLight : public VrmlNode
    {
      public:
	VrmlSpotLight();
	SFVec3f location;
	SFVec3f direction;
	SFBool  on;
	SFColor color;
	SFFloat intensity;
	SFFloat radius;
	SFFloat ambientIntensity;
	SFVec3f attenuation;
	SFFloat beamWidth;
	SFFloat cutOffAngle;
    };

    typedef boost::intrusive_ptr<VrmlSpotLight> VrmlSpotLightPtr;

    typedef boost::variant<SFBool,
                           SFInt32, SFFloat, SFVec2f, SFVec3f, SFRotation, SFColor, SFTime, SFString, SFNode, SFImage,
                           MFInt32, MFFloat, MFVec2f, MFVec3f, MFRotation, MFColor, MFTime, MFString, MFNode> VrmlVariantField;

    enum VrmlFieldTypeId {
        SFBOOL,
        SFINT32, SFFLOAT, SFVEC2F, SFVEC3F, SFROTATION, SFCOLOR, SFTIME, SFSTRING, SFNODE, SFIMAGE,
        MFINT32, MFFLOAT, MFVEC2F, MFVEC3F, MFROTATION, MFCOLOR, MFTIME, MFSTRING, MFNODE,
        UNKNOWN_VRML_FIELD_TYPE
    };
    
    typedef std::map <std::string, VrmlVariantField> TProtoFieldMap;
    typedef std::pair<std::string, VrmlVariantField> TProtoFieldPair;

    template<typename TValue> int vrmlFieldTypeId() { return UNKNOWN_VRML_FIELD_TYPE; }
    template<> inline int vrmlFieldTypeId<SFInt32>() { return SFINT32; }
    template<> inline int vrmlFieldTypeId<MFInt32>() { return MFINT32; }
    template<> inline int vrmlFieldTypeId<SFFloat>() { return SFFLOAT; }
    template<> inline int vrmlFieldTypeId<MFFloat>() { return MFFLOAT; }
    template<> inline int vrmlFieldTypeId<SFVec3f>() { return SFVEC3F; }
    template<> inline int vrmlFieldTypeId<MFVec3f>() { return MFVEC3F; }
    template<> inline int vrmlFieldTypeId<SFRotation>() { return SFROTATION; }
    template<> inline int vrmlFieldTypeId<MFRotation>() { return MFROTATION; }
    template<> inline int vrmlFieldTypeId<SFTime>() { return SFTIME; }
    template<> inline int vrmlFieldTypeId<MFTime>() { return MFTIME; }
    template<> inline int vrmlFieldTypeId<SFColor>() { return SFCOLOR; }
    template<> inline int vrmlFieldTypeId<MFColor>() { return MFCOLOR; }
    template<> inline int vrmlFieldTypeId<SFString>() { return SFSTRING; }
    template<> inline int vrmlFieldTypeId<MFString>() { return MFSTRING; }
    template<> inline int vrmlFieldTypeId<SFNode>() { return SFNODE; }
    template<> inline int vrmlFieldTypeId<MFNode>() { return MFNODE; }
    template<> inline int vrmlFieldTypeId<SFImage>() { return SFIMAGE; }

    CNOID_EXPORT const char* labelOfVrmlFieldTypeId(int type);

    template<typename TValue> inline const char* labelOfVrmlFieldType() {
        return labelOfVrmlFieldTypeId(vrmlFieldTypeId<TValue>());
    }

    //! VRML Proto definition
    class CNOID_EXPORT VrmlProto : public VrmlNode
    {
      public:
	std::string protoName;
	TProtoFieldMap fields;

	VrmlProto(const std::string& n);

	inline VrmlVariantField* findField(const std::string& fieldName) {
	    TProtoFieldMap::iterator p = fields.find(fieldName);
	    return (p != fields.end()) ? &p->second : 0;
	}

        inline VrmlVariantField& field(const std::string& fieldName){
	    return fields[fieldName];
        }

        /*
	inline VrmlVariantField& addField(const std::string& fieldName, VrmlFieldTypeId typeId){
	    VrmlVariantField* field = &(fields[fieldName]);
	    field->setType(typeId);
	    return field;
	}
        */

    };
    typedef boost::intrusive_ptr<VrmlProto> VrmlProtoPtr;


    //! VRML node which is instance of VRML Prototype
    class CNOID_EXPORT VrmlProtoInstance : public VrmlNode
    {
      public:
        VrmlProtoPtr proto;
        TProtoFieldMap fields;
        VrmlNodePtr actualNode;

	VrmlProtoInstance(VrmlProtoPtr proto0);

	inline VrmlVariantField* findField(const std::string& fieldName) {
	    TProtoFieldMap::iterator p = fields.find(fieldName);
	    return (p != fields.end()) ? &p->second : 0;
	} 
		
    };
    typedef boost::intrusive_ptr<VrmlProtoInstance> VrmlProtoInstancePtr;

    /**
       The upper cast operation that supports the situation where the original pointer
       is VrmlProtoInstance and you want to get the actual node,
       the node replaced with the pre-defined node type written in the PROTO definition.
    */
    template<class VrmlNodeType>
    inline boost::intrusive_ptr<VrmlNodeType> dynamic_node_cast(VrmlNodePtr node) {
        VrmlProtoInstancePtr protoInstance = boost::dynamic_pointer_cast<VrmlProtoInstance>(node);
        if(protoInstance){
            return boost::dynamic_pointer_cast<VrmlNodeType>(protoInstance->actualNode);
        } else {
            return boost::dynamic_pointer_cast<VrmlNodeType>(node);
        }
    }

};

#endif
