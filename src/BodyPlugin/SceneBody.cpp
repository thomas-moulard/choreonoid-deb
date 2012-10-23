/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBody.h"
#include "OsgCollision.h"
#include "KinematicsBar.h"
#include "BodyBar.h"
#include "LinkSelectionView.h"
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/VrmlToOsgConverter>
#include <cnoid/OsgOutlineFx>
#include <cnoid/OsgNormalVisualizer>
#include <cnoid/MenuManager>
#include <cnoid/ConnectionSet>
#include <cnoid/ScenePieces>
#include <cnoid/LeggedBody>
#include <cnoid/PinDragIK>
#include <cnoid/PenetrationBlocker>
#include <osg/Version>
#include <osg/Material>
#include <osg/BlendFunc> 
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osgManipulator/Projector>
#include <osgGA/GUIEventAdapter>
#include <osgFX/Scribe>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using osgGA::GUIEventAdapter;

namespace {

    const bool TRACE_FUNCTIONS = false;

#if OPENSCENEGRAPH_SOVERSION >= 55
    typedef osg::Vec3d Vec3ForProjection;
#else
    typedef osg::Vec3 Vec3ForProjection;
#endif

    class SceneLink : public osg::MatrixTransform
    {
    public:
        SceneLink();
        SceneLink(BodyPtr& body, JointNodeSetPtr jointNodeSet, VrmlToOsgConverter& vrmlToOsgConverter);

        void showMarker(const osg::Vec4 color, float width);
        void hideMarker();

        Link* link;
        osg::ref_ptr<osg::Node> shapeNode;
        osg::ref_ptr<BBMarker> marker;
        osg::ref_ptr<AttitudeDragger> attitudeDragger;
        osg::ref_ptr<OsgOutlineFx> outline;
        osg::ref_ptr<osgFX::Scribe> scribe;
        bool isValid;
        bool isPointed;
        bool isColliding;
    };
    
}

namespace cnoid {

    class SceneBodyImpl
    {
    public:
        SceneBodyImpl(SceneBody* self, BodyItemPtr bodyItem);
        ~SceneBodyImpl();

        SceneBody* self;
        BodyItemPtr bodyItem;
        BodyPtr body;
            
        ConnectionSet connections;
        signals::connection connectionToSigWorldCollisionLinkSetChanged;
            
        std::vector< osg::ref_ptr<SceneLink> > sceneLinks;

        osg::Group* visibleSceneLinks;

        osg::Group* markerGroup;

        SceneLink* outlinedLink;

        osg::ref_ptr<OsgCollision> osgCollision;

        osg::ref_ptr<CrossMarker> cmMarker;
        bool isCmVisible;

        osg::ref_ptr<SphereMarker> zmpMarker;
        bool isZmpVisible;
        Vector3 orgZmpPos;

        enum PointedType { PT_NONE, PT_SCENE_LINK, PT_ZMP };
        SceneLink* pointedSceneLink;

        Link* targetLink;
        Vector3 orgPointerPos;
        Vector3 orgBaseLinkPos;
        Vector3 orgTargetLinkPos;
        double orgJointPosition;
        Vector3 rotationBaseX;
        Vector3 rotationBaseY;
        Matrix3 orgAttitude;
        Vector3 axis;
            
        JointPathPtr ikPath;
        LinkTraverse fkTraverse;
        PinDragIKptr pinDragIK;
        InverseKinematicsPtr ik;
        PenetrationBlockerPtr penetrationBlocker;
        osg::ref_ptr<AttitudeDragger> attitudeDragger;

        //vector<Pose::LinkInfo*> snapFootLinkInfos;
        //bool isSnapping;
        //Matrix33 footAttitudeBeforeSnap;

        enum Axis { NOAXIS = -1, X = 0, Y = 1, Z = 2 };
        Axis rotationAxis;
            
        enum DragMode {
            DRAG_NONE, LINK_IK_TRANSLATION, LINK_IK_ROTATION, LINK_FK_ROTATION, ZMP_TRANSLATION
        };

        DragMode dragMode;
        bool isDragging;
            
        //osg::ref_ptr<osgManipulator::PlaneProjector> planeProjector;
        //osg::ref_ptr<osgManipulator::CylinderProjector> cylinderProjector;
        osg::ref_ptr<osgManipulator::Projector> projector;
        osgManipulator::PointerInfo pointerInfo;
            
        KinematicsBar* kinematicsBar;
        std::ostream& os;

        bool createSceneLinks();
        void createSceneLinksSub(JointNodeSetPtr jointNodeSet, VrmlToOsgConverter& vrmlToOsgConverter);
        void onAttachedToScene();
        void onDetachedFromScene();
        void onKinematicStateChanged();

        void createOutline(SceneLink* sceneLink);
        bool setPointedLinkVisual(SceneLink* sceneLink, bool on);
        bool setWorldCollisionLinkVisual(SceneLink* sceneLink, bool on);
        void createScribe(SceneLink* sceneLink);
        bool setSelfCollisionLinkVisual(SceneLink* sceneLink, bool on);
            
        void onSelfCollisionsUpdated();
        void onWorldCollisionLinkSetChanged();
        void onCollisionLinkSetChanged();
        void onCollisionLinkHighlightModeChanged();
        void changeCollisionLinkHighlightMode(bool on);
        void setLinkVisibilities(const boost::dynamic_bitset<>& visibilities);
        void showCenterOfMass(bool on);
        void showZmp(bool on);
        PointedType findPointedObject(const osg::NodePath& path);
        void setOutlineForPointedLink(SceneLink* sceneLink);
        void updateMarkersAndManipulators();
        void makeLinkFree(SceneLink* sceneLink);
        void setBaseLink(SceneLink* sceneLink);
        void toggleBaseLink(SceneLink* sceneLink);
        void togglePin(SceneLink* sceneLink, bool toggleTranslation, bool toggleRotation);
        void showTargetLinkManipulators(bool on);

        bool onKeyPressEvent(const SceneViewEvent& event);
        bool onKeyReleaseEvent(const SceneViewEvent& event);
        bool onButtonPressEvent(const SceneViewEvent& event);
        bool onButtonReleaseEvent(const SceneViewEvent& event);
        bool onDoubleClickEvent(const SceneViewEvent& event);
        bool onPointerMoveEvent(const SceneViewEvent& event);
        void onPointerLeaveEvent(const SceneViewEvent& event);
        void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
        void onSceneModeChanged();
            
        void makeLinkAttitudeLevel();
        void setPlaneProjector(const SceneViewEvent& event);
        void startIK(const SceneViewEvent& event);
        //void setupFootSnap();
        void attachAttitudeDraggerTo(Link* link);
        void dragIK(const SceneViewEvent& event);
        //void snapFoot(Vector3& p, Matrix33& R);
        void startFK(const SceneViewEvent& event);
        void dragFKRotation(const SceneViewEvent& event);
        void startZmpTranslation(const SceneViewEvent& event);
        void dragZmpTranslation(const SceneViewEvent& event);
    };
}


SceneLink::SceneLink()
{
    isValid = false;
}


SceneLink::SceneLink(BodyPtr& body, JointNodeSetPtr jointNodeSet, VrmlToOsgConverter& vrmlToOsgConverter)
{
    isValid = true;
    isPointed = false;
    isColliding = false;
    
    const string& name = jointNodeSet->jointNode->defName;
    setName(name);
    link = body->link(name);

    vector<VrmlProtoInstancePtr>& segmentNodes = jointNodeSet->segmentNodes;
    const JointNodeSet::Affine3Array& transforms = jointNodeSet->transforms;
    int numSegment = segmentNodes.size();
    for(int i = 0 ; i < numSegment ; ++i){
        shapeNode = vrmlToOsgConverter.convert(segmentNodes[i]);

        if(shapeNode.valid()){

            if(link->Rs != Matrix3::Identity() || transforms[i].isApprox(Affine3::Identity())){
                osg::MatrixTransform* transform = new osg::MatrixTransform();
                Affine3 Rs;
                Rs.translation().setZero();
                Rs.linear() = link->Rs;
                const Affine3 T(Rs * transforms[i]);
                const Matrix3& R = T.linear();
                const Vector3& p = T.translation();
                
                transform->setMatrix(
                    osg::Matrix(
                        R(0,0), R(1,0), R(2,0), 0,
                        R(0,1), R(1,1), R(2,1), 0,
                        R(0,2), R(1,2), R(2,2), 0,
                        p(0),   p(1),   p(2),   1.0));
                transform->addChild(shapeNode.get());
                shapeNode = transform;
            }

            shapeNode->setDataVariance(osg::Object::STATIC);
            addChild(shapeNode.get());

            /*
            VertexNormalVisualizer* normals = new VertexNormalVisualizer(shapeNode.get());
            addChild(normals);
            */
        }
    }
}


void SceneLink::showMarker(const osg::Vec4 color, float width)
{
    if(!marker){
        osg::ComputeBoundsVisitor cbv;
        shapeNode->accept(cbv);
        osg::BoundingBox& bbox = cbv.getBoundingBox();
        //marker = new BBMarker(bbox, color, width);
        marker = new BBMarker(bbox, color);
        addChild(marker.get());
    }
}


void SceneLink::hideMarker()
{
    if(marker.valid()){
        removeChild(marker.get());
        marker = 0;
    }
}


SceneBody::SceneBody(BodyItemPtr bodyItem)
{
    impl = new SceneBodyImpl(this, bodyItem);
}


SceneBodyImpl::SceneBodyImpl(SceneBody* self, BodyItemPtr bodyItem)
    : self(self),
      bodyItem(bodyItem),
      body(bodyItem->body()),
      kinematicsBar(KinematicsBar::instance()),
      os(MessageView::mainInstance()->cout())
{
    outlinedLink = 0;
    targetLink = 0;
    attitudeDragger = new AttitudeDragger();

    osgCollision = new OsgCollision();
    osgCollision->setColdetPairs(bodyItem->selfColdetPairs);
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(osgCollision.get());
    self->addChild(geode);

    markerGroup = new osg::Group();
    self->addChild(markerGroup);

    cmMarker = new CrossMarker(osg::Vec4(0.0, 1.0, 0.0, 1.0), 0.25f, 2.0f);
    isCmVisible = false;
    zmpMarker = new SphereMarker(0.1, osg::Vec4(0.0f, 1.0f, 0.0f, 0.3f));
    zmpMarker->setCross(osg::Vec4(0.0, 1.0, 0.0, 1.0), 0.25f, 2.0f);
    isZmpVisible = false;
    
    dragMode = DRAG_NONE;
    isDragging = false;

    self->setName(bodyItem->name());
    self->setEditable(!body->isStaticModel());

    visibleSceneLinks = new osg::Group();
    self->addChild(visibleSceneLinks);

    kinematicsBar->sigCollisionVisualizationChanged().connect(
        bind(&SceneBodyImpl::onCollisionLinkHighlightModeChanged, this));
    
    onCollisionLinkHighlightModeChanged();
}


SceneBody::~SceneBody()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneBody::~SceneBody()" << endl;
    }
    delete impl;
}

Link* SceneBody::getPointedSceneLink(){
	if(impl->pointedSceneLink) return impl->pointedSceneLink->link;
	return 0;
}

osg::ref_ptr<osg::Node> SceneBody::getPointedShapeNode(){
	if(impl->pointedSceneLink) return impl->pointedSceneLink->shapeNode;
	return 0;
}


SceneBodyImpl::~SceneBodyImpl()
{
    connectionToSigWorldCollisionLinkSetChanged.disconnect();
    connections.disconnect();
}


bool SceneBody::createSceneLinks()
{
    return impl->createSceneLinks();
}


bool SceneBodyImpl::createSceneLinks()
{
    bool created = false;

    ModelNodeSetPtr modelNodeSet = bodyItem->modelNodeSet();

    if(body && modelNodeSet){

        VrmlToOsgConverter converter;
      
        sceneLinks.clear();
        createSceneLinksSub(modelNodeSet->rootJointNodeSet(), converter);
      
        size_t numLinks = body->numLinks();
        if(sceneLinks.size() < numLinks){
            for(size_t i = sceneLinks.size(); i < numLinks; ++i){
                sceneLinks.push_back(new SceneLink());
            }
        }

        created = true;
    }

    return created;
}


void SceneBodyImpl::createSceneLinksSub(JointNodeSetPtr jointNodeSet, VrmlToOsgConverter& vrmlToOsgConverter)
{
    SceneLink* sceneLink = new SceneLink(body, jointNodeSet, vrmlToOsgConverter);
    sceneLinks.push_back(sceneLink);
    visibleSceneLinks->addChild(sceneLink);
  
    vector<JointNodeSetPtr>& childJointNodeSets = jointNodeSet->childJointNodeSets;
    for(size_t i=0; i < childJointNodeSets.size(); ++i){
        createSceneLinksSub(childJointNodeSets[i], vrmlToOsgConverter);
    }
}


void SceneBody::onAttachedToScene()
{
    impl->onAttachedToScene();
}


void SceneBodyImpl::onAttachedToScene()
{
    if(sceneLinks.empty()){
        createSceneLinks();

        LeggedBody* legged = dynamic_cast<LeggedBody*>(body.get());
        if(legged && legged->numFeet() > 0){
            Link* footLink = legged->footLink(0);
            SceneLink* sceneLink = sceneLinks[footLink->index];

            osg::ComputeBoundsVisitor cbv;
            sceneLink->shapeNode->accept(cbv);
            osg::BoundingBox& bbox = cbv.getBoundingBox();
            double V = ((bbox.xMax() - bbox.xMin()) * (bbox.yMax() - bbox.yMin()) * (bbox.zMax() - bbox.zMin()));
            double r = pow(V, 1.0 / 3.0) * 0.6;
            //double r = sceneLink->shapeNode->getBound().radius() * 0.5;
            zmpMarker->setRadius(r);
            zmpMarker->setCross(osg::Vec4(0.0, 1.0, 0.0, 1.0), r * 2.5, 2.0f);
        }
        cmMarker->setSize(visibleSceneLinks->getBound().radius());
    }
    
    connections.add(bodyItem->sigUpdated().connect(
                        bind(&SceneBodyImpl::updateMarkersAndManipulators, this)));
    connections.add(bodyItem->sigKinematicStateChanged().connect(
                        bind(&SceneBodyImpl::onKinematicStateChanged, this)));
    connections.add(bodyItem->sigSelfCollisionsUpdated().connect(
                        bind(&SceneBodyImpl::onSelfCollisionsUpdated, this)));

    onCollisionLinkHighlightModeChanged();

    updateMarkersAndManipulators();
    onKinematicStateChanged();
}


void SceneBody::onDetachedFromScene()
{
    impl->onDetachedFromScene();
}

        

void SceneBodyImpl::onDetachedFromScene()
{
    connections.disconnect();
    changeCollisionLinkHighlightMode(false);
}


void SceneBodyImpl::onKinematicStateChanged()
{
    if(TRACE_FUNCTIONS){
        static int counter = 0;
        cout << "SceneBodyImpl::onKinematicStateChanged():" << counter++ << endl;
    }
    
    if(body){
        int numLinks = body->numLinks();
        for(int i=0; i < numLinks; ++i){
            Link* link = body->link(i);
            const Vector3& p = link->p;
            const Matrix3& R = link->R;
            osg::Matrix m(R(0,0), R(1,0), R(2,0), 0.0,
                          R(0,1), R(1,1), R(2,1), 0.0,
                          R(0,2), R(1,2), R(2,2), 0.0,
                          p(0),   p(1),   p(2),   1.0);
            sceneLinks[i]->setMatrix(m);
        }
    }

    if(isCmVisible){
        cmMarker->setPosition(bodyItem->centerOfMass());
    }
    if(isZmpVisible){
        zmpMarker->setPosition(bodyItem->zmp());
    }
    
    self->requestRedraw();
}


void SceneBodyImpl::createOutline(SceneLink* sceneLink)
{
    sceneLink->outline = new OsgOutlineFx(4.0);
    sceneLink->removeChild(sceneLink->shapeNode.get());
    sceneLink->outline->addChild(sceneLink->shapeNode.get());
    sceneLink->addChild(sceneLink->outline.get());
    sceneLink->shapeNode = sceneLink->outline.get();
}


bool SceneBodyImpl::setPointedLinkVisual(SceneLink* sceneLink, bool on)
{
    bool outlineCreated = false;
    
    if(!sceneLink->isPointed && on){
        if(!sceneLink->outline){
            createOutline(sceneLink);
            outlineCreated = true;
        }
        sceneLink->outline->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
        sceneLink->outline->setEnabled(true);
        sceneLink->isPointed = true;
    } else if(sceneLink->isPointed && !on){
        if(sceneLink->isColliding){
            sceneLink->outline->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
        } else {
            sceneLink->outline->setEnabled(false);
        }
        sceneLink->isPointed = false;
    }

    return outlineCreated;
}


bool SceneBodyImpl::setWorldCollisionLinkVisual(SceneLink* sceneLink, bool on)
{
    bool outlineCreated = false;
    
    if(!sceneLink->isColliding && on){
        if(!sceneLink->isPointed){
            if(!sceneLink->outline){
                createOutline(sceneLink);
                outlineCreated = true;
            }
            sceneLink->outline->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
            sceneLink->outline->setEnabled(true);
        }
        sceneLink->isColliding = true;
    } else if(sceneLink->isColliding && !on){
        if(!sceneLink->isPointed){
            sceneLink->outline->setEnabled(false);
        }
        sceneLink->isColliding = false;
    }

    return outlineCreated;
}


void SceneBodyImpl::createScribe(SceneLink* sceneLink)
{
    sceneLink->scribe = new osgFX::Scribe();
    sceneLink->scribe->setWireframeColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
    sceneLink->removeChild(sceneLink->shapeNode.get());
    sceneLink->scribe->addChild(sceneLink->shapeNode.get());
    sceneLink->addChild(sceneLink->scribe.get());
    sceneLink->shapeNode = sceneLink->scribe.get();
}


bool SceneBodyImpl::setSelfCollisionLinkVisual(SceneLink* sceneLink, bool on)
{
    bool scribeCreated = false;

    if(on){
        if(!sceneLink->scribe){
            createScribe(sceneLink);
            scribeCreated = true;
        } else if(!sceneLink->scribe->getEnabled()){
            sceneLink->scribe->setEnabled(true);
        }
    } else {
        if(sceneLink->scribe.valid()){
            sceneLink->scribe->setEnabled(false);
        }
    }
    return scribeCreated;
}


void SceneBodyImpl::onSelfCollisionsUpdated()
{
    osgCollision->dirtyDisplayList();
}


void SceneBodyImpl::onWorldCollisionLinkSetChanged()
{
    bool fxCreated = false;

    const boost::dynamic_bitset<>& w = bodyItem->worldCollisionLinkBitSet;
    const boost::dynamic_bitset<>& s = bodyItem->selfCollisionLinkBitSet;
    for(size_t i=0; i < sceneLinks.size(); ++i){
        fxCreated |= setWorldCollisionLinkVisual(sceneLinks[i].get(), (w[i] && !s[i]));
        fxCreated |= setSelfCollisionLinkVisual(sceneLinks[i].get(), s[i]);
    }

    if(fxCreated){
        self->requestRedrawWhenEffectNodeCreated();
    } else {
        self->requestRedraw();
    }
}


void SceneBodyImpl::onCollisionLinkHighlightModeChanged()
{
    changeCollisionLinkHighlightMode(kinematicsBar->isCollisionLinkHighlihtMode());
}


void SceneBodyImpl::changeCollisionLinkHighlightMode(bool on)
{
    if(!connectionToSigWorldCollisionLinkSetChanged.connected() && on){

        connectionToSigWorldCollisionLinkSetChanged =
            bodyItem->sigWorldCollisionLinkSetChanged().connect(
                bind(&SceneBodyImpl::onWorldCollisionLinkSetChanged, this));
        onWorldCollisionLinkSetChanged();

    } else if(connectionToSigWorldCollisionLinkSetChanged.connected() && !on){

        connectionToSigWorldCollisionLinkSetChanged.disconnect();
        for(size_t i=0; i < sceneLinks.size(); ++i){
            setWorldCollisionLinkVisual(sceneLinks[i].get(), false);
            setSelfCollisionLinkVisual(sceneLinks[i].get(), false);
        }
        self->requestRedraw();
    }
}


void SceneBody::setLinkVisibilities(const boost::dynamic_bitset<>& visibilities)
{
    impl->setLinkVisibilities(visibilities);
}


void SceneBodyImpl::setLinkVisibilities(const boost::dynamic_bitset<>& visibilities)
{
    visibleSceneLinks->removeChildren(0, visibleSceneLinks->getNumChildren());

    int n = std::min(visibilities.size(), sceneLinks.size());

    for(int i=0; i < n; ++i){
        if(visibilities[i] && sceneLinks[i]->isValid){

            visibleSceneLinks->addChild(sceneLinks[i].get());
        }
    }

    self->requestRedraw();
}


void SceneBody::showCenterOfMass(bool on)
{
    impl->showCenterOfMass(on);
}


void SceneBodyImpl::showCenterOfMass(bool on)
{
    isCmVisible = on;
    if(on){
        markerGroup->addChild(cmMarker.get());
        cmMarker->setPosition(bodyItem->centerOfMass());
    } else {
        markerGroup->removeChild(cmMarker.get());
    }
    self->requestRedraw();
}


bool SceneBody::isCenterOfMassVisible() const
{
    return impl->isCmVisible;
}


void SceneBody::showZmp(bool on)
{
    impl->showZmp(on);
}


void SceneBodyImpl::showZmp(bool on)
{
    isZmpVisible = on;
    if(on){
        markerGroup->addChild(zmpMarker.get());
        zmpMarker->setPosition(bodyItem->zmp());
    } else {
        markerGroup->removeChild(zmpMarker.get());
    }
    self->requestRedraw();
}


bool SceneBody::isZmpVisible() const
{
    return impl->isZmpVisible;
}


/**
   If the return value is PT_SCENE_LINK
   member variable 'pointedSceneLink' is updated with the pointed SceneLink object.
   If the pointed object is AttitudeDragger, the function returns PT_SCENE_LINK
   and member variable 'rotationAxis' is also set.
   Otherwise, 'pointedSceneLink' is set to 0.
*/
SceneBodyImpl::PointedType SceneBodyImpl::findPointedObject(const osg::NodePath& path)
{
    PointedType pointedType = PT_NONE;
    pointedSceneLink = 0;
    rotationAxis = NOAXIS;
    const int n = path.size();
    bool pathContainsSelf = false;
    for(int i=0; i < n; ++i){
        if(!pathContainsSelf){
            if(dynamic_cast<SceneBody*>(path[i]) == self){
                pathContainsSelf = true;
            }
            continue;
        }
        SceneLink* sceneLink = dynamic_cast<SceneLink*>(path[i]);
        if(sceneLink){
            pointedSceneLink = sceneLink;
            pointedType = PT_SCENE_LINK;
            if(i < n - 2){
                AttitudeDragger* dragger = dynamic_cast<AttitudeDragger*>(path[i+1]);
                if(dragger){
                    const string& axis = path[i+2]->getName();
                    if(axis == "x"){
                        rotationAxis = X;
                    } else if(axis == "y"){
                        rotationAxis = Y;
                    } else if(axis == "z"){
                        rotationAxis = Z;
                    }
                }
            }
            break;
        }
        SphereMarker* marker = dynamic_cast<SphereMarker*>(path[i]);
        if(marker == zmpMarker.get()){
            pointedType = PT_ZMP;
            break;
        }
    }
    return pointedType;
}


void SceneBodyImpl::setOutlineForPointedLink(SceneLink* sceneLink)
{
    if(outlinedLink == sceneLink){
        return;
    }

    if(outlinedLink){
        setPointedLinkVisual(outlinedLink, false);
        outlinedLink = 0;
    }

    bool outlineCreated = false;
    if(sceneLink){
        outlinedLink = sceneLink;
        outlineCreated = setPointedLinkVisual(outlinedLink, true);
    }

    if(outlineCreated){
        self->requestRedrawWhenEffectNodeCreated();
    } else {
        self->requestRedraw();
    }
}


void SceneBodyImpl::updateMarkersAndManipulators()
{
    bool show = (self->sceneMode() == SceneObject::EDIT_MODE && self->isEditable());
    
    Link* baseLink = bodyItem->currentBaseLink();
    PinDragIKptr pin = bodyItem->pinDragIK();
    
    for(size_t i=0; i < sceneLinks.size(); ++i){
        SceneLink* sceneLink = sceneLinks[i].get();
        Link* link = sceneLink->link;
        sceneLink->hideMarker();
        if(show){
            if(link == baseLink){
                sceneLink->showMarker(osg::Vec4(1.0f, 0.1f, 0.1f, 0.4f), 0.02);
            } else {
                int pinAxes = pin->pinAxes(link);
                if(pinAxes & (InverseKinematics::TRANSFORM_6D)){
                    sceneLink->showMarker(osg::Vec4(1.0f, 1.0f, 0.1f, 0.4f), 0.02);
                }
            }
        }
    }

    attitudeDragger->detach();

    if(show && targetLink && (kinematicsBar->mode() != KinematicsBar::FK_MODE) && kinematicsBar->isAttitudeMode()){
        attitudeDragger->attachTo(sceneLinks[targetLink->index].get());
    }

    self->requestRedraw();
}


void SceneBodyImpl::makeLinkFree(SceneLink* sceneLink)
{
    Link* link = sceneLink->link;
    if(bodyItem->currentBaseLink() == link){
        bodyItem->setCurrentBaseLink(0);
    }
    
    bodyItem->pinDragIK()->setPin(link, InverseKinematics::NO_AXES);

    bodyItem->notifyUpdate();
}


void SceneBodyImpl::setBaseLink(SceneLink* sceneLink)
{
    bodyItem->setCurrentBaseLink(sceneLink->link);
    bodyItem->notifyUpdate();
}


void SceneBodyImpl::toggleBaseLink(SceneLink* sceneLink)
{
    Link* baseLink = bodyItem->currentBaseLink();
    if(sceneLink->link != baseLink){
        bodyItem->setCurrentBaseLink(sceneLink->link);
    } else {
        bodyItem->setCurrentBaseLink(0);
    }
    bodyItem->notifyUpdate();
}


void SceneBodyImpl::togglePin(SceneLink* sceneLink, bool toggleTranslation, bool toggleRotation)
{
    Link* link = sceneLink->link;
    PinDragIKptr pin = bodyItem->pinDragIK();

    InverseKinematics::AxisSet axes = pin->pinAxes(link);

    if(toggleTranslation && toggleRotation){
        if(axes == InverseKinematics::NO_AXES){
            axes = InverseKinematics::TRANSFORM_6D;
        } else {
            axes = InverseKinematics::NO_AXES;
        }
    } else {
        if(toggleTranslation){
            axes = (InverseKinematics::AxisSet)(axes ^ InverseKinematics::TRANSLATION_3D);
        }
        if(toggleRotation){
            axes = (InverseKinematics::AxisSet)(axes ^ InverseKinematics::ROTATION_3D);
        }
    }
        
    pin->setPin(link, axes);
    bodyItem->notifyUpdate();
}


bool SceneBody::onKeyPressEvent(const SceneViewEvent& event)
{
    return impl->onKeyPressEvent(event);
}


bool SceneBodyImpl::onKeyPressEvent(const SceneViewEvent& event)
{
    if(!outlinedLink){
        return false;
    }

    bool handled = true;

    int key = event.key();
    if(islower(key) == 0){
        key = toupper(key);
    }

    switch(key){
    case 'B':
        toggleBaseLink(outlinedLink);
        break;
        
    case 'R':
        togglePin(outlinedLink, false, true);
        break;

    case 'T':
        togglePin(outlinedLink, true, false);
        break;

    default:
        handled = false;
        break;
    }
        
    return handled;
}


bool SceneBody::onKeyReleaseEvent(const SceneViewEvent& event)
{
    return impl->onKeyReleaseEvent(event);
}


bool SceneBodyImpl::onKeyReleaseEvent(const SceneViewEvent& event)
{
    return false;
}


bool SceneBody::onButtonPressEvent(const SceneViewEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool SceneBodyImpl::onButtonPressEvent(const SceneViewEvent& event)
{
    bool handled = false;
    
    isDragging = false;
    
    PointedType pointedType = findPointedObject(event.path());

    setOutlineForPointedLink(pointedSceneLink);

    if(pointedType == PT_SCENE_LINK){

        if(event.button() == GUIEventAdapter::LEFT_MOUSE_BUTTON){
            targetLink = pointedSceneLink->link;
            updateMarkersAndManipulators();
            ik.reset();

            switch(kinematicsBar->mode()){
            case KinematicsBar::AUTO_MODE:
                ik = body->getDefaultIK(targetLink);
                if(ik){
                    startIK(event);
                } else {
                    startFK(event);
                }
                break;
            case KinematicsBar::FK_MODE:
                if(targetLink == bodyItem->currentBaseLink()){
                    startIK(event);
                } else {
                    startFK(event);
                }
                break;
            case KinematicsBar::IK_MODE:
                startIK(event);
                break;
            }
            
        } else if(event.button() == GUIEventAdapter::MIDDLE_MOUSE_BUTTON){
            togglePin(pointedSceneLink, true, true);
            
        } else if(event.button() == GUIEventAdapter::RIGHT_MOUSE_BUTTON){

        }

        handled = true;

    } else if(pointedType == PT_ZMP){
        startZmpTranslation(event);
        handled = true;
    }

    if(dragMode != DRAG_NONE && outlinedLink){
        setPointedLinkVisual(outlinedLink, false);
        self->requestRedraw();
    }

    return handled;
}


bool SceneBody::onButtonReleaseEvent(const SceneViewEvent& event)
{
    return impl->onButtonReleaseEvent(event);
}


bool SceneBodyImpl::onButtonReleaseEvent(const SceneViewEvent& event)
{
    isDragging = false;
    
    if(dragMode != DRAG_NONE){
        bodyItem->acceptKinematicStateEdit();
        dragMode = DRAG_NONE;
        if(outlinedLink){
            setPointedLinkVisual(outlinedLink, true);
            self->requestRedraw();
        }
        return true;
    }
    return false;
}


bool SceneBody::onDoubleClickEvent(const SceneViewEvent& event)
{
    return impl->onDoubleClickEvent(event);
}


bool SceneBodyImpl::onDoubleClickEvent(const SceneViewEvent& event)
{
    if(findPointedObject(event.path()) == PT_SCENE_LINK){
        if(event.button() == GUIEventAdapter::LEFT_MOUSE_BUTTON){
            if(BodyBar::instance()->makeSingleSelection(bodyItem)){
                LinkSelectionView::mainInstance()->makeSingleSelection(bodyItem, pointedSceneLink->link->index);
            }
            return true;
        }
    }
    return false;
}


bool SceneBody::onPointerMoveEvent(const SceneViewEvent& event)
{
    return impl->onPointerMoveEvent(event);
}


bool SceneBodyImpl::onPointerMoveEvent(const SceneViewEvent& event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneBodyImpl::onPointerMoveEvent()" << endl;
    }

    if(dragMode == DRAG_NONE){
        findPointedObject(event.path());
        setOutlineForPointedLink(pointedSceneLink);
        
        static format f(_("%1% / %2%"));
        if(pointedSceneLink){
            event.updateIndicator(str(f % bodyItem->name() % pointedSceneLink->link->name()));
        } else {
            event.updateIndicator("");
        }
    } else {
        if(!isDragging){
            bodyItem->beginKinematicStateEdit();
            isDragging = true;
        }

        switch(dragMode){

        case LINK_IK_TRANSLATION:
        case LINK_IK_ROTATION:
            dragIK(event);
            break;
            
        case LINK_FK_ROTATION:
            dragFKRotation(event);
            break;
            
        case ZMP_TRANSLATION:
            dragZmpTranslation(event);
            break;
            
        default:
            break;
        }
    }

    return true;
}


void SceneBody::onPointerLeaveEvent(const SceneViewEvent& event)
{
    impl->onPointerLeaveEvent(event);
}


void SceneBodyImpl::onPointerLeaveEvent(const SceneViewEvent& event)
{
    setOutlineForPointedLink(0);
}


void SceneBody::onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager)
{
    impl->onContextMenuRequest(event, menuManager);
}


void SceneBodyImpl::onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager)
{
    PointedType pointedType = findPointedObject(event.path());

    if(bodyItem && pointedType == PT_SCENE_LINK){

        menuManager.addItem(_("Set Free"))->sigTriggered().connect(
            bind(&SceneBodyImpl::makeLinkFree, this, pointedSceneLink));
        menuManager.addItem(_("Set Base"))->sigTriggered().connect(
            bind(&SceneBodyImpl::setBaseLink, this, pointedSceneLink));
        menuManager.addItem(_("Set Translation Pin"))->sigTriggered().connect(
            bind(&SceneBodyImpl::togglePin, this, pointedSceneLink, true, false));
        menuManager.addItem(_("Set Rotation Pin"))->sigTriggered().connect(
            bind(&SceneBodyImpl::togglePin, this, pointedSceneLink, false, true));
        menuManager.addItem(_("Set Both Pins"))->sigTriggered().connect(
            bind(&SceneBodyImpl::togglePin, this, pointedSceneLink, true, true));

        menuManager.addSeparator();

        menuManager.addItem(_("Level Attitude"))->sigTriggered().connect(
            bind(&SceneBodyImpl::makeLinkAttitudeLevel, this));

        menuManager.addSeparator();
        
        menuManager.setPath(N_("/Markers"));
        
        Action* item = menuManager.addCheckItem(_("Center of Mass"));
        item->setChecked(isCmVisible);
        item->sigToggled().connect(bind(&SceneBodyImpl::showCenterOfMass, this, _1));

        item = menuManager.addCheckItem(_("ZMP"));
        item->setChecked(isZmpVisible);
        item->sigToggled().connect(bind(&SceneBodyImpl::showZmp, this, _1));
    }
}


void SceneBody::onSceneModeChanged()
{
    impl->onSceneModeChanged();
}


void SceneBodyImpl::onSceneModeChanged()
{
    if(self->sceneMode() == SceneObject::VIEW_MODE && outlinedLink){
        setOutlineForPointedLink(0);
    }
    updateMarkersAndManipulators();
}


bool SceneBody::onUndoRequest()
{
    return impl->bodyItem->undoKinematicState();
}


bool SceneBody::onRedoRequest()
{
    return impl->bodyItem->redoKinematicState();
}


void SceneBodyImpl::makeLinkAttitudeLevel()
{
    if(pointedSceneLink){
        Link* targetLink = pointedSceneLink->link;
        InverseKinematicsPtr ik = bodyItem->getCurrentIK(targetLink);
        if(ik){
            const Matrix3& R = targetLink->R;
            const double theta = acos(R(2, 2));
            const Vector3 z(R(0,2), R(1, 2), R(2, 2));
            const Vector3 axis = z.cross(Vector3::UnitZ()).normalized();
            const Matrix3 R2 = AngleAxisd(theta, axis) * R;

            bodyItem->beginKinematicStateEdit();
            if(ik->calcInverseKinematics(targetLink->p, R2)){
                bodyItem->notifyKinematicStateChange(true);
                bodyItem->acceptKinematicStateEdit();
            }
        }
    }
}


void SceneBodyImpl::setPlaneProjector(const SceneViewEvent& event)
{
    osg::Vec3 eye, center, up;
    event.camera()->getViewMatrixAsLookAt(eye, center, up);
    projector = new osgManipulator::PlaneProjector(osg::Plane(eye - center, event.point()));
    pointerInfo.reset();
    pointerInfo.setCamera(event.camera());
}


/**
   \todo use BodyItem::getCurrentIK();
*/
void SceneBodyImpl::startIK(const SceneViewEvent& event)
{
    orgPointerPos << event.point().x(), event.point().y(), event.point().z();
    orgTargetLinkPos = targetLink->p;

    Link* baseLink = bodyItem->currentBaseLink();

    if(!ik){
        if(bodyItem->pinDragIK()->numPinnedLinks() == 0 && baseLink){
            ikPath = body->getJointPath(baseLink, targetLink);
            if(ikPath){
                if(!ikPath->hasAnalyticalIK()){
                    //ikPath->setBestEffortIKMode(true);
                }
                ik = ikPath;
            }
        }
    }
    
    if(!ik){
        pinDragIK = bodyItem->pinDragIK();
        pinDragIK->setBaseLink(baseLink);
        pinDragIK->setTargetLink(targetLink, kinematicsBar->isAttitudeMode());
        if(pinDragIK->initialize()){
            ik = pinDragIK;
        }
    }
    
    if(ik){

        projector = 0;
        
        if(kinematicsBar->isAttitudeMode()){

            if(rotationAxis >= X){
                orgAttitude = targetLink->R;
                const Matrix3& R = orgAttitude;
                axis = R.col(rotationAxis);
                const Vector3 arm = orgPointerPos - (axis.dot(orgPointerPos - orgTargetLinkPos) * axis + orgTargetLinkPos);
                rotationBaseX = arm.normalized();
                rotationBaseY = axis.cross(arm).normalized();
                const osg::Vec3 center(orgTargetLinkPos[0], orgTargetLinkPos[1], orgTargetLinkPos[2]);
                const float radius = arm.norm();
                osg::Cylinder* cylinder = new osg::Cylinder(center, radius, std::numeric_limits<float>::max());
                osg::Quat rot;
                rot.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), osg::Vec3d(axis[0], axis[1], axis[2]));
                cylinder->setRotation(rot);
                projector = new osgManipulator::CylinderProjector(cylinder);                
                pointerInfo.reset();
                pointerInfo.setCamera(event.camera());
                dragMode = LINK_IK_ROTATION;
            }
        }

        if(!projector){
            setPlaneProjector(event);
            dragMode = LINK_IK_TRANSLATION;
        }

        fkTraverse.find(baseLink ? baseLink : body->rootLink(), true, true);

        /*
        isSnapping = false;
        snapFootLinkInfos.clear();
        if(kinematicsBar->isFootSnapMode()){
            setupFootSnap();
        }
        */

        if(kinematicsBar->isPenetrationBlockMode()){
            penetrationBlocker = bodyItem->createPenetrationBlocker(targetLink, true);
        } else {
            penetrationBlocker.reset();
        }
    }

    self->requestRedraw();
}


/**
   \todo This function should be implemented in the PoseSeq plugin
*/
/*
void SceneBodyImpl::setupFootSnap()
{
    bool isFootLink = false;
    const vector<BodyItem::FootInfo>& feet = bodyItem->footInfos;
    for(size_t i=0; i < feet.size(); ++i){
        if(feet[i].link == targetLink){
            isFootLink = true;
            break;
        }
    }
    if(isFootLink){
        PoseSeqItem* poseSeqItem =
            ItemTreeView::mainInstance()->selectedSubItem<PoseSeqItem>(bodyItem, true);
        if(!poseSeqItem){
            BodyMotionItem* motionItem =
                ItemTreeView::mainInstance()->selectedSubItem<BodyMotionItem>(bodyItem, true);
            if(motionItem){
                poseSeqItem = motionItem->findOwnerItem<PoseSeqItem>();
            }
        }
            
        if(poseSeqItem){
            PoseSeqPtr pseq = poseSeqItem->poseSeq();
            TimeBar* timeBar = TimeBar::instance();
            double time = timeBar->time();
            /// \todo cache the previous iterator
            PoseSeq::iterator p = pseq->seek(pseq->begin(), time);
            if(p != pseq->end()){
                double ts = timeBar->timeStep();

                PoseSeq::iterator prev = p;
                do {
                    if(fabs(prev->time() - time) >= ts){
                        PosePtr pose = prev->get<Pose>();
                        if(pose){
                            Pose::LinkInfo* linkInfo = pose->ikLinkInfo(targetLink->index);
                            if(linkInfo){
                                snapFootLinkInfos.push_back(linkInfo);
                                break;
                            }
                        }
                    }
                } while(prev-- != pseq->begin());

                PoseSeq::iterator next = ++p;
                while(next != pseq->end()){
                    if(fabs(next->time() - time) >= ts){
                        PosePtr pose = next->get<Pose>();
                        if(pose){
                            Pose::LinkInfo* linkInfo = pose->ikLinkInfo(targetLink->index);
                            if(linkInfo){
                                snapFootLinkInfos.push_back(linkInfo);
                                break;
                            }
                        }
                    }
                    ++next;
                }
            }
        }
    }
}
*/


void SceneBodyImpl::dragIK(const SceneViewEvent& event)
{
    Vec3ForProjection p;
    pointerInfo.setMousePosition(event.x(), event.y());
    if(projector->project(pointerInfo, p)){
        Vector3 newPos;
        Matrix3 newR;
        if(dragMode == LINK_IK_TRANSLATION){
            newPos = (orgTargetLinkPos + (Vector3(p.x(), p.y(), p.z()) - orgPointerPos));
            newR = targetLink->R;

            if(penetrationBlocker){
                penetrationBlocker->adjust(newPos, newR, newPos - targetLink->p);
            }
            /*
            if(!snapFootLinkInfos.empty()){
                snapFoot(newPos, newR);
            }
            */
            
        } else if(dragMode == LINK_IK_ROTATION){
            const Vector3 r = Vector3(p.x(), p.y(), p.z()) - orgTargetLinkPos;
            const double angle = atan2(r.dot(rotationBaseY), r.dot(rotationBaseX));
            newR = AngleAxisd(angle, axis) * orgAttitude;
            newPos = orgTargetLinkPos;
        }
        if(ik->calcInverseKinematics(newPos, newR)){
            fkTraverse.calcForwardKinematics();
            bodyItem->notifyKinematicStateChange(true);
        }
    }
}


/*
void SceneBodyImpl::snapFoot(Vector3& p, Matrix33& R)
{
    double distanceThresh, angleThresh;
    kinematicsBar->getSnapThresholds(distanceThresh, angleThresh);
    
    double eMin = std::numeric_limits<double>::max();
    for(size_t i=0; i < snapFootLinkInfos.size(); ++i){
        Pose::LinkInfo* info = snapFootLinkInfos[i];
        double eT = norm2(info->p - p); // * 40.0; // 2.5cm
        double eR = norm2(omegaFromRot(Matrix33(trans(R) * info->R))); // * 2.0; // about 30 deg
        if(eT < distanceThresh && eR < angleThresh){
            double eTotal = eT * 40.0 + eR * 2.0;
            if(eTotal < eMin){
                if(!isSnapping){
                    footAttitudeBeforeSnap = R;
                }
                p = info->p;
                R = info->R;
                eMin = eTotal;
                isSnapping = true;
            }
        }
    }
    if(isSnapping && eMin == std::numeric_limits<double>::max()){
        R = footAttitudeBeforeSnap;
        isSnapping = false;
    }
}
*/


void SceneBodyImpl::startFK(const SceneViewEvent& event)
{
    if(targetLink->jointType == Link::ROTATIONAL_JOINT){

        orgJointPosition = targetLink->q;
        const Vector3 pos(event.point().x(), event.point().y(), event.point().z());
        const Vector3 axis = targetLink->R * targetLink->a;
        const Vector3 arm = pos - (axis.dot(pos - targetLink->p) * axis + targetLink->p);

        if(arm.norm() > 1.0e-6){
        
            rotationBaseX = arm.normalized();
            rotationBaseY = axis.cross(rotationBaseX);
            osg::Vec3 eye, center, up;
            event.camera()->getViewMatrixAsLookAt(eye, center, up);
            osg::Vec3 eyeDirection(center - eye);
            eyeDirection.normalize();
            osg::Vec3d osgAxis(axis[0], axis[1], axis[2]);
			
            if(fabs(osgAxis * eyeDirection) > 0.7){
                projector = new osgManipulator::PlaneProjector(osg::Plane(osg::Vec3d(axis[0], axis[1], axis[2]), event.point()));
            } else {
                const float radius = arm.norm();
                const osg::Vec3 center(targetLink->p[0], targetLink->p[1], targetLink->p[2]);
                osg::Cylinder* cylinder = new osg::Cylinder(center, radius, std::numeric_limits<float>::max());
                osg::Quat rot;
                rot.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), osgAxis);
                cylinder->setRotation(rot);
                projector = new osgManipulator::CylinderProjector(cylinder);
            }

            pointerInfo.reset();
            pointerInfo.setCamera(event.camera());
            dragMode = LINK_FK_ROTATION;
        }
    }
}


void SceneBodyImpl::dragFKRotation(const SceneViewEvent& event)
{
    Vec3ForProjection pos0;
    pointerInfo.setMousePosition(event.x(), event.y());
    if(projector->project(pointerInfo, pos0)){
        const Vector3 p = Vector3(pos0.x(), pos0.y(), pos0.z()) - targetLink->p;
        const double x = p.dot(rotationBaseX);
        const double y = p.dot(rotationBaseY);
        const double angle = atan2(y, x);
        targetLink->q = orgJointPosition + angle;
        bodyItem->notifyKinematicStateChange(true);
    }
}


void SceneBodyImpl::startZmpTranslation(const SceneViewEvent& event)
{
    orgPointerPos << event.point().x(), event.point().y(), event.point().z();
    osg::Plane plane(osg::Vec3(0.0, 0.0, 1.0), event.point());
    projector = new osgManipulator::PlaneProjector(plane);
    pointerInfo.reset();
    pointerInfo.setCamera(event.camera());
    orgZmpPos = bodyItem->zmp();
    dragMode = ZMP_TRANSLATION;
}


void SceneBodyImpl::dragZmpTranslation(const SceneViewEvent& event)
{
    Vec3ForProjection p;
    pointerInfo.setMousePosition(event.x(), event.y());
    if(projector->project(pointerInfo, p)){
        bodyItem->setZmp(orgZmpPos + (Vector3(p.x(), p.y(), p.z()) - orgPointerPos));
        bodyItem->notifyKinematicStateChange(true);
    }
}
