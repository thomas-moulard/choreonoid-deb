/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneView.h"
#include "SceneObject.h"
#include "OsgViewer.h"
#include "ToolBar.h"
#include "MessageView.h"
#include "MenuManager.h"
#include "Archive.h"
#include "MainWindow.h"
#include "SpinBox.h"
#include "Separator.h"
#include <cnoid/EigenUtil>
#include <QLabel>
#include <QBoxLayout>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osgText/Text>
#include <osgText/Font>
#include <osgViewer/Viewer>
#include <osgFX/Scribe>
#include <osgFX/Cartoon>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowTexture>
#include <osgDB/WriteFile>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    bool TRACE_FUNCTIONS = false;
    SceneView* mainSceneView = 0;
    const bool ENABLE_PERSPECTIVE_COORDINATE_AXES = false;


    class SetupDialog : public QDialog
    {
    public:
        DoubleSpinBox floorGridSpanSpin;
        DoubleSpinBox floorGridIntervalSpin;
        CheckBox hiPriorityRenderingCheck;

        SetupDialog(SceneViewImpl* impl);
        void storeState(Archive& archive);
        void restoreState(const Archive& archive);
    };
}


namespace cnoid {
    
    class SceneViewImpl : public QObject, public boost::signals::trackable
    {
    public:
        SceneViewImpl(SceneView* self);
        ~SceneViewImpl();

        SceneView* self;
        SetupDialog setup;
        OsgViewer* viewer;
        int drawingAreaHeight;

        osg::ref_ptr<osg::Camera> camera;
        int numOverriddenCameraControls;
        enum ProjectionMode { PERSPECTIVE, ORTHO } projectionMode;
        osg::Matrixd orgOrthoProjectionMatrix;
        osg::Matrixd orgPerspectiveProjectionMatrix;

        osg::ref_ptr<osg::Camera> hudCamera;
        osg::ref_ptr<osg::PositionAttitudeTransform> axesTransform;
        bool coordinateAxesEnabled;
        osg::ref_ptr<osg::Group> rootNode;
        osg::ref_ptr<osg::Group> objectGroupNode;
        osg::ref_ptr<osg::Switch> switchNode;
        osg::ref_ptr<osg::Geode> floorGridNode;
        osg::ref_ptr<osg::PolygonMode> polygonMode;
        osg::ref_ptr<osgFX::Scribe> scribe;
        osg::ref_ptr<osgFX::Cartoon> cartoon;
        osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene;
        osg::ref_ptr<osgShadow::ShadowMap> shadowMap;

        osg::ref_ptr<osg::Group> preprocessingNodeGroup;

        ostream& os;
        QLabel* indicatorLabel;
        
        ToolButton* editModeToggle;

        ToolButton* perspectiveToggle;
        ToolButton* floorGridToggle;
        ToolButton* collisionToggle;
        Action* shadowCheck;

        QString lastCaptureFile;
        QString lastCaptureFolder;
        
        Menu* popupMenu;
        MenuManager menuManager;

        SceneObject::SceneMode operationMode;
        enum DragMode { NO_DRAGGING, OBJECT_DELEGATION, VIEW_ROTATION, VIEW_TRANSLATION, VIEW_ZOOM } dragMode;

        osg::Matrixd homeViewMatrix;
        double orgMouseX;
        double orgMouseY;
        osg::Matrixd orgViewMatrix;
        osg::Vec3d orgViewUp;
        osg::Vec3d orgViewSide;
        osg::Vec3d orgEye;
        osg::Vec3d orgViewForward;
        double viewTranslationRatioX;
        double viewTranslationRatioY;
        double orgZoomDistance;

        SceneViewEvent latestEvent;
        SceneObject* pointedObject;
        osg::Vec3d orgPointedPos;

        void initializeOsgObjects();
        void initializeGL();
        void setupMenu(MenuManager& mm);
        ToolBar* setupToolBar();
        void onEditModeToggled(bool on);
        void setOperationMode(SceneObject::SceneMode mode);
        void flipOperationMode();
        
        void initializeCamera();
        void resetCameraProjection();
        void correctCameraProjectionAspectRatio();
        void viewAll();
        
        osg::Node* createLights();
        void setupHUD();
        void setupCoordinateAxes();
        osg::Node* createAxes(float length, float width);
        void enableCoordinateAxes(bool on, bool doRequestRedraw);
        void updateAxesPosition();
        void setupFloorGrid();
        void updateFloorGridLines();

        void addSceneObject(SceneObject* object);
        void removeSceneObject(SceneObject* object);
        void onSceneObjectRedrawRequest(int flags);
        
        void setHomeView();

        bool onViewerFocusOut(QFocusEvent* event);
        bool handleOsgEvent(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action);
        void handleOsgResizeEvent(const osgGA::GUIEventAdapter& event);

        void updateLatestEvent(QKeyEvent* event);
        void updateLatestEvent(int x, int y, int modifiers);
        void updateLatestEvent(QMouseEvent* event);

        virtual bool eventFilter(QObject* obj, QEvent* event);
        
        bool onKeyPressEvent(QKeyEvent* event);
        bool onKeyReleaseEvent(QKeyEvent* event);
        bool onMousePressEvent(QMouseEvent* event);
        bool onMouseReleaseEvent(QMouseEvent* event);
        bool onMouseDoubleClickEvent(QMouseEvent* event);
        bool onMouseMoveEvent(QMouseEvent* event);
        void updatePointerPosition();
        bool onWheelEvent(QWheelEvent* event);
        
        SceneObject* findPointedObject();
        void showViewModePopupMenu(const QPoint& globalPos);
        void showEditModePopupMenu(SceneObject* object, const QPoint& globalPos);
        void onPopupMenuSelectionDone();
        void startViewRotation();
        void dragViewRotation(QMouseEvent* event);
        void startViewTranslation();
        void dragViewTranslation();
        void startViewZoom();
        void dragViewZoom();
        void zoomView(double ratio);

        void onWireframeModeToggled(bool on);
        void onScribeModeToggled(bool on);
        void onCartoonModeToggled(bool on);
        void onFloorGridToggled(bool on);
        void onPerspectiveViewToggled(bool on);
        void onShadowToggled(bool on);
        void onCollisionToggled(bool on);
        void onOverPaintToggled(bool on);
        void onHiPriorityRenderingToggled(bool on);
        void onCaptureImageButtonClicked();
        
        bool saveImage(const std::string& filename);
        void setScreenSize(int width, int height);
        
        bool storeState(Archive& archive);
        bool restoreState(const Archive& archive);
    };
}


namespace {

    class SceneViewEventHandler : public osgGA::GUIEventHandler
    {
    public:
        SceneViewEventHandler(SceneViewImpl* sceneWindow) : sceneWindow(sceneWindow) { }
        
        bool handle(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action) {
            return sceneWindow->handleOsgEvent(event, action);
        }
        
        SceneViewImpl* sceneWindow;
    };


    class OsgViewerEx : public OsgViewer
    {
    public:
        OsgViewerEx(SceneViewImpl* viewImpl)
            : OsgViewer(viewImpl->self),
              viewImpl(viewImpl) {
        }
        
        virtual void initializeGL() {
            viewImpl->initializeGL();
        }

        SceneViewImpl* viewImpl;
    };
}


SetupDialog::SetupDialog(SceneViewImpl* impl) : QDialog(MainWindow::instance())
{
    setWindowTitle(_("SceneView Setup"));

    QVBoxLayout* vbox = new QVBoxLayout();
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Floor Grid"))));
    
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Span")));
    floorGridSpanSpin.setAlignment(Qt::AlignCenter);
    floorGridSpanSpin.setDecimals(1);
    floorGridSpanSpin.setRange(0.0, 99.9);
    floorGridSpanSpin.setSingleStep(0.1);
    floorGridSpanSpin.setValue(10.0);
    floorGridSpanSpin.sigValueChanged().connect(bind(&SceneViewImpl::updateFloorGridLines, impl));
    hbox->addWidget(&floorGridSpanSpin);
    hbox->addSpacing(8);
    
    hbox->addWidget(new QLabel(_("Interval")));
    floorGridIntervalSpin.setAlignment(Qt::AlignCenter);
    floorGridIntervalSpin.setDecimals(2);
    floorGridIntervalSpin.setRange(0.01, 9.99);
    floorGridIntervalSpin.setSingleStep(0.01);
    floorGridIntervalSpin.setValue(0.5);
    floorGridIntervalSpin.sigValueChanged().connect(bind(&SceneViewImpl::updateFloorGridLines, impl));
    hbox->addWidget(&floorGridIntervalSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addWidget(new HSeparator());
    
    hbox = new QHBoxLayout();
    hiPriorityRenderingCheck.setText(_("Hi-priority rendering mode"));
    hiPriorityRenderingCheck.setChecked(false);
    hiPriorityRenderingCheck.sigToggled().connect(bind(&SceneViewImpl::onHiPriorityRenderingToggled, impl, _1));
    hbox->addWidget(&hiPriorityRenderingCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator());
    QPushButton* okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);
    
    setLayout(vbox);
}


void SetupDialog::storeState(Archive& archive)
{
    archive.write("floorGridSpan", floorGridSpanSpin.value());
    archive.write("floorGridInterval", floorGridIntervalSpin.value());
    archive.write("hiPriorityRendering", hiPriorityRenderingCheck.isChecked());
}


void SetupDialog::restoreState(const Archive& archive)
{
    floorGridSpanSpin.setValue(archive.get("floorGridSpan", floorGridSpanSpin.value()));
    floorGridIntervalSpin.setValue(archive.get("floorGridInterval", floorGridIntervalSpin.value()));
    hiPriorityRenderingCheck.setChecked(archive.get("hiPriorityRendering", hiPriorityRenderingCheck.isChecked()));
}


SceneView* SceneView::mainInstance()
{
    assert(mainSceneView);
    return mainSceneView;
}


void SceneView::initialize(ExtensionManager* ext)
{
    if(!mainSceneView){
        mainSceneView = new SceneView();
        mainSceneView->impl->setupMenu(ext->menuManager());
        ext->addView(mainSceneView);
        ext->addToolBar(mainSceneView->impl->setupToolBar());
    }
}


SceneView::SceneView()
{
    impl = new SceneViewImpl(this);
}


SceneViewImpl::SceneViewImpl(SceneView* self)
    : self(self),
      setup(this),
      os(MessageView::mainInstance()->cout())
{
    self->setName(N_("Scene"));
    self->setDefaultLayoutArea(View::RIGHT);
    self->setMinimumSize(1, 1);

    operationMode = SceneObject::VIEW_MODE;
    dragMode = NO_DRAGGING;
    pointedObject = 0;
    drawingAreaHeight = 0;
    latestEvent.sceneView = self;

    indicatorLabel = new QLabel();
    indicatorLabel->setAlignment(Qt::AlignLeft);
    QFont font = indicatorLabel->font();
    font.setFixedPitch(true);
    indicatorLabel->setFont(font);

    viewer = new OsgViewerEx(this);
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(viewer);
    self->setLayout(layout);
    viewer->addEventHandler(new SceneViewEventHandler(this));
    viewer->getEventQueue()->getCurrentEventState()
        ->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);
    viewer->setMouseTracking(true);
    viewer->installEventFilter(this);

    popupMenu = new Menu(self);
    menuManager.setTopMenu(popupMenu);
    popupMenu->sigTriggered().connect(bind(&SceneViewImpl::onPopupMenuSelectionDone, this));

    viewer->requestRedraw();
}


SceneView::~SceneView()
{
    if(impl){
        delete impl;
    }
}


SceneViewImpl::~SceneViewImpl()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneViewImpl::~SceneViewImpl()" << endl;
    }
    
    delete indicatorLabel;
}


void SceneViewImpl::initializeGL()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneViewImpl::initializeGL()" << endl;
    }

    static bool isFirst = true;
    if(isFirst){
        initializeOsgObjects();
        isFirst = false;
    }
}


void SceneViewImpl::initializeOsgObjects()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneViewImpl::initializeOsgObjects()" << endl;
    }
    
    initializeCamera();

    rootNode = new osg::Group;

    preprocessingNodeGroup = new osg::Group;
    rootNode->addChild(preprocessingNodeGroup.get());

    rootNode->addChild(createLights());

    objectGroupNode = new osg::Group;
    rootNode->addChild(objectGroupNode.get());

    osg::StateSet* stateSet = objectGroupNode->getOrCreateStateSet();

    polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
    stateSet->setAttributeAndModes(polygonMode.get());

    switchNode = new osg::Switch;
    rootNode->addChild(switchNode.get());

    setupHUD();
    rootNode->addChild(hudCamera.get());
    
    setupCoordinateAxes();
    
    setupFloorGrid();

    viewer->setSceneData(rootNode.get());
}    


void SceneViewImpl::setupMenu(MenuManager& mm)
{
    mm.setPath("/Options").setPath(N_("Scene View"));
    
    Action* coordinateAxesCheck = mm.addCheckItem(_("Coordinate axes"));
    coordinateAxesCheck->setChecked(true);
    coordinateAxesCheck->sigToggled().connect(bind(&SceneViewImpl::enableCoordinateAxes, this, _1, true));

    Action* scribeCheck = mm.addCheckItem(_("Scribe mode"));
    scribeCheck->sigToggled().connect(bind(&SceneViewImpl::onScribeModeToggled, this, _1));
    
    Action* cartoonCheck = mm.addCheckItem(_("Cartoon mode on/off"));
    cartoonCheck->sigToggled().connect(bind(&SceneViewImpl::onCartoonModeToggled, this, _1));

    shadowCheck = mm.addCheckItem(_("toggle shadow on/off"));
    shadowCheck->sigToggled().connect(bind(&SceneViewImpl::onShadowToggled, this, _1));

    Action* overPaintCheck = mm.addCheckItem(_("Over paint mode"));
    overPaintCheck->sigToggled().connect(bind(&SceneViewImpl::onOverPaintToggled, this, _1));
}


ToolBar* SceneViewImpl::setupToolBar()
{
    ToolBar* toolBar = new ToolBar(N_("SceneBar"));

    editModeToggle = toolBar->addToggleButton(QIcon(":/Base/icons/sceneedit.png"), _("Switch to the edit mode"));
    editModeToggle->setChecked(false);
    editModeToggle->sigToggled().connect(bind(&SceneViewImpl::onEditModeToggled, this, _1));

    toolBar->addButton(QIcon(":/Base/icons/viewfitting.png"), _("Move the camera to look at the objects"))
        ->sigClicked().connect(bind(&SceneViewImpl::viewAll, this));

    perspectiveToggle = toolBar->addToggleButton(QIcon(":/Base/icons/perspective.png"), _("Toggle the perspective / orthogonal view mode"));
    perspectiveToggle->setChecked(true);
    perspectiveToggle->sigToggled().connect(bind(&SceneViewImpl::onPerspectiveViewToggled, this, _1));

    floorGridToggle = toolBar->addToggleButton(QIcon(":/Base/icons/floorgrid.png"), _("Show the floor grid"));
    floorGridToggle->setChecked(true);
    floorGridToggle->sigToggled().connect(bind(&SceneViewImpl::onFloorGridToggled, this, _1));

    collisionToggle = toolBar->addToggleButton(QIcon(":/Base/icons/collisionlines.png"), _("Toggle the collision visualization"));
    collisionToggle->setChecked(true);
    viewer->isCollisionVisibleMode_ = collisionToggle->isChecked();
    collisionToggle->sigToggled().connect(bind(&SceneViewImpl::onCollisionToggled, this, _1));

    ToolButton* wireframeToggle = toolBar->addToggleButton(QIcon(":/Base/icons/wireframe.png"), _("Toggle the wireframe mode"));
    wireframeToggle->sigToggled().connect(bind(&SceneViewImpl::onWireframeModeToggled, this, _1));

    toolBar->addButton(QIcon(":/Base/icons/scenecapture.png"), _("Capture a screen image and save it into a file"))
        ->sigClicked().connect(bind(&SceneViewImpl::onCaptureImageButtonClicked, this));

    toolBar->addButton(QIcon(":/Base/icons/setup.png"), _("Open the dialog to setup SceneView"))
        ->sigClicked().connect(bind(&SetupDialog::show, &setup));

    return toolBar;
}


QWidget* SceneView::indicatorOnInfoBar()
{
    return impl->indicatorLabel;
}


void SceneViewImpl::onEditModeToggled(bool on)
{
    setOperationMode(on ? SceneObject::EDIT_MODE : SceneObject::VIEW_MODE);
}


void SceneViewImpl::setOperationMode(SceneObject::SceneMode mode)
{
    operationMode = mode;
    
    int n = objectGroupNode->getNumChildren();
    for(int i=0; i < n; ++i){
        SceneObject* object = dynamic_cast<SceneObject*>(objectGroupNode->getChild(i));
        if(object){
            object->sceneMode_ = mode;
            object->onSceneModeChanged();
        }
    }
}


void SceneViewImpl::flipOperationMode()
{
    editModeToggle->setChecked(operationMode == SceneObject::VIEW_MODE);
}


void SceneViewImpl::initializeCamera()
{
    projectionMode = PERSPECTIVE;
    orgOrthoProjectionMatrix.makeOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    orgPerspectiveProjectionMatrix.makePerspective(40.0, 1.0, 1.0, 100.0);
    
    numOverriddenCameraControls = 0;
    camera = viewer->getCamera();
    latestEvent.camera_ = camera;
    //camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setClearColor(osg::Vec4(0.1, 0.1, 0.3, 1.0));
    homeViewMatrix.makeLookAt(osg::Vec3d(4.0, 2.0, 1.5), osg::Vec3d(0.0, 0.0, 1.0), osg::Vec3d(0.0, 0.0, 1.0));
    camera->setViewMatrix(homeViewMatrix);

    resetCameraProjection();
}


void SceneViewImpl::resetCameraProjection()
{
    if(numOverriddenCameraControls > 0){
        return;
    }

    if(projectionMode == PERSPECTIVE){
        camera->setProjectionMatrix(orgPerspectiveProjectionMatrix);
    } else {
        camera->setProjectionMatrix(orgOrthoProjectionMatrix);
    }
    
    correctCameraProjectionAspectRatio();
}


void SceneViewImpl::correctCameraProjectionAspectRatio()
{
    double w = viewer->width();
    double h = viewer->height();

    if(projectionMode == PERSPECTIVE){
        double fovy, aspect, zNear, zFar;
        camera->getProjectionMatrixAsPerspective(fovy, aspect, zNear, zFar);
        camera->setProjectionMatrixAsPerspective(fovy, (w / h), zNear, zFar);
    } else {
        double left, right, bottom, top, zNear, zFar;
        camera->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
        double hw = (right - left) / 2.0;
        double hh = (h / w) * hw;
        camera->setProjectionMatrixAsOrtho(-hw, hw, -hh, hh, zNear, zFar);
    }
}


void SceneViewImpl::viewAll()
{
    osg::BoundingSphere bsphere = objectGroupNode->getBound();
    if(bsphere.radius() <= 0.0){
        bsphere.set(osg::Vec3d(0.0, 0.0, 0.0), 3.0);
    }

    osg::Vec3d eye, center, up;
    camera->getViewMatrixAsLookAt(eye, center, up);
    osg::Matrixd T = camera->getViewMatrix();
    osg::Vec3d p = osg::Matrixd::transform3x3(bsphere.center() - eye, T);
    double left, right, bottom, top, zNear, zFar;
    
    if(projectionMode == PERSPECTIVE){
        camera->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
        p[2] += 2.0 * bsphere.radius() * zNear / (right - left);

    } else {
        camera->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
        double z = bsphere.center()[2];
        double r = bsphere.radius();
        double s = r * 2.0 / (right - left);
        camera->setProjectionMatrixAsOrtho(left * s, right * s, bottom * s, top * s, z + r, z - r);
        p[2] = 0.0;
    }

    osg::Vec3d p2 = osg::Matrixd::transform3x3(T, p);
    T.preMultTranslate(-p2);
    camera->setViewMatrix(T);

    viewer->requestRedraw();
}


void SceneViewImpl::setupHUD()
{
    hudCamera = new osg::Camera;
    hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudCamera->setViewMatrix(osg::Matrix::identity());
    hudCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
    hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
    hudCamera->setAllowEventFocus(false);
    osg::StateSet* stateset = hudCamera->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // sample of putting a text
    if(false){
        osg::Geode* geode = new osg::Geode();
	
        osgText::Text* text = new  osgText::Text;
        osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");
        //font->setMagFilterHint(osg::Texture::NEAREST);
        text->setFont(font);
        //text->setBackdropType(osgText::Text::NONE);
        //text->setCharacterSizeMode(osgText::TextBase::OBJECT_COORDS);
        //text->setCharacterSize(16);
        text->setPosition(osg::Vec3(60.0f, 20.0f, 0.0f));
        text->setText("Head Up Displays are simple :-)");
        text->update();
        
        geode->addDrawable(text);
        
        if(false){
            osg::BoundingBox bb;
            for(unsigned int i=0;i<geode->getNumDrawables();++i){
                bb.expandBy(geode->getDrawable(i)->getBound());
            }
            
            osg::Geometry* geom = new osg::Geometry;
            
            osg::Vec3Array* vertices = new osg::Vec3Array;
            float depth = bb.zMin()-0.1;
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
            geom->setVertexArray(vertices);
            
            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            geom->setNormalArray(normals);
            geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
            
            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
            geom->setColorArray(colors);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            
            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
            
            osg::StateSet* stateset = geom->getOrCreateStateSet();
            stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
            //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
            stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            
            geode->addDrawable(geom);
        }
	
        hudCamera->addChild(geode);
    }
}


osg::Node* SceneViewImpl::createLights()
{
    osg::Light* light = viewer->getLight();
    light->setDiffuse(osg::Vec4(0.7f,0.7f,0.7f,1.0f));
    light->setSpecular(osg::Vec4(0.7f,0.7f,0.7f,1.0f));
    
    // lights
    osg::Group* lightGroup (new osg::Group);
    osg::ref_ptr<osg::StateSet> lightSS (rootNode->getOrCreateStateSet());
    osg::ref_ptr<osg::LightSource> lightSource1 = new osg::LightSource;
    osg::ref_ptr<osg::LightSource> lightSource2 = new osg::LightSource;
    
    // create a local light.
    osg::Vec4f lightPosition (osg::Vec4f(0.0, 0.0, 10.0, 1.0f)); // point light
    //osg::Vec4f lightPosition(osg::Vec4f(0.0, -1.0, 0.0, 0.0f)); // directional light
    //osg::Vec4f lightPosition(osg::Vec4f(1.0, 0.0, 0.0, 0.0f)); // directional light
    osg::ref_ptr<osg::Light> myLight = new osg::Light;
    myLight->setLightNum(1);
    myLight->setPosition(lightPosition);
    //myLight->setAmbient(osg::Vec4(0.2f,0.2f,0.2f,1.0f));
    myLight->setAmbient(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    myLight->setDiffuse(osg::Vec4(0.7f,0.7f,0.7f,1.0f));
    //myLight->setDiffuse(osg::Vec4(0.6f,0.6f,0.6f,1.0f));
    
    myLight->setSpecular(osg::Vec4(0.7f,0.7f,0.7f,1.0f));
    //myLight->setSpecular(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
    //myLight->setSpecular(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    
    myLight->setConstantAttenuation(1.0f);
    lightSource1->setLight(myLight.get());
    
    lightSource1->setLocalStateSetModes(osg::StateAttribute::ON); 
    lightSource1->setStateSetModes(*lightSS,osg::StateAttribute::ON);
    //osg::StateSet* lightSS (lightGroup->getOrCreateStateSet());
    
    // create a local light.
    osg::Vec4f lightPosition2 (osg::Vec4f(2.0,-1.0,3.0,1.0f));
    osg::ref_ptr<osg::Light> myLight2 = new osg::Light;
    myLight2->setLightNum(2);
    myLight2->setPosition(lightPosition2);
    myLight2->setAmbient(osg::Vec4(0.2f,0.2f,0.2f,1.0f));
    myLight2->setDiffuse(osg::Vec4(0.4f,0.1f,0.1f,1.0f));
    myLight2->setConstantAttenuation(1.0f);
    
    lightSource2->setLight(myLight2.get());
    lightSource2->setLocalStateSetModes(osg::StateAttribute::ON); 
    lightSource2->setStateSetModes(*lightSS,osg::StateAttribute::ON);
    
    lightGroup->addChild(lightSource1.get());
    //lightGroup->addChild(lightSource2.get());

    return lightGroup;
}


void SceneViewImpl::setupCoordinateAxes()
{
    coordinateAxesEnabled = false;
    axesTransform = new osg::PositionAttitudeTransform();
    
    if(ENABLE_PERSPECTIVE_COORDINATE_AXES){
        axesTransform->addChild(createAxes(0.1, 0.01));
    } else {
        axesTransform->setPosition(osg::Vec3d(26.0, 24.0, 0.0));
        axesTransform->addChild(createAxes(16.0, 2.0));
    }

    enableCoordinateAxes(true, false);
}


osg::Node* SceneViewImpl::createAxes(float length, float width)
{
    osg::Cone* cones[3];
    osg::Cylinder* cylinders[3];

    for(int i=0; i < 3; ++i){
        cones[i] = new osg::Cone();
        cones[i]->setRadius(width * 2.0);
        cones[i]->setHeight(width * 2.0 * 2.0);

        cylinders[i] = new osg::Cylinder();
        cylinders[i]->setRadius(width);
        cylinders[i]->setHeight(length);
    }

    // x
    cones[0]->setRotation(osg::Quat(1.571f, osg::Vec3(0.0f, 1.0f, 0.0f)));
    cones[0]->setCenter(osg::Vec3(length, 0.0f, 0.0f));
    cylinders[0]->setRotation(osg::Quat(1.571f, osg::Vec3(0.0f, 1.0f, 0.0f)));
    cylinders[0]->setCenter(osg::Vec3(length / 2.0f, 0.0f, 0.0f));

    // y
    cones[1]->setRotation(osg::Quat(1.571f, osg::Vec3(-1.0f, 0.0f, 0.0f)));
    cones[1]->setCenter(osg::Vec3(0.0f, length, 0.0f));
    cylinders[1]->setRotation(osg::Quat(1.571f, osg::Vec3(-1.0f, 0.0f, 0.0f)));
    cylinders[1]->setCenter(osg::Vec3(0.0f, length / 2.0f, 0.0f));

    // z
    cones[2]->setCenter(osg::Vec3(0.0f, 0.0f, length));
    cylinders[2]->setCenter(osg::Vec3(0.0f, 0.0f, length / 2.0f));

    osg::Geode* geode = new osg::Geode;

    float c[][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.4f, 0.6f, 1.0f } };

    for(int i=0; i < 3; ++i){
        osg::Vec4 color(c[i][0], c[i][1], c[i][2], 1.0f);

        osg::ShapeDrawable* dCone = new osg::ShapeDrawable(cones[i]);
        dCone->setColor(color);
        geode->addDrawable(dCone);

        osg::ShapeDrawable* dCylinder = new osg::ShapeDrawable(cylinders[i]);
        dCylinder->setColor(color);
        geode->addDrawable(dCylinder);
    }

    geode->setDataVariance(osg::Object::STATIC);

    return geode;
}


void SceneViewImpl::enableCoordinateAxes(bool on, bool doRequestRedraw)
{
    if(coordinateAxesEnabled != on){
        if(on){
            if(ENABLE_PERSPECTIVE_COORDINATE_AXES){
                rootNode->addChild(axesTransform.get());
            } else {
                hudCamera->addChild(axesTransform.get());
            }
            updateAxesPosition();
        } else {
            if(ENABLE_PERSPECTIVE_COORDINATE_AXES){
                rootNode->removeChild(axesTransform.get());
            } else {
                hudCamera->removeChild(axesTransform.get());
            }
        }
        coordinateAxesEnabled = on;

        if(doRequestRedraw){
            viewer->requestRedraw();
        }
    }
}


void SceneViewImpl::updateAxesPosition()
{
    if(coordinateAxesEnabled){
        if(ENABLE_PERSPECTIVE_COORDINATE_AXES){
            osg::Vec3d p(0.3, -0.3, -1.5);
            axesTransform->setPosition(p * camera->getInverseViewMatrix());
        } else {
            axesTransform->setAttitude(camera->getViewMatrix().getRotate());
        }
    }
}


void SceneViewImpl::setupFloorGrid()
{
    floorGridNode = new osg::Geode;
    floorGridNode->setDataVariance(osg::Object::STATIC);

    osg::StateSet* state = floorGridNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    //state->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    //state->setMode(GL_BLEND, osg::StateAttribute::ON);
    osg::Material* mat = new osg::Material;
    osg::Vec4 color(0.9f, 0.9f, 0.9f, 1.0f);
    mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    state->setAttribute(mat);

    updateFloorGridLines();

    switchNode->addChild(floorGridNode.get());
}


void SceneViewImpl::updateFloorGridLines()
{
    int prevNumDrawables = floorGridNode->getNumDrawables();
    if(prevNumDrawables > 0){
        floorGridNode->removeDrawables(0, prevNumDrawables);
    }

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* v = new osg::Vec3Array;
    geom->setVertexArray(v);

    double half = setup.floorGridSpanSpin.value() / 2.0;
    double interval = setup.floorGridIntervalSpin.value();

    double i = 0.0f;
    double x = 0.0f;
    do {
        x = i * interval;
        // y-line
        v->push_back(osg::Vec3( x, -half, 0.0f));
        v->push_back(osg::Vec3( x,  half, 0.0f));
        v->push_back(osg::Vec3(-x, -half, 0.0f));
        v->push_back(osg::Vec3(-x,  half, 0.0f));
        // x-line
        v->push_back(osg::Vec3(-half,  x, 0.0f));
        v->push_back(osg::Vec3( half,  x, 0.0f));
        v->push_back(osg::Vec3(-half, -x, 0.0f));
        v->push_back(osg::Vec3( half, -x, 0.0f));
        ++i;
    } while(x < half);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));

    floorGridNode->addDrawable(geom);

    if(prevNumDrawables > 0){
        viewer->requestRedraw();
    }
}


OsgViewer* SceneView::viewer()
{
    return impl->viewer;
}


void SceneView::updateIndicator(const std::string& text)
{
    impl->indicatorLabel->setText(text.c_str());
}


/**
   @note The type of 'object' should not be a ref_ptr type but a plain pointer type
   because osg::ref_ptr does not seem to support the automatic conversion
   to the ref_ptrs of the super classes.
*/
void SceneView::addSceneObject(SceneObject* object)
{
    impl->addSceneObject(object);
}


void SceneViewImpl::addSceneObject(SceneObject* object)
{
    objectGroupNode->addChild(object);

    object->sigRedrawRequest.connect(bind(&SceneViewImpl::onSceneObjectRedrawRequest, this, _1));

    if(object->getNumParents() == 1){
        object->onAttachedToScene();
        object->isActive_ = true;
    }

    if(object->sceneMode_ != operationMode){
        object->sceneMode_ = operationMode;
        object->onSceneModeChanged();
    }
}


/**
   @note The type of 'object' should not be a ref_ptr type but a plain pointer type
   because osg::ref_ptr does not seem to support the automatic conversion
   to the ref_ptrs of the super classes.
*/
void SceneView::removeSceneObject(SceneObject* object)
{
    impl->removeSceneObject(object);
}


void SceneViewImpl::removeSceneObject(SceneObject* object)
{
    if(object == pointedObject){
        pointedObject = 0;
    }
    
    objectGroupNode->removeChild(object);

    object->sigRedrawRequest.disconnect(bind(&SceneViewImpl::onSceneObjectRedrawRequest, this, _1));

    if(object->getNumParents() == 0){
        object->onDetachedFromScene();
        object->isActive_ = false;
    }
}


void SceneViewImpl::onSceneObjectRedrawRequest(int flags)
{
    if(flags & SceneObject::EFFECT_NODE_CREATED){
        //viewer->requestRedrawWhenEffectNodeCreated();
        viewer->requestRedraw();
    } else {
        viewer->requestRedraw();
    }
}


void SceneView::requestRedraw()
{
    impl->updateAxesPosition(); /// \todo move this ?
    impl->viewer->requestRedraw();
}


void SceneViewImpl::setHomeView()
{
    camera->setViewMatrix(homeViewMatrix);
    updateAxesPosition();
    viewer->requestRedraw();
}


bool SceneViewImpl::onViewerFocusOut(QFocusEvent* event)
{
    return false;
}


bool SceneViewImpl::handleOsgEvent(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& action)
{
    bool handled = true;
    
    switch(event.getEventType()){

    case osgGA::GUIEventAdapter::RESIZE:
        handleOsgResizeEvent(event);
        break;

    default:
        handled = false;
        break;
    }

    return handled;
}


void SceneViewImpl::handleOsgResizeEvent(const osgGA::GUIEventAdapter& event)
{
    drawingAreaHeight = event.getWindowHeight();
    
    // just the area of the axis
    hudCamera->setProjectionMatrixAsOrtho(0, 50, 0, 50, -20.0, 20.0);
    hudCamera->setViewport(0, 0, 50, 50);

    correctCameraProjectionAspectRatio();
}


void SceneViewImpl::updateLatestEvent(QKeyEvent* event)
{
    latestEvent.modKeyMask_ = OsgWidget::convertToOsgModKeyMask(event->modifiers());
    latestEvent.key_ = OsgWidget::convertToOsgKeySymbol(event->key());
}


void SceneViewImpl::updateLatestEvent(int x, int y, int modifiers)
{
    latestEvent.x_ = x;
    latestEvent.y_ = drawingAreaHeight - y;
    latestEvent.modKeyMask_ = OsgWidget::convertToOsgModKeyMask(modifiers);
}


void SceneViewImpl::updateLatestEvent(QMouseEvent* event)
{
    updateLatestEvent(event->x(), event->y(), event->modifiers());
    latestEvent.button_ = OsgWidget::convertToOsgMouseButtonMask(event->button());
}


bool SceneViewImpl::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == viewer){
        switch(event->type()){
        case QEvent::KeyPress:
            return onKeyPressEvent(static_cast<QKeyEvent*>(event));
        case QEvent::KeyRelease:
            return onKeyReleaseEvent(static_cast<QKeyEvent*>(event));
        case QEvent::MouseButtonPress:
            return onMousePressEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonDblClick:
            return onMouseDoubleClickEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonRelease:
            return onMouseReleaseEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return onMouseMoveEvent(static_cast<QMouseEvent*>(event));
        case QEvent::Wheel:
            return onWheelEvent(static_cast<QWheelEvent*>(event));
        case QEvent::FocusOut:
            return onViewerFocusOut(static_cast<QFocusEvent*>(event));
        default:
            return false;
        }
    }
    return false;
}


bool SceneViewImpl::onKeyPressEvent(QKeyEvent* event)
{
    bool handled = false;

    updateLatestEvent(event);
    
    switch(event->key()){

    case Qt::Key_Escape:
        flipOperationMode();
        handled = true;
        break;
        
    case Qt::Key_Z:
        if(event->modifiers() & Qt::ControlModifier){
            if(pointedObject){
                if(event->modifiers() & Qt::ShiftModifier){
                    pointedObject->onRedoRequest();
                } else {
                    pointedObject->onUndoRequest();
                }
            }
            handled = true;
        }
        break;

    default:
        break;
    }

    if(!handled && operationMode == SceneObject::EDIT_MODE){
        if(pointedObject){
            if(pointedObject->onKeyPressEvent(latestEvent)){
                handled = true;
            }
        }
    }

    return handled;
}


bool SceneViewImpl::onKeyReleaseEvent(QKeyEvent* event)
{
    updateLatestEvent(event);
    
    bool handled = false;

    if(dragMode == VIEW_ZOOM && (event->key() == Qt::Key_Control)){
        dragMode = NO_DRAGGING;
    }

    if(operationMode == SceneObject::EDIT_MODE){
        if(pointedObject){
            handled = pointedObject->onKeyReleaseEvent(latestEvent);
        }
    }

    return handled;
}


bool SceneViewImpl::onMousePressEvent(QMouseEvent* event)
{
    updateLatestEvent(event);
    
    SceneObject* object = findPointedObject();

    bool handled = false;

    SceneObject::SceneMode tmpOperationMode = operationMode;
    if(event->modifiers() & Qt::AltModifier){
        tmpOperationMode = SceneObject::VIEW_MODE;
    }
    
    if(tmpOperationMode == SceneObject::EDIT_MODE){
        if(pointedObject && pointedObject != object){
            handled = pointedObject->onButtonPressEvent(latestEvent);
        }
        if(event->button() == Qt::RightButton){
            showEditModePopupMenu(object, event->globalPos());
            handled = true;
        } else if(object){
            if(object->onButtonPressEvent(latestEvent)){
                handled = true;
                pointedObject = object;
                dragMode = OBJECT_DELEGATION;
            }
        }
    }

    if(tmpOperationMode == SceneObject::VIEW_MODE || !handled){
            
        switch(event->button()){
                
        case Qt::LeftButton:
            startViewRotation();
            break;
                
        case Qt::MidButton:
            if(event->modifiers() & Qt::ControlModifier){
                startViewZoom();
            } else {
                startViewTranslation();
            }
            break;
                
        case Qt::RightButton:
            if(operationMode == SceneObject::VIEW_MODE){
                showViewModePopupMenu(event->globalPos());
            }
            break;
                
        default:
            break;
        }
    }
        
    return true;
}


bool SceneViewImpl::onMouseDoubleClickEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    SceneObject* object = findPointedObject();
    bool handled = false;
    
    if(operationMode == SceneObject::EDIT_MODE && object){
        handled = object->onDoubleClickEvent(latestEvent);
    }
    if(!handled){
        flipOperationMode();
    }

    return true;
}


bool SceneViewImpl::onMouseReleaseEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    if(operationMode == SceneObject::EDIT_MODE){
        if(pointedObject){
            pointedObject->onButtonReleaseEvent(latestEvent);
        }
    }
    
    dragMode = NO_DRAGGING;

    return true;
}


bool SceneViewImpl::onMouseMoveEvent(QMouseEvent* event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneViewImpl::onMouseMoveEvent()" << endl;
    }

    updateLatestEvent(event->x(), event->y(), event->modifiers());

    switch(dragMode){
        
    case OBJECT_DELEGATION:
        pointedObject->onPointerMoveEvent(latestEvent);
        break;
        
    case VIEW_ROTATION:
        dragViewRotation(event);
        break;
        
    case VIEW_TRANSLATION:
        dragViewTranslation();
        break;
        
    case VIEW_ZOOM:
        dragViewZoom();
        break;
        
    default:
        updatePointerPosition();
        break;
    }

    return true;
}


void SceneViewImpl::updatePointerPosition()
{
    if(TRACE_FUNCTIONS){
        os << "SceneViewImpl::updatePointerPosition()" << endl;
    }
    
    SceneObject* object = findPointedObject();
    
    if(operationMode == SceneObject::EDIT_MODE){

        if(pointedObject && (pointedObject != object)){
            pointedObject->onPointerLeaveEvent(latestEvent);
            //pointedObject = 0;
        }
        if(object){
            if(object->onPointerMoveEvent(latestEvent)){
                pointedObject = object;
                if(!viewer->hasFocus()){
                    viewer->setFocus(Qt::MouseFocusReason);
                }
            }
        }
    } else {
        static boost::format f(_("Position = (%.3f %.3f %.3f)"));
        const osg::Vec3d& p = latestEvent.point();
        self->updateIndicator(str(f % p.x() % p.y() % p.z()));
    }
}


bool SceneViewImpl::onWheelEvent(QWheelEvent* event)
{
    updateLatestEvent(event->x(), event->y(), event->modifiers());
    latestEvent.scrollingMotion_ = OsgWidget::convertToOsgScrollingMotion(event);
    latestEvent.scrollingDelta_ = static_cast<double>(event->delta());

    bool handled = false;

    if(operationMode == SceneObject::EDIT_MODE){
        SceneObject* object = findPointedObject();
        if(object && object->onScrollEvent(latestEvent)){
            pointedObject = object;
            handled = true;
        }
    }    
    
    if(operationMode == SceneObject::VIEW_MODE || !handled){

        double r = (event->modifiers() & Qt::ShiftModifier) ? 0.05 : 0.25;

        switch(latestEvent.scrollingMotion()){
        case osgGA::GUIEventAdapter::SCROLL_UP:
            zoomView(1.0 + r);
            handled = true;
            break;
        case osgGA::GUIEventAdapter::SCROLL_DOWN:
            zoomView(1.0 - r);
            handled = true;
            break;
        default:
            break;
        }
    }

    return handled;
}


SceneObject* SceneViewImpl::findPointedObject()
{
    if(TRACE_FUNCTIONS){
        os << "SceneViewImpl::findPointedObject(): x, y = " << latestEvent.x() << ", " << latestEvent.y() << endl;
    }
    
    SceneObject* object = 0;
    latestEvent.path_.clear();
    
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if(viewer->computeIntersections(latestEvent.x(), latestEvent.y(), intersections)){
        const osgUtil::LineSegmentIntersector::Intersection& intersection = *intersections.begin();
        if(!intersection.nodePath.empty()){
            bool isPointingIndicator = false;
            const osg::NodePath& path = intersection.nodePath;
            for(size_t i=0; i < path.size(); ++i){
                osg::Node* node = path[i];
                if(node == axesTransform){
                    isPointingIndicator = true;
                    break;
                }
                object = dynamic_cast<SceneObject*>(node);
                if(object){
                    if(object->isEditable()){
                        latestEvent.path_.resize(path.size() - i);
                        std::copy(path.begin() + i, path.end(), latestEvent.path_.begin());
                    } else {
                        object = 0;
                    }
                    break;
                }
            }
            if(!isPointingIndicator){
                latestEvent.point_ = intersection.getWorldIntersectPoint();
                latestEvent.normal_ = intersection.getWorldIntersectNormal();
                if(!object){
                    latestEvent.path_ = path;
                }
            }
        }
    }

    return object;
}


void SceneViewImpl::showViewModePopupMenu(const QPoint& globalPos)
{
    menuManager.releaseAllItems();
    
    menuManager.addItem(_("Edit Mode"))
        ->sigTriggered().connect(bind(&SceneViewImpl::flipOperationMode, this));

    popupMenu->popup(globalPos);
}


void SceneViewImpl::showEditModePopupMenu(SceneObject* object, const QPoint& globalPos)
{
    menuManager.releaseAllItems();
    
    menuManager.setBackwardMode().addItem(_("View Mode"))
        ->sigTriggered().connect(bind(&SceneViewImpl::flipOperationMode, this));
    
    if(object){
        menuManager.addSeparator();
        menuManager.setPath("/");
        object->onContextMenuRequest(latestEvent, menuManager);
    }
    
    popupMenu->popup(globalPos);
}


void SceneViewImpl::onPopupMenuSelectionDone()
{
    menuManager.releaseAllItems();
}


void SceneViewImpl::startViewRotation()
{
    if(numOverriddenCameraControls > 0){
        return;
    }
    
    orgMouseX = latestEvent.x();
    orgMouseY = latestEvent.y();
    orgPointedPos = latestEvent.point();

    dragMode = VIEW_ROTATION;
    orgViewMatrix = camera->getViewMatrix();
    osg::Matrixd inv;
    inv.invert(orgViewMatrix);
    orgEye = inv.getTrans();
    orgViewUp = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(0.0f, 1.0f, 0.0f));
    orgViewSide = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(1.0f, 0.0f, 0.0f));
}


void SceneViewImpl::dragViewRotation(QMouseEvent* event)
{
    static const float angleRatio = 0.01;
    
    double dx = latestEvent.x() - orgMouseX;
    double dy = latestEvent.y() - orgMouseY;

    osg::Matrixd T;
    T.makeTranslate(orgPointedPos);
    osg::Matrixd invT;
    invT.invert(T);
    osg::Matrixd C(orgViewMatrix);
    osg::Matrixd invRu;
    //invRu.makeRotate(dx * angleRatio, orgViewUp);
    invRu.makeRotate(dx * angleRatio, osg::Vec3d(0.0, 0.0, 1.0));
    osg::Matrixd invRs;
    invRs.makeRotate(-dy * angleRatio, orgViewSide);

    T.preMult(invRs);
    T.preMult(invRu);
    T.preMult(invT);

    osg::Camera* camera = viewer->getCamera();
    camera->setViewMatrix(T * orgViewMatrix);

    // snap rotation
    if(event->modifiers() & Qt::ShiftModifier){
        osg::Matrixd& T = camera->getViewMatrix();
        Matrix3 R;
        R << T(0,0), T(1,0), T(2,0),
             T(0,1), T(1,1), T(2,1),
             T(0,2), T(1,2), T(2,2);
        Vector3 rpy = rpyFromRot(R);
        for(int i=0; i < 3; ++i){
            double& a = rpy[i];
            for(int j=0; j < 5; ++j){
                double b = j * (PI / 2.0) - PI;
                if(fabs(b - a) < (PI / 8)){
                    a = b;
                    break;
                }
            }
        }
        R = rotFromRpy(rpy);
        for(int i=0; i < 3; ++i){
            for(int j=0; j < 3; ++j){
                T(i,j) = R(j,i);
            }
        }
        camera->setViewMatrix(T);
    }

    updateAxesPosition();
    viewer->requestRedraw();
}


void SceneViewImpl::startViewTranslation()
{
    if(numOverriddenCameraControls > 0){
        return;
    }

    osg::Viewport* viewport = camera->getViewport();
    double r = 1.0;
    double left, right, bottom, top, zNear, zFar;
            
    if(projectionMode == PERSPECTIVE){
        osg::Vec3d eye, center, up;
        camera->getViewMatrixAsLookAt(eye, center, up);
        camera->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
        osg::Vec3d az = center - eye;
        az.normalize();
        r = ((latestEvent.point() - eye) * az) / zNear;
    } else {
        camera->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
    }
    viewTranslationRatioX = r * (right - left) / viewport->width();
    viewTranslationRatioY = r * (top - bottom) / viewport->height();
    
    orgMouseX = latestEvent.x();
    orgMouseY = latestEvent.y();
    
    dragMode = VIEW_TRANSLATION;
    orgViewMatrix = camera->getViewMatrix();
    orgViewSide = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(1.0f, 0.0f, 0.0f));
    orgViewUp = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(0.0f, 1.0f, 0.0f));
}


void SceneViewImpl::dragViewTranslation()
{
    osg::Camera* camera = viewer->getCamera();

    double dx = viewTranslationRatioX * (latestEvent.x() - orgMouseX);
    double dy = viewTranslationRatioY * (latestEvent.y() - orgMouseY);
    osg::Matrixd invT;
    invT.makeTranslate(orgViewSide * dx + orgViewUp * dy);
    camera->setViewMatrix(invT * orgViewMatrix);

    updateAxesPosition();
    viewer->requestRedraw();
}


void SceneViewImpl::startViewZoom()
{
    if(numOverriddenCameraControls > 0){
        return;
    }

    dragMode = VIEW_ZOOM;
    orgMouseY = latestEvent.y();
    
    if(projectionMode == PERSPECTIVE){
        orgViewMatrix = camera->getViewMatrix();
        orgViewForward = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(0.0f, 0.0f, 1.0f));
        osg::Vec3d eye, center, up;
        camera->getViewMatrixAsLookAt(eye, center, up);
        orgZoomDistance = fabs((latestEvent.point() - eye) * orgViewForward);
    } else {
        orgOrthoProjectionMatrix = camera->getProjectionMatrix();
    }
}


void SceneViewImpl::dragViewZoom()
{
    double dy = latestEvent.y() - orgMouseY;

    if(projectionMode == PERSPECTIVE){
        osg::Matrixd invT;
        invT.makeTranslate(orgViewForward * (orgZoomDistance * (-exp(dy * 0.01) + 1.0)));
        camera->setViewMatrix(invT * orgViewMatrix);
    } else {
        double p[4];
        double zNear, zFar;
        orgOrthoProjectionMatrix.getOrtho(p[0], p[1], p[2], p[3], zNear, zFar);
        double ratio = exp(dy * 0.01);
        for(int i=0; i < 4; ++i){
            p[i] *= ratio;
        }
        camera->setProjectionMatrixAsOrtho(p[0], p[1], p[2], p[3], zNear, zFar);
    }

    correctCameraProjectionAspectRatio();
    updateAxesPosition();
    viewer->requestRedraw();
}


void SceneViewImpl::zoomView(double ratio)
{
    if(numOverriddenCameraControls > 0){
        return;
    }

    if(projectionMode == PERSPECTIVE){
        orgViewMatrix = camera->getViewMatrix();
        orgViewForward = osg::Matrixd::transform3x3(orgViewMatrix, osg::Vec3(0.0f, 0.0f, -1.0f));
        osg::Vec3d eye, center, up;
        camera->getViewMatrixAsLookAt(eye, center, up);
        double dz = (ratio - 1.0) * ((latestEvent.point() - eye) * orgViewForward);
        osg::Matrixd invT;
        invT.makeTranslate(orgViewForward * dz);
        camera->setViewMatrix(invT * orgViewMatrix);
    } else {
        double p[4];
        double zNear, zFar;
        camera->getProjectionMatrixAsOrtho(p[0], p[1], p[2], p[3], zNear, zFar);
        for(int i=0; i < 4; ++i){
            p[i] *= ratio;
        }
        camera->setProjectionMatrixAsOrtho(p[0], p[1], p[2], p[3], zNear, zFar);
    }

    correctCameraProjectionAspectRatio();
    updateAxesPosition();
    viewer->requestRedraw();
}


void SceneViewImpl::onWireframeModeToggled(bool on)
{
    osg::PolygonMode::Mode mode = on ? osg::PolygonMode::LINE : osg::PolygonMode::FILL;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, mode);
    viewer->requestRedraw();
}


void SceneViewImpl::onScribeModeToggled(bool on)
{
    if(on){
        if(!scribe){
            scribe = new osgFX::Scribe();
            scribe->setEnabled(true);
        }
        scribe->addChild(objectGroupNode.get());
        rootNode->replaceChild(objectGroupNode.get(), scribe.get());
    } else {
        if(scribe.valid()){
            rootNode->replaceChild(scribe.get(), objectGroupNode.get());
        }
    }

    //viewer->requestRedrawWhenEffectNodeCreated();
    viewer->requestRedraw();
}


void SceneViewImpl::onCartoonModeToggled(bool on)
{
    if(on){
        if(!cartoon){
            cartoon = new osgFX::Cartoon();
            //cartoon->setOutlineLineWidth(20.0f);
        }
        cartoon->addChild(objectGroupNode.get());
        rootNode->replaceChild(objectGroupNode.get(), cartoon.get());
    } else {
        if(cartoon.valid()){
            rootNode->replaceChild(cartoon.get(), objectGroupNode.get());
        }
    }

    //viewer->requestRedrawWhenEffectNodeCreated();
    viewer->requestRedraw();
}


void SceneView::toggleFloorGrid(bool on)
{
    impl->floorGridToggle->setChecked(on);
}

void SceneViewImpl::onFloorGridToggled(bool on)
{
    switchNode->setChildValue(floorGridNode.get(), on);
    viewer->requestRedraw();
}


void SceneViewImpl::onPerspectiveViewToggled(bool on)
{
    ProjectionMode newProjectionMode = on ? PERSPECTIVE : ORTHO;

    if(newProjectionMode != projectionMode){
        if(projectionMode == PERSPECTIVE){
            orgPerspectiveProjectionMatrix = camera->getProjectionMatrix();
        } else {
            orgOrthoProjectionMatrix = camera->getProjectionMatrix();
        }
        projectionMode = newProjectionMode;
        
        resetCameraProjection();
        viewer->requestRedraw();
    }
}


void SceneView::toggleShadow(bool on)
{
    impl->shadowCheck->setChecked(on);
}


void SceneViewImpl::onShadowToggled(bool on)
{
    const bool UseShadowMap = false;
    const int ReceivesShadowTraversalMask = 0x1;
    const int CastsShadowTraversalMask = 0x2;
    
    if(on){
        if(!shadowedScene){
            shadowedScene = new osgShadow::ShadowedScene();
            shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
            shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);

            if(UseShadowMap){
                shadowMap = new osgShadow::ShadowMap();
                shadowedScene->setShadowTechnique(shadowMap.get());
                int mapres = 1024;
                shadowMap->setTextureSize(osg::Vec2s(mapres, mapres));
            } else {
                osgShadow::ShadowTexture* st = new osgShadow::ShadowTexture();
                shadowedScene->setShadowTechnique(st);
            }
        }
        shadowedScene->addChild(objectGroupNode.get());
        rootNode->replaceChild(objectGroupNode.get(), shadowedScene.get());
        if(!UseShadowMap){
            objectGroupNode->getChild(0)->setNodeMask(CastsShadowTraversalMask);
            objectGroupNode->getChild(1)->setNodeMask(ReceivesShadowTraversalMask);
        } else {
            osg::ref_ptr<osg::Material> matirial = new osg::Material;
            matirial->setColorMode(osg::Material::DIFFUSE);
            matirial->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
            matirial->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
            matirial->setShininess(osg::Material::FRONT_AND_BACK, 64);
            objectGroupNode->getOrCreateStateSet()->setAttributeAndModes(matirial.get(), osg::StateAttribute::ON);
        }
        //objectGroupNode->setNodeMask(ReceivesShadowTraversalMask);
    } else {
        if(shadowedScene.valid()){
            rootNode->replaceChild(shadowedScene.get(), objectGroupNode.get());
        }
    }

    viewer->frame();
    viewer->requestRedraw();
}


void SceneViewImpl::onCollisionToggled(bool on)
{
    viewer->isCollisionVisibleMode_ = on;
    viewer->requestRedraw();
}


void SceneViewImpl::onOverPaintToggled(bool on)
{
    if(on){
        camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    } else {
        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    viewer->requestRedraw();
}


osg::Camera* SceneView::getCameraControl()
{
    impl->numOverriddenCameraControls++;
    return impl->camera.get();

}


bool SceneView::releaseCameraControl(osg::Camera* camera)
{
    impl->numOverriddenCameraControls--;

    if(impl->numOverriddenCameraControls >= 0){
        return true;
    } else {
        impl->numOverriddenCameraControls = 0;
        return false;
    }
}


void SceneView::addPreprocessingNode(osg::Node* node)
{
    if(!impl->preprocessingNodeGroup->containsNode(node)){
        impl->preprocessingNodeGroup->addChild(node);
    }
}


bool SceneView::removePreprocessingNode(osg::Node* node)
{
    return impl->preprocessingNodeGroup->removeChild(node);
}


void SceneViewImpl::onHiPriorityRenderingToggled(bool on)
{
    //viewer->enableHiPriorityRendering(on);
}


void SceneViewImpl::onCaptureImageButtonClicked()
{
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Save an image of the scene view"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    
    QStringList filters;
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);
    
    dialog.setDirectory(lastCaptureFolder);
    if(!lastCaptureFile.isEmpty()){
        dialog.selectFile(lastCaptureFile);
    }

    if(dialog.exec()){
        QString filename = dialog.selectedFiles().front();
        if(!filename.isEmpty()){
            saveImage(filename.toLocal8Bit().data());
            lastCaptureFile = filename;
            lastCaptureFolder = dialog.directory().absolutePath();
        }
    }
}


bool SceneView::saveImage(const std::string& filename)
{
    return impl->saveImage(filename);
}


bool SceneViewImpl::saveImage(const std::string& filename)
{
    const bool USE_READ_PIXELS = true;

    osg::ref_ptr<osg::Image> image = new osg::Image;

    int width = viewer->width();
    int height = viewer->height();
    
    if(!USE_READ_PIXELS){
        image->allocateImage(width, height, 24, GL_RGB, GL_UNSIGNED_BYTE);
        camera->attach(osg::Camera::COLOR_BUFFER, image.get());
    }
    
    viewer->graphicsWindow()->makeCurrentImplementation();
    viewer->renderingTraversals();

    if(USE_READ_PIXELS){
        image->readPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE);
    }

    return osgDB::writeImageFile(*image, filename);
}


void SceneView::setScreenSize(int width, int height)
{
    impl->setScreenSize(width, height);
}


void SceneViewImpl::setScreenSize(int width, int height)
{
    QRect r = self->geometry();
    viewer->setGeometry((r.width() - width) / 2, (r.height() - height) / 2, width, height);
}


bool SceneView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool SceneViewImpl::storeState(Archive& archive)
{
    archive.write("mode", (operationMode == SceneObject::VIEW_MODE) ? "view" : "edit");
    archive.write("floorGird", floorGridToggle->isChecked());
    archive.write("collisions", collisionToggle->isChecked());
    archive.write("shadow", shadowCheck->isChecked());

    setup.storeState(archive);

    YamlMapping& cameraData = *archive.createMapping("camera");

    if(projectionMode == PERSPECTIVE){
        cameraData.write("projection", "perspetive");
        orgPerspectiveProjectionMatrix = camera->getProjectionMatrix();
    } else {
        cameraData.write("projection", "ortho");
        orgOrthoProjectionMatrix = camera->getProjectionMatrix();
    }
    double p[6];
    if(orgPerspectiveProjectionMatrix.getPerspective(p[0], p[1], p[2], p[3])){
        YamlSequence& perspective = *cameraData.createFlowStyleSequence("perspective");
        for(int i=0; i < 4; ++i){
            perspective.append(p[i]);
        }
    }
    if(orgOrthoProjectionMatrix.getOrtho(p[0], p[1], p[2], p[3], p[4], p[5])){
        YamlSequence& ortho = *cameraData.createFlowStyleSequence("ortho");
        for(int i=0; i < 6; ++i){
            ortho.append(p[i]);
        }
    }

    const char* vnames[] = { "eye", "center", "up" };
    osg::Vec3f v[3];
    double distance = 1.0;
    // use the float version because the older OSG does not have the double version
    camera->getViewMatrixAsLookAt(v[0], v[1], v[2], distance); 

    for(int i=0; i < 3; ++i){
        YamlSequence& vs = *cameraData.createFlowStyleSequence(vnames[i]);
        for(int j=0; j < 3; ++j){
            vs.append(v[i][j]);
        }
    }
            
    return true;
}


bool SceneView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool SceneViewImpl::restoreState(const Archive& archive)
{
    editModeToggle->setChecked(archive.get("mode", "view") == "edit");

    bool on;
    if(archive.read("shadow", on)){
        self->toggleShadow(on);
    }
    collisionToggle->setChecked(archive.get("collisions", collisionToggle->isChecked()));
    floorGridToggle->setChecked(archive.get("floorGrid", floorGridToggle->isChecked()));
    
    setup.restoreState(archive);
    
    const YamlMapping& cameraData = *archive.findMapping("camera");
    if(cameraData.isValid()){

        const char* vnames[] = { "eye", "center", "up" };
        osg::Vec3d v[3];

        int i;
        for(i=0; i < 3; ++i){
            const YamlSequence& vs = *cameraData.findSequence(vnames[i]);
            if(vs.isValid() && vs.size() == 3){
                for(int j=0; j < 3; ++j){
                    v[i][j] = vs[j].toDouble();
                }
            } else {
                break;
            }
        }
        if(i == 3){
            camera->setViewMatrixAsLookAt(v[0], v[1], v[2]);
        }

        string projection;
        if(cameraData.read("projection", projection)){
            if(projection == "perspective"){
                projectionMode = PERSPECTIVE;
                perspectiveToggle->setChecked(true);
            } else if(projection == "ortho"){
                projectionMode = ORTHO;
                perspectiveToggle->setChecked(false);
            }
        }
        const YamlSequence& pers = *cameraData.findSequence("perspective");
        if(pers.isValid() && pers.size() == 4){
            orgPerspectiveProjectionMatrix.makePerspective(
                pers[0].toDouble(), pers[1].toDouble(), pers[2].toDouble(), pers[3].toDouble());
        }
        const YamlSequence& o = *cameraData.findSequence("ortho");
        if(o.isValid() && o.size() == 6){
            orgOrthoProjectionMatrix.makeOrtho(
                o[0].toDouble(), o[1].toDouble(), o[2].toDouble(), o[3].toDouble(), o[4].toDouble(), o[5].toDouble());
        }
        resetCameraProjection();
        
        viewer->requestRedraw();
    }

    return true;
}
