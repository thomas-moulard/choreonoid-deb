/**
   @author Shin'ichiro Nakaoka
*/

#include "MainWindow.h"
#include "App.h"
#include "View.h"
#include "ToolBar.h"
#include "ToolBarArea.h"
#include "InfoBar.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include <cnoid/YamlNodes>
#include <QSplitter>
#include <QTabWidget>
#include <QTabBar>
#include <QMouseEvent>
#include <QApplication>
#include <QBoxLayout>
#include <QRubberBand>
#include <QFileDialog>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/filesystem.hpp>
#include <set>
#include <bitset>
#include <iostream>

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    const bool TRACE_FUNCTIONS = false;

    MainWindow* mainWindow = 0;

    enum DropArea { OVER = -1, LEFT = 0, TOP, RIGHT, BOTTOM, NUM_DROP_AREAS };
    const int SPLIT_DISTANCE_THRESHOLD = 35;

    class TabWidget : public QTabWidget
    {
    public:
        TabWidget(MainWindowImpl* mwi, QWidget* parent = 0);

        int addView(View* view) {
            return addTab(view, view->windowTitle());
        }
            
        QTabBar* tabBar() const { return QTabWidget::tabBar(); }
        virtual bool eventFilter(QObject* object, QEvent* event);

        // For the whole tab widget dragging
        //virtual void mousePressEvent(QMouseEvent *event);
        //virtual void mouseMoveEvent(QMouseEvent *event);
        //virtual void mouseReleaseEvent(QMouseEvent *event);

        MainWindowImpl* mwi;
    };
}

namespace cnoid {
    
    class MainWindowImpl //: public boost::signals::trackable
    {
    public:
        MainWindow* self;

        MainWindowImpl(MainWindow* self, const char* appName, ExtensionManager* ext);
        ~MainWindowImpl();

        void setupMenus(ExtensionManager* ext);
        void createDefaultPanes();

        bool addView(const std::string& pluginName, View* view);
        bool removeView(View* view);

        void showFirst();
        void onFullScreenToggled(bool on);
        void resizeEvent(QResizeEvent* event);
        void keyPressEvent(QKeyEvent* event);

        bool viewTabMousePressEvent(TabWidget* pane, QMouseEvent* event);
        bool viewTabMouseMoveEvent(TabWidget* pane, QMouseEvent* event);
        bool viewTabMouseReleaseEvent(TabWidget* pane, QMouseEvent *event);
        
        void startViewDrag(View* view);
        void dragView(QMouseEvent* event);
        void dragViewInsidePane(const QPoint& posInDestPane);
        void dropViewInsidePane();
        void dragViewOutsidePane();
        void dropViewOutsidePane();
        void removePaneIfEmpty(TabWidget* pane);
        void clearAllPanes();
        void clearAllPanesSub(QSplitter* splitter);
        void clearEmptyPanes();

        void onSaveLayout();
        void onSaveLayoutAs();
        void storeLayout(YamlMappingPtr layout);
        YamlMapping* storeSplitterState(QSplitter* splitter);
        YamlMapping* storePaneState(TabWidget* pane);
        void onLoadLayout();
        void onLoadLayoutAs();
        void restoreLayout(const YamlMappingPtr layout);
        QWidget* restoreSplitterState(const YamlMapping& state, TabWidget*& out_firstPane);
        TabWidget* restorePaneState(const YamlMapping& state);
        void restoreDefaultLayout();
        
        std::vector<View*> views;

        typedef multimap<QString, View*> NameToViewMap;
        NameToViewMap nameToViewMap;
        typedef set<View*> ViewSet;
        ViewSet storedViews;

        std::vector<ToolBar*> toolBars;

        TabWidget* areaToPane[View::NUM_AREAS];

        struct AreaDetectionInfo {
            AreaDetectionInfo() {
                for(int i=0; i < View::NUM_AREAS; ++i){
                    scores[i] = 0;
                }
            }
            TabWidget* pane;
            int scores[View::NUM_AREAS];
        };

        typedef bitset<NUM_DROP_AREAS> EdgeContactState;

        ToolBarArea* toolBarArea;
        
        QWidget* centralWidget;
        QVBoxLayout* centralVBox;
        QSplitter* topSplitter;

        YamlMappingPtr config;
        YamlMappingPtr initialLayout;

        bool isBeforeShowing;
        bool isBeforeDoingInitialLayout;
        bool isMaximized;
        bool isMaximizedJustBeforeFullScreen;
        bool isFullScreen;
        QSize normalStateSize;
        
        QString currentLayoutFolder;

        Action* fullScreenCheck;

        QPoint tabDragStartPosition;
        bool isViewDragging;
        TabWidget* dragSrcPane;
        TabWidget* dragDestPane;
        bool isViewDraggingOutsidePane;
        int dropEdge;
        QRubberBand* rubberBand;
    };
}


TabWidget::TabWidget(MainWindowImpl* mwi, QWidget* parent)
    : QTabWidget(parent),
      mwi(mwi)
{
    setMovable(true);
    setUsesScrollButtons(true);
    tabBar()->installEventFilter(this);
}


bool TabWidget::eventFilter(QObject* object, QEvent* event)
{
    if(object == tabBar()){
        switch(event->type()){
        case QEvent::MouseButtonPress:
            return mwi->viewTabMousePressEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonDblClick:
            break;
        case QEvent::MouseButtonRelease:
            return mwi->viewTabMouseReleaseEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return mwi->viewTabMouseMoveEvent(this, static_cast<QMouseEvent*>(event));
        default:
            break;
        }
    }
    return false;
}


void MainWindow::initialize(const char* appName, ExtensionManager* ext)
{
    if(!mainWindow){
        new MainWindow(appName, ext);
    }
}


MainWindow* MainWindow::instance()
{
    return mainWindow;
}


MainWindow::MainWindow(const char* appName, ExtensionManager* ext)
{
    mainWindow = this;

    setWindowTitle(appName);
    setFocusPolicy(Qt::WheelFocus);

    impl = new MainWindowImpl(this, appName, ext);
}


MainWindowImpl::MainWindowImpl(MainWindow* self, const char* appName, ExtensionManager* ext)
    : self(self)
{
    isBeforeDoingInitialLayout = true;
    isMaximized = false;
    isViewDragging = false;
    
    centralWidget = new QWidget(self);
    
    centralVBox = new QVBoxLayout(centralWidget);
    centralVBox->setSpacing(0);
    centralVBox->setContentsMargins(0, 0, 0, 0);

    toolBarArea = new ToolBarArea(centralWidget);
    centralVBox->addWidget(toolBarArea);

    //centralVBox->addSpacing(2);
    
    topSplitter = new QSplitter(centralWidget);
    centralVBox->addWidget(topSplitter, 1);
    
    self->setCentralWidget(centralWidget);

    rubberBand = new QRubberBand(QRubberBand::Rectangle, centralWidget);
    rubberBand->hide();

    config = AppConfig::archive()->openMapping("MainWindow");

    createDefaultPanes();

    self->setStatusBar(InfoBar::instance());

    setupMenus(ext);

    initialLayout = config;
    toolBarArea->setInitialLayout(config);

    isBeforeShowing = true;

    if(TRACE_FUNCTIONS){
        cout << "size = (" << self->width() << ", " << self->height() << ")" << endl;
    }

    normalStateSize.setWidth(config->get("width", self->width()));
    normalStateSize.setHeight(config->get("height", self->width()));
}


MainWindow::~MainWindow()
{
    if(impl){
        delete impl;
        impl = 0;
    }
}


MainWindowImpl::~MainWindowImpl()
{
    config->write("fullScreen", self->isFullScreen());
    config->write("maximized", isMaximized);
    config->write("width", normalStateSize.width());
    config->write("height", normalStateSize.height());
}


void MainWindowImpl::setupMenus(ExtensionManager* ext)
{
    MenuManager& mm = ext->menuManager();

    mm.setPath("/" N_("File")).setBackwardMode().addItem(_("Exit"))
        ->sigTriggered().connect(bind(&MainWindow::close, self));

    mm.setPath("/" N_("Edit"));

    mm.setPath("/" N_("View"));
    
    fullScreenCheck = mm.addCheckItem(_("Full Screen"));
    fullScreenCheck->setChecked(config->get("fullScreen", false));
    fullScreenCheck->sigToggled().connect(bind(&MainWindowImpl::onFullScreenToggled, this, _1));

    mm.setPath("/View").setPath(N_("Layout"));
    
    mm.addItem(_("Save Default Layout"))
        ->sigTriggered().connect(bind(&MainWindowImpl::onSaveLayout, this));
    mm.addItem(_("Load Default Layout"))
        ->sigTriggered().connect(bind(&MainWindowImpl::onLoadLayout, this));
    mm.addItem(_("Save Layout As"))
        ->sigTriggered().connect(bind(&MainWindowImpl::onSaveLayoutAs, this));
    mm.addItem(_("Load Layout As"))
        ->sigTriggered().connect(bind(&MainWindowImpl::onLoadLayoutAs, this));

    mm.setPath("/" N_("Tools"));
    mm.setPath("/" N_("Filters"));
    mm.setPath("/" N_("Options"));
    mm.setPath("/").setBackwardMode().setPath(N_("Help"));
}


void MainWindowImpl::createDefaultPanes()
{
    topSplitter->setOrientation(Qt::Horizontal);
    
    QSplitter* vSplitter0 = new QSplitter(Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter0);
    
    areaToPane[View::LEFT_TOP] = new TabWidget(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_TOP]);
    
    areaToPane[View::LEFT_BOTTOM] = new TabWidget(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_BOTTOM]);
    
    QSplitter* vSplitter1 = new QSplitter(Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter1);

    QSplitter* hSplitter1 = new QSplitter(Qt::Horizontal, vSplitter1);
    vSplitter1->addWidget(hSplitter1);
    
    areaToPane[View::BOTTOM] = new TabWidget(this, vSplitter1);
    vSplitter1->addWidget(areaToPane[View::BOTTOM]);
    
    areaToPane[View::CENTER] = new TabWidget(this, hSplitter1);
    hSplitter1->addWidget(areaToPane[View::CENTER]);
    
    areaToPane[View::RIGHT]  = new TabWidget(this, hSplitter1);
    hSplitter1->addWidget(areaToPane[View::RIGHT]);

    QList<int> sizes;
    sizes << 100 << 600;
    topSplitter->setSizes(sizes);
    sizes.last() = 100;
    vSplitter0->setSizes(sizes);
    sizes.last() = 40;
    vSplitter1->setSizes(sizes);
    sizes.last() = 160;
    hSplitter1->setSizes(sizes);
}


#if 0
void MainWindowImpl::detectExistingPaneAreas()
{
    for(int i=0; i < View::NUM_AREAS; ++i){
        areaToPane[i] = 0;
    }

    vector<AreaDetectionInfo> infos;
    EdgeContactState edge;
    edge.set();
    
    updateAreaDetectionInfos(topSplitter, edge, infos);

    if(infos.empty()){
        createDefaultPanes();

    } else {
        areaToPane[View::CENTER] = extractBestAreaMatchPane(infos, View::CENTER);
        areaToPane[View::LEFT]   = extractBestAreaMatchPane(infos, View::LEFT);
        areaToPane[View::RIGHT]  = extractBestAreaMatchPane(infos, View::RIGHT);
        areaToPane[View::BOTTOM] = extractBestAreaMatchPane(infos, View::BOTTOM);
    }
}


void MainWindowImpl::updateAreaDetectionInfos
(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos)
{
    QWidget* childWidgets[2];
    childWidgets[0] = splitter->get_child1();
    childWidgets[1] = splitter->get_child2();
    bool isSingle = !(childWidgets[0] && childWidgets[1]);

    for(int i=0; i < 2; ++i){
        EdgeContactState currentEdge(edge);
        if(!isSingle){
            if(dynamic_cast<HSplitter*>(splitter)){
                currentEdge.reset((i == 0) ? BOTTOM : TOP);
            } else {
                currentEdge.reset((i == 0) ? RIGHT : LEFT);
            }
        }
        if(childWidgets[i]){
            Splitter* childSplitter = dynamic_cast<Splitter*>(childWidgets[i]);
            if(childSplitter){
                updateAreaDetectionInfos(childSplitter, currentEdge, infos);
            } else {
                Pane* pane = dynamic_cast<Pane*>(childWidgets[i]);
                if(pane){
                    AreaDetectionInfo info;
                    info.pane = pane;
                    
                    // calculate scores for area matching
                    static const int offset = 100000000;
                    int width = pane->get_width();
                    int height = pane->get_height();

                    info.scores[View::CENTER] = (4 - currentEdge.count()) * offset + width * height;

                    if(currentEdge.test(LEFT) && !currentEdge.test(RIGHT)){
                        info.scores[View::LEFT] = offset + height;
                    }
                    if(currentEdge.test(RIGHT) && !currentEdge.test(LEFT)){
                        info.scores[View::RIGHT] = offset + height;
                    }
                    if(currentEdge.test(BOTTOM) && !currentEdge.test(TOP)){
                        info.scores[View::BOTTOM] = offset + width;
                    }
                    
                    infos.push_back(info);
                }
            }
        }
    }
}


Pane* MainWindowImpl::extractBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area)
{
    int topScore = 0;
    int topIndex = -1;

    for(int i=0; i < (signed)infos.size(); ++i){
        int s = infos[i].scores[area];
        if(s > topScore){
            topScore = s;
            topIndex = i;
        }
    }

    Pane* pane = 0;
    
    if(topIndex > 0){
        pane = infos[topIndex].pane;
        infos.erase(infos.begin() + topIndex);
    }

    return pane;
}
#endif


bool MainWindow::addView(const std::string& pluginName, View* view)
{
    return impl->addView(pluginName, view);
}


bool MainWindowImpl::addView(const std::string& pluginName, View* view)
{
    if(view->isManagedByMainWindow){
        return false;
    }
    view->isManagedByMainWindow = true;
    
    areaToPane[view->defaultLayoutArea()]->addView(view);
    views.push_back(view);
    nameToViewMap.insert(make_pair(view->name(), view));

    return true;
}


bool MainWindow::removeView(View* view)
{
    return impl->removeView(view);
}


bool MainWindowImpl::removeView(View* view)
{
    bool removed = false;

    if(view && view->isManagedByMainWindow){
        std::remove(views.begin(), views.end(), view);
        
        TabWidget* tab = dynamic_cast<TabWidget*>(view->parentWidget());
        if(tab){
            tab->removeTab(tab->indexOf(view));
        }
        view->isManagedByMainWindow = false;
        view->hide();
        removed = true;
    }

    return removed;
}


void MainWindow::addToolBar(ToolBar* toolbar)
{
    impl->toolBarArea->addToolBar(toolbar);
}


std::vector<View*> MainWindow::allViews()
{
    return impl->views;
}


std::vector<ToolBar*> MainWindow::allToolBars()
{
    return impl->toolBarArea->getAllToolBars();
}


void MainWindow::setInitialLayout(const YamlMappingPtr layout)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindow::setInitialLayout()" << endl;
    }
    if(impl->isBeforeDoingInitialLayout){
        impl->initialLayout = layout;
        impl->toolBarArea->setInitialLayout(layout);
    }
}


void MainWindow::changeEvent(QEvent* event)
{
    if(event->type() == QEvent::WindowStateChange){
        if(TRACE_FUNCTIONS){
            cout << "MainWindow::changeEvent() of WindowStateChange: " << windowState() << endl;
        }
        if(!(windowState() & Qt::WindowFullScreen)){
            impl->isMaximized = (windowState() & Qt::WindowMaximized);
        }
    }
}


void MainWindow::show()
{
    impl->showFirst();
}


void MainWindowImpl::showFirst()
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::showFirst()" << endl;
    }
    if(isBeforeShowing){

        //self->resize(normalStateSize);
        
        if(config->get("fullScreen", false)){
            isMaximizedJustBeforeFullScreen = isMaximized;
            self->showFullScreen();
        } else if(config->get("maximized", true)){
            self->showMaximized();
        } else {
            self->QMainWindow::show();
        }
        isBeforeShowing = false;
    } else {
        self->QMainWindow::show();
    }
}


void MainWindowImpl::onFullScreenToggled(bool on)
{
    if(on){
        if(!self->isFullScreen()){
            isMaximizedJustBeforeFullScreen = isMaximized;
            self->showFullScreen();
        }
    } else {
        if(self->isFullScreen()){
            if(isMaximizedJustBeforeFullScreen){
                self->showMaximized();
            } else {
                self->showNormal();
            }
        }
    }
}


void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    impl->resizeEvent(event);
}


void MainWindowImpl::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::resizeEvent(): size = (";
        cout << event->size().width() << ", " << event->size().height() << ")";
        cout << ", isMaximized = " << (self->windowState() & Qt::WindowMaximized) << ", " << isMaximized;
        cout << ", isVisible = " << self->isVisible() << endl;
    }
    
    if(isBeforeDoingInitialLayout){

        if(!(self->windowState() & (Qt::WindowMaximized | Qt::WindowFullScreen)) || self->isVisible()){

            if(TRACE_FUNCTIONS){
                cout << "MainWindowImpl::resizeEvent(): initializeLayout" << endl;
            }
            
            if(initialLayout){
                restoreLayout(initialLayout);
            } else {
                restoreDefaultLayout();
            }
            initialLayout = 0;
            isBeforeDoingInitialLayout = false;
        }

    } else {
        if(!(self->windowState() &
             (Qt::WindowMinimized | Qt::WindowMaximized | Qt::WindowFullScreen))){ // normal state ?
            normalStateSize = self->size();
        }
    }
}


void MainWindow::keyPressEvent(QKeyEvent* event)
{
    impl->keyPressEvent(event);
}


void MainWindowImpl::keyPressEvent(QKeyEvent* event)
{
    switch(event->key()){
    case Qt::Key_F11:
        fullScreenCheck->toggle();
        break;

    default:
        break;
    }
}


void MainWindowImpl::restoreDefaultLayout()
{
    /*
    detectExistingPaneAreas();
    
    for(ViewSet::iterator p = storedViews.begin(); p != storedViews.end(); ++p){
        
        View* view = *p;
        Pane* pane = areaToPane[view->defaultLayoutArea()];
        if(!pane){
            pane = areaToPane[View::CENTER];
        }
        pane->appendView(view);
    }
    
    storedViews.clear();
    topSplitter->show_all();
    */
}


void MainWindow::restoreLayout(const YamlMappingPtr layout)
{
    impl->restoreLayout(layout);
}


void MainWindowImpl::onLoadLayout()
{
    restoreLayout(config);
}


void MainWindowImpl::onLoadLayoutAs()
{
    QFileDialog dialog(self);
    dialog.setWindowTitle(_("Open a layout"));
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Open"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    QStringList filters;
    filters << _("Layout files (*.conf)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    if(!currentLayoutFolder.isEmpty()){
        dialog.setDirectory(currentLayoutFolder);
    }
    if(dialog.exec()){
        currentLayoutFolder = dialog.directory().absolutePath();
        AppConfig::load(dialog.selectedFiles().front().toStdString());
        config = AppConfig::archive()->openMapping("MainWindow");
        restoreLayout(config);
    }
}


void MainWindowImpl::restoreLayout(const YamlMappingPtr layout)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restoreLayout()" << endl;
    }

    /*
    if(isBeforeDoingInitialLayout){
        self->setInitialLayout(layout);
        return;
    }
    */

    if(!isBeforeDoingInitialLayout){
        toolBarArea->restoreLayout(layout);
    } else {
        toolBarArea->doInitialLayout();
    }
    
    const YamlMappingPtr layoutOfViews = layout->findMapping("layoutOfViews");

    if(layoutOfViews->isValid()){

        clearAllPanes();

        TabWidget* firstPane = 0;

        QWidget* restoredWidget = restoreSplitterState(*layoutOfViews, firstPane);

        topSplitter = dynamic_cast<QSplitter*>(restoredWidget);
        
        if(!topSplitter){
            topSplitter = new QSplitter();
            firstPane = dynamic_cast<TabWidget*>(restoredWidget);
            if(!firstPane){
                firstPane = new TabWidget(this);
            }
            topSplitter->addWidget(firstPane);
        }
        centralVBox->addWidget(topSplitter, 1);

        for(ViewSet::iterator p = storedViews.begin(); p != storedViews.end(); ++p){
            firstPane->addView(*p);
        }
        storedViews.clear();
    }
}


void MainWindowImpl::clearAllPanes()
{
    if(TRACE_FUNCTIONS){
        cout << "clearAllPanes" << endl;
    }
    clearAllPanesSub(topSplitter);
    delete topSplitter;
    topSplitter = 0;
    if(TRACE_FUNCTIONS){
        cout << "End of clearAllPanes" << endl;
    }
}


void MainWindowImpl::clearAllPanesSub(QSplitter* splitter)
{
    if(TRACE_FUNCTIONS){
        cout << "clearAllPanesSub" << endl;
    }
    
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            clearAllPanesSub(childSplitter);
        } else {
            TabWidget* pane = dynamic_cast<TabWidget*>(splitter->widget(i));
            if(pane){
                for(int i=0; i < pane->count(); ++i){
                    View* view = dynamic_cast<View*>(pane->widget(i));
                    if(view){
                        storedViews.insert(view);
                    }
                }
                while(pane->count() > 0){
                    int index = pane->count() - 1;
                    QWidget* view = pane->widget(index);
                    pane->removeTab(index);
                    view->hide();
                    view->setParent(centralWidget);
                }
            }
        }
    }
}  


QWidget* MainWindowImpl::restoreSplitterState(const YamlMapping& state, TabWidget*& out_firstPane)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restoreSplitterState" << endl;
    }
    
    QWidget* restoredWidget = 0;
    QWidget* childWidgets[2] = { 0, 0 };
    
    const YamlSequence& children = *state.findSequence("children");
    if(children.isValid()){
        int numChildren = std::min(children.size(), 2);
        for(int i=0; i < numChildren; ++i){
            if(children[i].isMapping()){
                const YamlMapping& childState = *children[i].toMapping();
                string type;
                if(childState.read("type", type)){
                    if(type == "splitter"){
                        childWidgets[i] = restoreSplitterState(childState, out_firstPane);
                    } else if(type == "pane"){
                        TabWidget* pane = restorePaneState(childState);
                        if(pane){
                            childWidgets[i] = pane;
                            if(!out_firstPane){
                                out_firstPane = pane;
                            }
                        }
                    }
                }
            }
        }

        if(childWidgets[0] && childWidgets[1]){

            QSplitter* splitter = new QSplitter();

            string orientation;
            if(state.read("orientation", orientation)){
                splitter->setOrientation((orientation == "vertical") ? Qt::Vertical : Qt::Horizontal);
            }

            splitter->addWidget(childWidgets[0]);
            splitter->addWidget(childWidgets[1]);

            const YamlSequence& sizes = *state.findSequence("sizes");
            if(sizes.isValid() && sizes.size() == 2){
                QList<int> s;
                int size;
                for(int i=0; i < 2; ++i){
                    if(sizes[i].read(size)){
                        s.push_back(size);
                    }
                }
                splitter->setSizes(s);
            }
            restoredWidget = splitter;
            
        } else {
            for(int i=0; i < 2; ++i){
                if(childWidgets[i]){
                    restoredWidget = childWidgets[i];
                    break;
                }
            }
        }
    }

    return restoredWidget;
}


TabWidget* MainWindowImpl::restorePaneState(const YamlMapping& state)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restorePaneState" << endl;
    }
    
    TabWidget* pane = 0;
    const YamlSequence& viewNames = *state.findSequence("views");
    
    if(viewNames.isValid() && !viewNames.empty()){
        pane = new TabWidget(this);
        QString currentViewName(state.get("current", "").c_str());
        for(int i=0; i < viewNames.size(); ++i){
            if(viewNames[i].isString()){
                NameToViewMap::iterator p, upper_bound;
                tie(p, upper_bound) = nameToViewMap.equal_range(viewNames[i].toString().c_str());
                if(p != upper_bound){
                    View* view = p->second;
                    ViewSet::iterator q = storedViews.find(view);
                    if(q != storedViews.end()){
                        storedViews.erase(q);
                        pane->addView(view);
                        if(view->name() == currentViewName){
                            pane->setCurrentIndex(i);
                        }
                    }
                }
            }
        }
        if(pane->count() == 0){
            delete pane;
            pane = 0;
        }
    }

    return pane;
}


void MainWindow::storeLayout(YamlMappingPtr layout)
{
    impl->storeLayout(layout);
    AppConfig::flush();
}


void MainWindowImpl::onSaveLayout()
{
    storeLayout(config);
}


void MainWindowImpl::onSaveLayoutAs()
{
    QFileDialog dialog(self);
    dialog.setWindowTitle(_("Save a layout"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    
    QStringList filters;
    filters << _("Layout files (*.conf)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);
    
    if(!currentLayoutFolder.isEmpty()){
        dialog.setDirectory(currentLayoutFolder);
    }
    if(dialog.exec()){
        currentLayoutFolder = dialog.directory().absolutePath();
        string filename(dialog.selectedFiles().front().toStdString());
        string ext = filesystem::extension(filesystem::path(filename));
        if(ext != ".conf"){
            filename += ".conf";
        }
        storeLayout(config);
        AppConfig::save(filename);
    }
}


void MainWindowImpl::storeLayout(YamlMappingPtr layout)
{
    try {
        layout->insert("layoutOfViews", storeSplitterState(topSplitter));
        toolBarArea->storeLayout(layout);
    }
    catch(const YamlNode::Exception& ex){
        cout << ex.message() << endl;
    }
}


YamlMapping* MainWindowImpl::storeSplitterState(QSplitter* splitter)
{
    YamlMapping* state = new YamlMapping();
    
    state->write("type", "splitter");

    if(splitter->count() == 2){
        state->write("orientation", (splitter->orientation() == Qt::Vertical) ? "vertical" : "horizontal");
        YamlSequence* sizeSeq = state->createSequence("sizes");
        QList<int> sizes = splitter->sizes();
        for(int i=0; i < sizes.size(); ++i){
            sizeSeq->append(sizes[i]);
        }
    }

    YamlSequence* children = state->createSequence("children");

    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            children->append(storeSplitterState(childSplitter));
        } else {
            TabWidget* pane = dynamic_cast<TabWidget*>(splitter->widget(i));
            if(pane && pane->count() > 0){
                children->append(storePaneState(pane));
            }
        }
    }

    return state;
}


YamlMapping* MainWindowImpl::storePaneState(TabWidget* pane)
{
    YamlMapping* state = new YamlMapping();
    
    state->write("type", "pane");
    
    YamlSequence* views = state->createFlowStyleSequence("views");
    const int n = pane->count();
    for(int i=0; i < n; ++i){
        View* view = dynamic_cast<View*>(pane->widget(i));
        if(view){
            const string name(view->name().toStdString());
            views->append(name, YAML_DOUBLE_QUOTED);
            if(i == pane->currentIndex()){
                state->write("current", name, YAML_DOUBLE_QUOTED);
            }
        }
    }
    return state;
}


void MainWindowImpl::clearEmptyPanes()
{
    
}


bool MainWindowImpl::viewTabMousePressEvent(TabWidget* pane, QMouseEvent* event)
{            
    if(event->button() == Qt::LeftButton){
        tabDragStartPosition = event->pos();
    }
    return false;
}


bool MainWindowImpl::viewTabMouseMoveEvent(TabWidget* pane, QMouseEvent* event)
{
    if(!isViewDragging){
        if(event->buttons() & Qt::LeftButton){
            if((event->pos() - tabDragStartPosition).manhattanLength() > QApplication::startDragDistance()){
                if(!pane->tabBar()->geometry().contains(event->pos())){
                    View* view = dynamic_cast<View*>(pane->currentWidget());
                    if(view){
                        isViewDragging = true;
                        dragSrcPane = pane;
                        startViewDrag(view);
                    }
                }
            }
        }
    } else {
        dragDestPane = 0;
        QWidget* pointed = topSplitter->childAt(topSplitter->mapFromGlobal(event->globalPos()));
        while(pointed){
            dragDestPane = dynamic_cast<TabWidget*>(pointed);
            if(dragDestPane){
                dragView(event);
                break;
            }
            pointed = pointed->parentWidget();
        }
        if(!dragDestPane){
            rubberBand->hide();
        }
    }
    return false;
}


bool MainWindowImpl::viewTabMouseReleaseEvent(TabWidget* pane, QMouseEvent *event)
{
    if(isViewDragging){
        if(dragDestPane){
            if(isViewDraggingOutsidePane){
                dropViewOutsidePane();
            } else {
                dropViewInsidePane();
            }
        }
        QApplication::restoreOverrideCursor();
    }
    isViewDragging = false;
    return false;
}


void MainWindowImpl::startViewDrag(View* view)
{
    QApplication::setOverrideCursor(Qt::ClosedHandCursor);
}


void MainWindowImpl::dragView(QMouseEvent* event)
{
    dropEdge = LEFT;
        
    QPoint p = topSplitter->mapFromGlobal(event->globalPos());
    const int w = topSplitter->width();
    const int h = topSplitter->height();
    
    int distance[4];
    distance[LEFT] = p.x();
    distance[TOP] = p.y();
    distance[RIGHT] = w - p.x();
    distance[BOTTOM] = h - p.y();
        
    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    isViewDraggingOutsidePane = false;
    if(distance[dropEdge] < 8){
        isViewDraggingOutsidePane = true;
        dragViewOutsidePane();
    } else {
        dragViewInsidePane(dragDestPane->mapFromGlobal(event->globalPos()));
    }
}

    
void MainWindowImpl::dragViewInsidePane(const QPoint& posInDestPane)
{
    dropEdge = LEFT;
        
    const int w = dragDestPane->width();
    const int h = dragDestPane->height();
    
    int distance[4];
    distance[LEFT] = posInDestPane.x();
    distance[TOP] = posInDestPane.y();
    distance[RIGHT] = w - posInDestPane.x();
    distance[BOTTOM] = h - posInDestPane.y();
        
    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    QRect r;
    if(SPLIT_DISTANCE_THRESHOLD < distance[dropEdge]){
        r.setRect(0, 0, w, h);
        dropEdge = OVER;
    } else if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }

    r.translate(centralWidget->mapFromGlobal(dragDestPane->mapToGlobal(QPoint(0, 0))));
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void MainWindowImpl::dropViewInsidePane()
{
    View* view = static_cast<View*>(dragSrcPane->currentWidget());
    if(dropEdge == OVER){
        int index = dragDestPane->addView(view);
        dragDestPane->setCurrentIndex(index);
    } else {

        QSize destSize = dragDestPane->size();

        QSplitter* parentSplitter = static_cast<QSplitter*>(dragDestPane->parentWidget());
        QList<int> parentSizes = parentSplitter->sizes();
        QSplitter* newSplitter = new QSplitter(parentSplitter);
        parentSplitter->insertWidget(parentSplitter->indexOf(dragDestPane), newSplitter);
        TabWidget* newTabWidget = new TabWidget(this, newSplitter);

        if(dropEdge == LEFT){
            newSplitter->setOrientation(Qt::Horizontal);
            newSplitter->addWidget(newTabWidget);
            newSplitter->addWidget(dragDestPane);
        } else if(dropEdge == RIGHT){
            newSplitter->setOrientation(Qt::Horizontal);
            newSplitter->addWidget(dragDestPane);
            newSplitter->addWidget(newTabWidget);
        } else if(dropEdge == TOP){
            newSplitter->setOrientation(Qt::Vertical);
            newSplitter->addWidget(newTabWidget);
            newSplitter->addWidget(dragDestPane);
        } else {
            newSplitter->setOrientation(Qt::Vertical);
            newSplitter->addWidget(dragDestPane);
            newSplitter->addWidget(newTabWidget);
        }
        newTabWidget->addView(view);

        int half;
        if(newSplitter->orientation() == Qt::Horizontal){
            half = destSize.height() / 2;
        } else {
            half = destSize.width() / 2;
        }
        QList<int> sizes;
        sizes << half << half;
        newSplitter->setSizes(sizes);

        parentSplitter->setSizes(parentSizes);
    }
    if(dragSrcPane->count() > 0){
        dragSrcPane->setCurrentIndex(0);
    }
    removePaneIfEmpty(dragSrcPane);
    rubberBand->hide();
}


void MainWindowImpl::dragViewOutsidePane()
{
    QRect r;
    int w = topSplitter->width();
    int h = topSplitter->height();
    if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }
    r.translate(topSplitter->pos());
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void MainWindowImpl::dropViewOutsidePane()
{
    View* view = static_cast<View*>(dragSrcPane->currentWidget());

    QSize size = topSplitter->size();

    if(topSplitter->count() >= 2){
        QSplitter* newTopSplitter = new QSplitter(centralWidget);
        newTopSplitter->addWidget(topSplitter);
        topSplitter = newTopSplitter;
        centralVBox->addWidget(topSplitter, 1);
    }
    TabWidget* newTabWidget = new TabWidget(this, topSplitter);

    if(dropEdge == LEFT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->insertWidget(0, newTabWidget);
    } else if(dropEdge == RIGHT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->addWidget(newTabWidget);
    } else if(dropEdge == TOP){
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->insertWidget(0, newTabWidget);
    } else {
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->addWidget(newTabWidget);
    }
    newTabWidget->addView(view);

    int half;
    if(topSplitter->orientation() == Qt::Horizontal){
        half = size.height() / 2;
    } else {
        half = size.width() / 2;
    }
    QList<int> sizes;
    sizes << half << half;
    topSplitter->setSizes(sizes);
    
    removePaneIfEmpty(dragSrcPane);
    rubberBand->hide();
}


void MainWindowImpl::removePaneIfEmpty(TabWidget* pane)
{
    if(pane->count() == 0){
        QSplitter* parentSplitter = dynamic_cast<QSplitter*>(pane->parentWidget());
        pane->deleteLater();
        if(parentSplitter){
            if(parentSplitter->count() <= 1){
                QSplitter* grandParentSplitter = dynamic_cast<QSplitter*>(parentSplitter->parentWidget());
                if(grandParentSplitter){
                    if(parentSplitter->count() == 1){
                        int parentSplitterIndex = grandParentSplitter->indexOf(parentSplitter);
                        grandParentSplitter->insertWidget(parentSplitterIndex, parentSplitter->widget(0));
                    }
                    delete parentSplitter;
                }
            }
        }
    }
}
