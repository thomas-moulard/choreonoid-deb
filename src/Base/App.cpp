/**
   @author Shin'ichiro Nakaoka
*/

#include "App.h"
#include "AppConfig.h"
#include "PluginManager.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "ProjectManager.h"
#include "MenuManager.h"
#include "OptionManager.h"
#include "TimeSyncItemEngineManager.h"
#include "MainWindow.h"
#include "RootItem.h"
#include "FolderItem.h"
#include "View.h"
#include "MessageView.h"
#include "ItemTreeView.h"
#include "ItemPropertyView.h"
#include "SceneView.h"
#include "TimeBar.h"
#include "FileBar.h"
#include "GraphBar.h"
#include "SceneItem.h"
#include "MultiValueSeqItem.h"
#include "MultiAffine3SeqItem.h"
#include "Vector3SeqItem.h"
#include "ProjectPathSetEditor.h"
#include "DescriptionDialog.h"
#include "Licenses.h"
//#include "MovieGenerator.h"

#include <cnoid/Config>
#include <cnoid/YamlNodes>
#include <QTextCodec>
#include <QIcon>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <exception>
#include <clocale>

#ifdef Q_OS_WIN32
#include <windows.h>
#endif

#ifdef Q_OS_UNIX
#include <sys/utsname.h>
#endif

#ifdef Q_OS_DARWIN
#include <mach-o/dyld.h>
#endif

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

    class AppImpl : public QObject
    {
    public:
        AppImpl(App* self, int& argc, char**& argv);
        ~AppImpl();

        App* self;
        int& argc;
        char**& argv;
        PluginManager* pluginManager;
        ExtensionManager* ext;
        string appName;
        string vendorName;
        DescriptionDialog* descriptionDialog;

        void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);
        void findProgramTopDirectory();
        int exec();
        bool processCommandLineOptions();
        virtual bool eventFilter(QObject* watched, QEvent* event);

        void showInformationDialog();
    };
}

namespace {
    string programTopDirectory_;
    string shareDirectory_;
    string programDirectory_;
}


App::App(int& argc, char**& argv)
    : QApplication(argc, argv)
{
    impl = new AppImpl(this, argc, argv);
}


AppImpl::AppImpl(App* self, int& argc, char**& argv)
    : self(self),
      argc(argc),
      argv(argv)
{
    descriptionDialog = 0;
}

void App::initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList)
{
    impl->initialize(appName, vendorName, icon, pluginPathList);
}


void AppImpl::initialize( const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList)
{
    this->appName = appName;
    this->vendorName = vendorName;

    findProgramTopDirectory();

    setlocale(LC_ALL, ""); // for gettext
    
    YamlNode::initialize();

    QTextCodec::setCodecForCStrings(QTextCodec::codecForLocale());

    self->setApplicationName(appName);
    self->setOrganizationName(vendorName);
    self->setWindowIcon(icon);

    ext = new ExtensionManager("Base", false);

    AppConfig::initialize(appName, vendorName);
    MainWindow::initialize(appName, ext);
    MessageView::initialize(ext);
    RootItem::initialize(ext);
    ProjectManager::initialize(ext);

    FileBar::initialize(ext);
    TimeBar::initialize(ext);
    ItemTreeView::initialize(ext);
    ItemPropertyView::initialize(ext);
    SceneView::initialize(ext);
    GraphBar::initialize(ext);
    TimeSyncItemEngineManager::initialize();
    FolderItem::initialize(ext);
    SceneItem::initialize(ext);
    MultiValueSeqItem::initialize(ext);
    MultiAffine3SeqItem::initialize(ext);
    Vector3SeqItem::initialize(ext);

    ProjectPathSetEditor::initialize(ext);
    
    //initializeMovieGenerator(*ext);

    ext->menuManager().setPath("/Help").addItem(_("About Choreonoid Framework"))
        ->sigTriggered().connect(bind(&AppImpl::showInformationDialog, this));

    pluginManager = PluginManager::instance();

    if(pluginPathList){
        pluginManager->scanPluginFilesInPathList(pluginPathList);
    }
    pluginManager->scanPluginFilesInDirectoyOfExecFile();
    pluginManager->loadAllPlugins();

    MainWindow::instance()->installEventFilter(this);
}


App::~App()
{
    if(impl){
        delete impl;
    }
}


AppImpl::~AppImpl()
{
    AppConfig::flush();
    delete pluginManager;
}


void AppImpl::findProgramTopDirectory()
{
#ifdef Q_OS_WIN32
    static const int BUFSIZE = 1024;
    TCHAR execFilePath[BUFSIZE];
    if(GetModuleFileName(NULL, execFilePath, BUFSIZE)){
#ifndef UNICODE
        programDirectory_ = filesystem::path(execFilePath).branch_path().file_string();
        programTopDirectory_ = filesystem::path(execFilePath).branch_path().branch_path().file_string();
#else
        int codepage = _getmbcp();
        const int newSize = WideCharToMultiByte(codepage, 0, execFilePath, -1, NULL, 0, NULL, NULL);
        if(newSize > 0){
            vector<filesystem::path::String> execFilePathMB(newSize + 1);
            newSize = WideCharToMultiByte(codepage, 0, execFilePath, -1, &execFilePathMB[0], newSize + 1, NULL, NULL);
            programDirectory_ = filesystem::path(execFilePathUtf8).branch_path().file_string();
            programTopDirectory_ = filesystem::path(execFilePathUtf8).branch_path().branch_path().file_string();
        }
#endif // UNICODE
    }
#endif // Q_OS_WIN32

#ifdef Q_OS_LINUX
    utsname info;
    if(uname(&info) == 0){
        if(strncmp(info.sysname, "Linux", 6) == 0){
            static const int BUFSIZE = 1024;
            char buf[BUFSIZE];
            int n = readlink("/proc/self/exe", buf, BUFSIZE - 1);
            buf[n] = 0;
	    programDirectory_ = filesystem::path(buf).branch_path().file_string();
            programTopDirectory_ = filesystem::path(buf).branch_path().branch_path().file_string();
        }
    }
#endif // Q_OS_LINUX

#ifdef Q_OS_DARWIN
    char buf[1024];
    uint32_t n = sizeof(buf);
    if(_NSGetExecutablePath(buf, &n) == 0){
        programDirectory_ = filesystem::path(buf).branch_path().file_string();
        programTopDirectory_ = filesystem::path(buf).branch_path().branch_path().file_string();
    }
#endif // Q_OS_DARWIN

    filesystem::path sharePath = filesystem::path(programTopDirectory_) / CNOID_SHARE_SUBDIR;
    if(filesystem::is_directory(sharePath)){
        shareDirectory_ = sharePath.file_string();
    } else {
        shareDirectory_ = sharePath.parent_path().file_string();
    }
}


const string& App::topDirectory()
{
    return programTopDirectory_;
}


const std::string& App::shareDirectory()
{
    return shareDirectory_;
}


const string& App::programDirectory()
{
    return programDirectory_;
}


int App::exec()
{
    return impl->exec();
}


int AppImpl::exec()
{
    MainWindow* mainWindow = MainWindow::instance();

    processCommandLineOptions();

    if(!mainWindow->isVisible()){
        mainWindow->show();
    }

    int result = self->QApplication::exec();

    if(pluginManager->unloadAllPlugins()){
        delete ext;
        delete mainWindow;
    }
    
    return result;
}


bool AppImpl::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == MainWindow::instance()){
        if(event->type() == QEvent::Close){
            event->accept();
            return true;
        } 
    }
                
    return false;
}



bool AppImpl::processCommandLineOptions()
{
    if(!ext->optionManager().parseCommandLine(argc, argv)){
        //put error messages
    }

    return false;
}


void AppImpl::showInformationDialog()
{
    if(!descriptionDialog){

        descriptionDialog = new DescriptionDialog();

        descriptionDialog->setWindowTitle(_("About Choreonoid Framework"));

        descriptionDialog->setDescription(
            QString(_("Choreonoid Framework Version %1\n")).arg(CNOID_FULL_VERSION_STRING) +
            _("\n"
              "This program has been developed by Shin'ichiro Nakaoka and Choreonoid Development Team, AIST, "
              "and is distributed as a part of the Choreonoid package.\n"
              "\n") +
            LGPLtext() +
            _("The source and some binary packages of this program also include the following thirdparty "
              "libraries:\n"
              "\n"
              "* Eigen (http://eigen.tuxfamily.org/)\n"
              "* IJG JPEG Library (http://www.ijg.org/)\n"
              "* libpng (http://www.libpng.org/pub/png/libpng.html)\n"
              "* LibYAML (http://pyyaml.org/wiki/LibYAML)\n"
              "* zlib (http://zlib.net/)\n"
              "\n"
              "These libraries are used and redistributed under the terms of their licenses. Please see "
              "the corresponding directories in the source package or their official web sites to see the "
              "details of their licenses.\n"
              "\n"
              "This program also depends on a lot of other external libraries, which are linked into this "
              "program during the execution. Some binary packages of this program may include the binaries "
              "of the following libraries:\n"
              "\n"
              "* Qt (http://qt.nokia.com/)\n"
              "* OpenSceneGraph (http://www.openscenegraph.org)\n"
              "* Boost C++ Libraries (http://www.boost.org/) \n"
              "\n"
              "Please see their official web sites to see the details of their licenses.\n"
                ));
    }

    descriptionDialog->show();
}
