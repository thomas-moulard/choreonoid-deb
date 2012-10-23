/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_MAIN_WINDOW_H_INCLUDED
#define CNOID_GUIBASE_MAIN_WINDOW_H_INCLUDED

#include <cnoid/YamlNodes>
#include <QMainWindow>
#include "exportdecl.h"

namespace cnoid {

    class View;
    class ToolBar;
    class MainWindowImpl;
    class ExtensionManager;

    /**
       @if jp
       メインウィンドウ。
       @endif
    */
    class CNOID_EXPORT MainWindow : public QMainWindow
    {
     public:
        static void initialize(const char* appName, ExtensionManager* ext);
        static MainWindow* instance();

        void show();
        
        bool addView(const std::string& pluginName, View* view);
        bool removeView(View* view);

        void addToolBar(ToolBar* toolbar);
        
        std::vector<View*> allViews();
        std::vector<ToolBar*> allToolBars();

        void storeLayout(YamlMappingPtr layout);
        void restoreLayout(const YamlMappingPtr layout);
        void setInitialLayout(const YamlMappingPtr layout);

      protected:
        virtual void changeEvent(QEvent* event);
        virtual void resizeEvent(QResizeEvent* event);
        virtual void keyPressEvent(QKeyEvent* event);

      private:
        MainWindowImpl* impl;

        MainWindow(const char* appName, ExtensionManager* ext);
        virtual ~MainWindow();

        friend class AppImpl;
        friend class ExtensionManager;
        friend class View;
    };
}

#endif
