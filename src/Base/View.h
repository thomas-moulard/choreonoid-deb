/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_VIEW_H_INCLUDED
#define CNOID_GUIBASE_VIEW_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

    class ToolBar;
    class Archive;
    class ExtensionManager;

    class CNOID_EXPORT View : public QWidget
    {
      public:
        
        View();
        virtual ~View();

        void setName(const QString& name);
        inline QString name() const { return objectName(); }

        enum LayoutArea { LEFT = 0,
                          LEFT_TOP = 0,
                          LEFT_BOTTOM = 1,
                          CENTER = 2,
                          RIGHT = 3,
                          BOTTOM = 4,
                          NUM_AREAS };

        void setDefaultLayoutArea(LayoutArea area);
        LayoutArea defaultLayoutArea() const;

        bool isActive() const;

        inline SignalProxy< boost::signal<void()> > sigActivated() {
            return sigActivated_;
        }

        inline SignalProxy< boost::signal<void()> > sigDeactivated() {
            return sigDeactivated_;
        }

        void setLayout(QLayout* layout);

        virtual QWidget* indicatorOnInfoBar();

      protected:

        virtual void onActivated();
        virtual void onDeactivated();

        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);


      private:

        // Qt events (make hidden)
        virtual void showEvent(QShowEvent* event);
        virtual void hideEvent(QHideEvent* event);
        
        LayoutArea defaultLayoutArea_;
        bool isManagedByMainWindow;

        bool isActive_;
        boost::signal<void()> sigActivated_;
        boost::signal<void()> sigDeactivated_;

        friend class MainWindow;
        friend class MainWindowImpl;
        friend class ProjectManagerImpl;
    };
}

#endif
