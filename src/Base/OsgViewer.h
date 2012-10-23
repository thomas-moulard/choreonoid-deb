
#ifndef CNOID_GUIBASE_OSG_VIEWER_H_INCLUDED
#define CNOID_GUIBASE_OSG_VIEWER_H_INCLUDED

#include <osgViewer/Viewer>
#include <QGLWidget>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT OsgWidget : public QGLWidget
    {
        osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphicsWindow_;
        
    public:
        OsgWidget(QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
        virtual ~OsgWidget();

        inline osgViewer::GraphicsWindow* graphicsWindow() {
            return graphicsWindow_.get();
        }
        inline const osgViewer::GraphicsWindow* graphicsWindow() const {
            return graphicsWindow_.get();
        }

        static int convertToOsgMouseButtonMask(int qtButton);
        static int convertToOsgKeySymbol(int qtKeySymbol);
        static int convertToOsgModKeyMask(int qtModifiers);
        static int convertToOsgScrollingMotion(QWheelEvent* event);

    protected:
        virtual void resizeGL(int width, int height);
    };


    class CNOID_EXPORT OsgViewer : public osgViewer::Viewer, public OsgWidget
    {
      public:
        OsgViewer(QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
        virtual ~OsgViewer();

        virtual void requestRedraw();
        virtual void paintGL();

        /// temorary hack
        inline bool isCollisionVisibleMode() { return isCollisionVisibleMode_; }

      private:
        bool isCollisionVisibleMode_;
        friend class SceneViewImpl;
    };
}

#endif
