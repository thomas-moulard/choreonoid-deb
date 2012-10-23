/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TOOL_BAR_AREA_H_INCLUDED
#define CNOID_GUIBASE_TOOL_BAR_AREA_H_INCLUDED

#include <cnoid/YamlNodes>
#include <QWidget>
#include <vector>

namespace cnoid {

    class ToolBar;
    class ToolBarAreaImpl;
    class YamlMapping;

    class ToolBarArea : public QWidget
    {
    public:
        ToolBarArea(QWidget* parent);
        ~ToolBarArea();

        std::vector<ToolBar*> getAllToolBars();

        void setInitialLayout(YamlMappingPtr layout);

        void storeLayout(YamlMappingPtr layout);
        void restoreLayout(const YamlMappingPtr layout);

        bool addToolBar(ToolBar* toolBar);
        void removeToolBar(ToolBar* toolBar);

        // called from ToolBar
        void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);

    protected:
        virtual void resizeEvent(QResizeEvent* event);
        virtual bool event(QEvent* event);
        
        //virtual QSize sizeHint() const;
        //virtual QSize minimumSizeHint () const;

    private:
        ToolBarAreaImpl* impl;

        void doInitialLayout(); // called from MainWindowImpl;

        friend class MainWindowImpl;
    };
}

#endif
