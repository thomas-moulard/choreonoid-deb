/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TOOL_BAR_H_INCLUDED
#define CNOID_GUIBASE_TOOL_BAR_H_INCLUDED

#include <cnoid/Action>
#include <QLabel>
#include <QToolBar>
#include "exportdecl.h"

namespace cnoid {

    class Archive;
    class ExtensionManager;
    class ToolBarArea;

    class CNOID_EXPORT ToolBar : public QToolBar
    {
      public:

        ToolBar(const QString& title);
        virtual ~ToolBar();

        Action* addButton(const QString& text, const QString& tooltip = QString());
        Action* addButton(const QIcon& icon, const QString& tooltip = QString());
        Action* addButton(const char* const* xpm, const QString& tooltip = QString());

        Action* addToggleButton(const QString& text, const QString& tooltip = QString());
        Action* addToggleButton(const QIcon& icon, const QString& tooltip = QString());
        Action* addToggleButton(const char* const* xpm, const QString& tooltip = QString());

        void requestNewRadioGroup();
        QActionGroup* currentRadioGroup();
        
        Action* addRadioButton(const QString& text, const QString& tooltip = QString());
        Action* addRadioButton(const QIcon& icon, const QString& tooltip = QString());
        Action* addRadioButton(const char* const* xpm, const QString& tooltip = QString());
        
        QLabel* addLabel(const QString& text);
        void addSpace(int size);

        void setStretchable(bool on) { isStretchable_ = on; }
        bool isStretchable() { return isStretchable_; }

        virtual int bestStretchWidth();

        class LayoutPriorityCmp {
        public:
            bool operator() (ToolBar* bar1, ToolBar* bar2) {
                return (bar1->layoutPriority < bar2->layoutPriority);
            }
        };
        
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);

      private:
        QActionGroup* radioGroup;
        bool isNewRadioGroupRequested;

        ToolBarArea* toolBarArea;

        // used for layouting tool bars on a ToolBarArea
        bool isStretchable_;
        int desiredX;
        int layoutPriority;

        void setRadioButton(Action* action);

        friend class ToolBarAreaImpl;
    };
}

#endif
