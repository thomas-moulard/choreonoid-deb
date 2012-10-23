/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_MENU_MANAGER_H_INCLUDED
#define CNOID_GUIBASE_MENU_MANAGER_H_INCLUDED

#include "Action.h"
#include <cnoid/ExtensionManager>
#include <QMenu>
#include <QMenuBar>
#include "exportdecl.h"

namespace cnoid {

    class MenuManagerImpl;

    class CNOID_EXPORT Menu : public QMenu
    {
        Q_OBJECT

      public:
        Menu(QWidget* parent = 0);
        Menu(const QString& title, QWidget* parent = 0);

        inline SignalProxy< boost::signal<void(QAction*)> > sigTriggered() {
            return sigTriggered_;
        }
        inline SignalProxy< boost::signal<void()> > sigAboutToHide() {
            return sigAboutToHide_;
        }

      private Q_SLOTS:
        void onTriggered(QAction* action);
        void onAboutToHide();

      private:
        boost::signal<void(QAction*)> sigTriggered_;
        boost::signal<void()> sigAboutToHide_;

        void initialize();
    };

    
    /**
       @if jp
       メニューを簡単に管理するためのクラス。   
       @endif
    */
    class CNOID_EXPORT MenuManager : public QObject
    {
        Q_OBJECT

      public:

        MenuManager();
        MenuManager(QWidget* topMenu);
        virtual ~MenuManager();

        void bindTextDomain(const std::string& domain);

        void setTopMenu(QWidget* topMenu);

        QAction* findItem(const QString& path);
        MenuManager& setPath(const QString& path);
        
        MenuManager& setBackwardMode();

        void addAction(QAction* action);

        Action* addItem(const QString& text);
        Action* addCheckItem(const QString& text);
        Action* addRadioItem(QActionGroup* group, const QString& text);

        MenuManager& addSeparator();

        void removeItem(QAction* item);

        void releaseAllItems();

      private:

        MenuManager(const MenuManager* org);

        MenuManagerImpl* impl;

      public Q_SLOTS:
        void onItemDestroyed();
    };
}

#endif
