/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_DIALOG_H_INCLUDED
#define CNOID_GUIBASE_DIALOG_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QDialog>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT Dialog : public QDialog
    {
        Q_OBJECT

      public:
        Dialog(QWidget* parent = 0, Qt::WindowFlags f = 0);
        
        inline SignalProxy< boost::signal<void()> > sigAccepted() {
            return sigAccepted_;
        }
        inline SignalProxy< boost::signal<void(int)> > sigFinished() {
            return sigFinished_;
        }
        inline SignalProxy< boost::signal<void()> > sigRejected() {
            return sigRejected_;
        }

      private Q_SLOTS:
        void onAccepted();
        void onFinished(int result);
        void onRejected();

      private:
        boost::signal<void()> sigAccepted_;
        boost::signal<void(int)> sigFinished_;
        boost::signal<void()> sigRejected_;

        void initialize();
    };
}

#endif
