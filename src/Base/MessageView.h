/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_MESSAGE_VIEW_H_INCLUDED
#define CNOID_GUIBASE_MESSAGE_VIEW_H_INCLUDED

#include <cnoid/View>
#include <string>
#include <iosfwd>
#include <boost/format.hpp>
#include <QString>
#include "exportdecl.h"

namespace cnoid {

    class ExtensionManager;
    class MessageViewImpl;

    class CNOID_EXPORT MessageView : public View
    {
      public:
        static void initialize(ExtensionManager* ext);
        static MessageView* mainInstance();
      
        MessageView();
        ~MessageView();

        void put(const std::string& message);
        void put(const boost::format& message);
        void putln(const std::string& message);
        void putln(const boost::format& message);
        void notify(const std::string& message);
        void notify(const boost::format& message);

        void put(const char* message);
        void put(const QString& message);
        void putln();
        void putln(const char* message);
        void putln(const QString& message);
        void notify(const char* message);
        void notify(const QString& message);
        
        void flush();
        void clear();
      
        std::ostream& cout(bool doFlush = false);

        void beginStdioRedirect();
        void endStdioRedirect();

      protected:
        virtual bool event(QEvent* e);

      private:
        MessageViewImpl* impl;
    };

    CNOID_EXPORT void showWarningDialog(const std::string& message);
    CNOID_EXPORT void showWarningDialog(const boost::format& message);
    CNOID_EXPORT void showWarningDialog(const char* message);
    CNOID_EXPORT void showWarningDialog(const QString& message);
}

#endif
