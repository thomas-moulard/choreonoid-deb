/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_APP_H_INCLUDED
#define CNOID_GUIBASE_APP_H_INCLUDED

#include <string>
#include <QApplication>
#include "exportdecl.h"

namespace cnoid {

    class ExtensionManager;
    class AppImpl;

    class CNOID_EXPORT App : public QApplication
    {
      public:

	/**
           @if jp
           @param appName アプリケーション名
           @param vendorName ベンダ名
           @endif
        */
        App(int& argc, char**& argv);
        ~App();
        
        void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);

        int exec();

        static const std::string& topDirectory();
        static const std::string& shareDirectory();
        static const std::string& programDirectory();

    private:

	AppImpl* impl;
    };
}


#endif
