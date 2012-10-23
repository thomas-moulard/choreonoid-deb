/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_FILE_BAR_H_INCLUDED
#define CNOID_GUIBASE_FILE_BAR_H_INCLUDED

#include <cnoid/ToolBar>

namespace cnoid {

    class FileBar : public ToolBar
    {
      public:
        static void initialize(ExtensionManager* ext);
        static FileBar* instance();
        
        virtual ~FileBar();
        
      private:
        FileBar();
    };
}

#endif
