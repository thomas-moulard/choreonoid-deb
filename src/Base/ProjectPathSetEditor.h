/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_PROJECT_PATH_SET_EDITOR_H_INCLUDED
#define CNOID_GUIBASE_PROJECT_PATH_SET_EDITOR_H_INCLUDED

namespace cnoid {
    
    class ExtensionManager;

    class ProjectPathSetEditor
    {
    public:
        static void initialize(ExtensionManager* ext);
    };
}

#endif
