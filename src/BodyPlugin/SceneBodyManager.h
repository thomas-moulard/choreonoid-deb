/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SCENE_BODY_MANAGER_H_INCLUDED
#define CNOID_BODYPLUGIN_SCENE_BODY_MANAGER_H_INCLUDED

#include <cnoid/ExtensionManager>
#include "exportdecl.h"

namespace cnoid {

    class SceneBody;
    class BodyItem;
    class SBMImpl;
    
    class CNOID_EXPORT SceneBodyManager
    {
      public:
        
        static SceneBodyManager* instance();
        
        SceneBodyManager(ExtensionManager& ext);
        ~SceneBodyManager();
        
        class FactoryHolder {
        public:
            virtual ~FactoryHolder() { }
        };
        
        FactoryHolder* addSceneBodyFactory(boost::function<SceneBody*(BodyItem*)> factory);
        
      private:
        SBMImpl* impl;
    };
}

#endif
