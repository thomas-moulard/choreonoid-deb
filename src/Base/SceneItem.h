/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_SCENE_ITEM_H_INCLUDED
#define CNOID_GUIBASE_SCENE_ITEM_H_INCLUDED

#include "Item.h"
#include "SceneObject.h"
#include "exportdecl.h"

namespace cnoid {
        
    class CNOID_EXPORT SceneItem : public Item
    {
      public:
        static void initialize(ExtensionManager* ext);

        SceneItem();
        SceneItem(const SceneItem& org);
        virtual ~SceneItem();

        inline SceneObject* sceneObject() { return sceneObject_.get(); }

        SceneObject::ReadResult loadScene(const std::string& filename);

      protected:
        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
        virtual void doPutProperties(PutPropertyFunction& putProperty);
            
      private:
        SceneObjectPtr sceneObject_;
    };

    typedef boost::intrusive_ptr<SceneItem> SceneItemPtr;


}

#endif
