/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_FOLDER_ITEM_H_INCLUDED
#define CNOID_GUIBASE_FOLDER_ITEM_H_INCLUDED

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT FolderItem : public Item
    {
      public:
        static void initialize(ExtensionManager* ext);
        
        FolderItem();
        FolderItem(const std::string& name);
        FolderItem(const FolderItem& org);

      protected:

        virtual ~FolderItem();

        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
    };

    typedef boost::intrusive_ptr<FolderItem> FolderItemPtr;
}

#endif
