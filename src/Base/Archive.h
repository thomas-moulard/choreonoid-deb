/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_GUIBASE_ARCHIVE_H_INCLUDED
#define CNOID_GUIBASE_GUIBASE_ARCHIVE_H_INCLUDED

#include <cnoid/YamlNodes>
#include <string>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

    class Item;
    typedef boost::intrusive_ptr<Item> ItemPtr;

    class Archive;
    typedef boost::intrusive_ptr<Archive> ArchivePtr;

    class ArchiveSharedData;

    class CNOID_EXPORT Archive : public YamlMapping
    {
      public:
        Archive();
        Archive(int line, int column);
        virtual ~Archive();

        void initSharedInfo(const std::string& projectFile);
        void inheritSharedInfoFrom(ArchivePtr archive);

        void addPostProcess(const boost::function<void()>& func) const;

        Archive* findSubArchive(const std::string& name);

        //int getItemId(Item* item) const;
        int getItemId(ItemPtr item) const;
        Item* findItem(int id) const;
        
        template<class ItemType> inline ItemType* findItem(int id) const {
            return dynamic_cast<ItemType*>(findItem(id));
        }

        void writeItemId(const std::string& key, ItemPtr item);

        template<class ItemType> inline ItemType* findItem(const std::string& key) const {
            int id;
            return read(key, id) ? findItem<ItemType>(id) : 0;
        }
        
        bool readRelocatablePath(const std::string& key, std::string& out_value) const;
        void writeRelocatablePath(const std::string& key, const std::string& path);

        Item* currentParentItem() const;

      private:

        boost::intrusive_ptr<ArchiveSharedData> shared;

        void setCurrentParentItem(Item* parentItem);

        static ArchivePtr invalidArchive();
        void registerItemId(Item* item, int id);

        // called from ItemTreeArchiver
        void callPostProcesses();

        friend class ItemTreeArchiver;
        friend class ProjectManagerImpl;
    };
}

#endif
