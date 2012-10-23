/**
   @author Shin'ichiro Nakaoka
*/

#include "Archive.h"
#include "Item.h"
#include "App.h"
#include "AppConfig.h"
#include "MessageView.h"
#include <cnoid/Referenced>
#include <cnoid/FileUtil>
#include <map>
#include <vector>
#include <boost/signals.hpp>
#include <boost/algorithm/string.hpp>
#include <QRegExp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {
    QRegExp regexVar("^\\$\\{(\\w+)\\}");

    typedef map<ItemPtr, int> ItemToIdMap;
}


namespace cnoid {
    
    class ArchiveSharedData : public Referenced
    {
    public:
        YamlMapping* directoryVariableMap;
        filesystem::path projectDirPath;
        filesystem::path topDirPath;
        filesystem::path shareDirPath;
        QString topDirString;
        QString shareDirString;

        vector<Item*> idToItems;
        ItemToIdMap itemToIds;

        Item* currentParentItem;

        boost::signal<void()> postProcesses;
    };


    bool findSubDirectoryOfDirectoryVariable
    (ArchiveSharedData* shared, const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath)
    {
        YamlMapping::const_iterator p;
        for(p = shared->directoryVariableMap->begin(); p != shared->directoryVariableMap->end(); ++p){
            YamlSequence* paths = p->second->toSequence();
            if(paths){
                for(int i=0; i < paths->size(); ++i){
                    filesystem::path dirPath(paths[i].toString());
                    if(findSubDirectory(dirPath, path, out_relativePath)){
                        out_varName = fromUtf8(p->first);
                        return true;
                    }
                }
            }
        }
        return false;
    }


    void replaceDirectoryVariable(ArchiveSharedData* shared, QString& io_pathString, const QString& varname, int pos, int len)
    {
        YamlSequence* paths = shared->directoryVariableMap->findSequence(varname.toStdString());
        if(paths){
            for(int i=0; i < paths->size(); ++i){
                string vpath;
                QString replaced(io_pathString);
                replaced.replace(pos, len, paths[i].toString().c_str());
                filesystem::file_status fstatus = filesystem::status(filesystem::path(replaced.toStdString()));
                if(filesystem::is_directory(fstatus) || filesystem::exists(fstatus)) {
                    io_pathString = replaced;
                    return;
                }
            }
        }
        MessageView::mainInstance()->putln(QString(_("Warning: ${%1} of \"%2\" cannot be expanded !")).arg(varname).arg(io_pathString));
    }
}


ArchivePtr Archive::invalidArchive()
{
    static ArchivePtr invalidArchive_ = new Archive();
    invalidArchive_->type_ = YAML_NONE;
    return invalidArchive_;
}


Archive::Archive()
{

}


Archive::Archive(int line, int column)
    : YamlMapping(line, column)
{

}


Archive::~Archive()
{

}


void Archive::initSharedInfo(const std::string& projectFile)
{
    shared = new ArchiveSharedData;

    shared->directoryVariableMap = AppConfig::archive()->openMapping("PathVariables");

    shared->projectDirPath = filesystem::complete(filesystem::path(projectFile)).branch_path();
    shared->topDirPath = App::topDirectory();
    shared->shareDirPath = App::shareDirectory();

    shared->topDirString = shared->topDirPath.string().c_str();
    shared->shareDirString = shared->shareDirPath.string().c_str();
    
    shared->currentParentItem = 0;
}


void Archive::inheritSharedInfoFrom(ArchivePtr archive)
{
    shared = archive->shared;
}


void Archive::addPostProcess(const boost::function<void()>& func) const
{
    if(shared){
        shared->postProcesses.connect(func);
    }
}


void Archive::callPostProcesses()
{
    if(shared){
        shared->postProcesses();
    }
}


Archive* Archive::findSubArchive(const std::string& name)
{
    YamlMapping* mapping = findMapping(name);
    if(mapping->isValid()){
        Archive* archive = dynamic_cast<Archive*>(mapping);
        if(archive){
            return archive;
        }
    }

    return invalidArchive().get();
}


bool Archive::readRelocatablePath(const std::string& key, std::string& out_value) const
{
    string orgPathString;

    if(read(key, orgPathString)){

        QString pathString(orgPathString.c_str());

        // expand variables in the path
        int pos = regexVar.indexIn(pathString);
        if(pos != -1){
            int len = regexVar.matchedLength();
            if(regexVar.captureCount() > 0){
                QString varname = regexVar.cap(1);
                if(varname == "SHARE"){
                    pathString.replace(pos, len, shared->shareDirString);
                } else if(varname == "PROGRAM_TOP"){
                    pathString.replace(pos, len, shared->topDirString);
                } else {
                    replaceDirectoryVariable(shared.get(), pathString, varname, pos, len);
                }
            }
        }
            
        filesystem::path path(pathString.toStdString());
        if(path.is_complete()){
            out_value = path.file_string();
        } else {
            filesystem::path fullPath = shared->projectDirPath / path;
            if(!path.empty() && (*path.begin() == "..")){
                filesystem::path compact;
                makePathCompact(fullPath, compact);
                out_value = compact.file_string();
            } else {
                out_value = fullPath.file_string();
            }
        } 
        return true;
    }

    return false;
}


/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
void Archive::writeRelocatablePath(const std::string& key, const std::string& orgPathString)
{
    filesystem::path orgPath(orgPathString);
    filesystem::path relativePath;
    string varName;

    if(findSubDirectory(shared->projectDirPath, orgPath, relativePath)){
        write(key, relativePath.string(), YAML_DOUBLE_QUOTED);
    
    } else if(findSubDirectoryOfDirectoryVariable(shared.get(), orgPath, varName, relativePath)){
        write(key, string("${") + varName + ("}/") + relativePath.string(), YAML_DOUBLE_QUOTED);

    } else if(findSubDirectory(shared->shareDirPath, orgPath, relativePath)){
        write(key, string("${SHARE}/") + relativePath.string(), YAML_DOUBLE_QUOTED);

    } else if(findSubDirectory(shared->topDirPath, orgPath, relativePath)){
        write(key, string("${PROGRAM_TOP}/") + relativePath.string(), YAML_DOUBLE_QUOTED);

    } else if(findRelativePath(shared->projectDirPath, orgPath, relativePath)){
        write(key, relativePath.string(), YAML_DOUBLE_QUOTED);
    }
}


void Archive::registerItemId(Item* item, int id)
{
    if(shared){
        if(id >= (signed)shared->idToItems.size()){
            shared->idToItems.resize(id + 1);
        }
        shared->idToItems[id] = item;
        shared->itemToIds[item] = id;
    }
}


/**
   @return -1 if item does not belong to the archive
*/
//int Archive::getItemId(Item* item) const
int Archive::getItemId(ItemPtr item) const
{
    if(shared && item){
        ItemToIdMap::const_iterator p = shared->itemToIds.find(item);
        if(p != shared->itemToIds.end()){
            return p->second;
        }
    }
    return -1;
}


void Archive::writeItemId(const std::string& key, ItemPtr item)
{
    if(item){
        int id = getItemId(item);
        if(id >= 0){
            write(key, id);
        }
    }
}


Item* Archive::findItem(int id) const
{
    if(shared){
        if(id >= 0 && id < (signed)shared->idToItems.size()){
            return shared->idToItems[id];
        }
    }
    return 0;
}


Item* Archive::currentParentItem() const
{
    if(shared){
        return shared->currentParentItem;
    }
    return 0;
}


void Archive::setCurrentParentItem(Item* parentItem)
{
    if(shared){
        shared->currentParentItem = parentItem;
    }
}
