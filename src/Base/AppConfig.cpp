/**
   @author Shin'ichiro Nakaoka
*/

#include "AppConfig.h"
#include "MessageView.h"
#include <cnoid/YamlReader>
#include <cnoid/YamlWriter>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

using boost::filesystem::path;

namespace {
    string application;
    string organization;

    path configDirPath;
    path filePath;
    path fullPath;

    shared_ptr<YamlReader> pYamlReader;
};

bool AppConfig::initialize(const std::string& application_, const std::string& organization_)
{
    application = application_;
    organization = organization_;

#ifdef WIN32
    const char* appdata = getenv("APPDATA");
    if(appdata){
        configDirPath = path(appdata) / path(organization_);
    }
#else
    const char* home = getenv("HOME");
    if(home){
        configDirPath = path(home) / path(".config") / path(organization_);
    }
#endif
    
    filePath = application + ".conf";

    if(!configDirPath.empty()){
        fullPath = configDirPath / filePath;
        std::string fullPathString = fullPath.file_string(); 
        load(fullPathString);
    }

    return !fullPath.empty();
}

// ！注意 archive()で取得したポインタを保持する場合は、load時に入れ替えること！
YamlMapping* AppConfig::archive()
{
    if(pYamlReader && pYamlReader->numDocuments()){
        return pYamlReader->document()->toMapping();
    }
    static YamlMappingPtr appArchive(new YamlMapping);
    return appArchive.get();
}
  

bool AppConfig::flush()
{
    if(configDirPath.empty()){
        return false;
    }
    
    if(!filesystem::exists(fullPath)){
        if(filesystem::exists(configDirPath)){
            if(!filesystem::is_directory(configDirPath)){

                const char* m =
                    "\"%1%\" is not a directory.\n"
                    "It should be directory to contain the config file.\n"
                    "The configuration cannot be stored into the file system";
                showWarningDialog(format(_(m)) % configDirPath.file_string());
                return false;
            }
        } else {
            filesystem::create_directories(configDirPath);
        }
    }

    return save(fullPath.file_string());
}


bool AppConfig::save(const std::string& filename)
{
    try {
        YamlWriter writer(filename);
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive());
    }
    catch(const YamlNode::Exception& ex){
        showWarningDialog(ex.message());
        return false;
    }
    return true;
}


/// @note 注意! YamlReaderが入れ替わるのでload後はarchive()で取得したポインタを入れ替えること！
bool AppConfig::load(const std::string& filename)
{
    YamlReader* pyaml = new YamlReader();

    boost::filesystem::path filePath(filename);
    if(filesystem::exists(filePath)){

        try {
            if(pyaml->load(filename)){
                if(pyaml->numDocuments() != 1 || pyaml->document()->type() != YAML_MAPPING){
                    pyaml->clearDocuments();
                }
            }
        } catch (const YamlNode::Exception& ex){
            ostream& os = MessageView::mainInstance()->cout();
            os << format("Application config file \"%1%\" cannot be loaded (%2%).")
                % filename % ex.message() << endl;
            pyaml->clearDocuments();
            delete pyaml;
            return false;
        }
    }
    
    pYamlReader = shared_ptr<YamlReader>(pyaml);
    return true;
}
