/**
    @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneItem.h"
#include "SceneView.h"
#include "ItemTreeView.h"
#include "ItemManager.h"
#include "Archive.h"
#include "RootItem.h"
#include "OptionManager.h"
#include "PutPropertyFunction.h"
#include <cnoid/YamlReader>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    const bool TRACE_FUNCTIONS = false;

    bool loadSceneItem(SceneItem* item, const std::string& filename, std::ostream& os)
    {
        SceneObject::ReadResult result = item->loadScene(filename);
        if(result.success()){
            if(item->name().empty()){
                item->setName(filesystem::path(filename).leaf());
            }
        } else {
            os << result.message();
        }

        return result.success();
    }


    class SceneItemManager
    {
        SceneView* sceneView;
        std::set<SceneItemPtr> visibleSceneItems;

    public:

        SceneItemManager(ExtensionManager* ext) {

            ext->itemManager().registerClass<SceneItem>("SceneItem");

            ext->itemManager().addLoader<SceneItem>(
                _("Scene file"), "OSG-IMPORTABLE-SCENE-FILE", "", bind(loadSceneItem, _1, _2, _3), ItemManager::PRIORITY_CONVERSION);

            ext->optionManager().addOption("model", program_options::value< vector<string> >(), "load a 3d model file");
            ext->optionManager().sigOptionsParsed().connect(
                bind(&SceneItemManager::onSigOptionsParsed, this, _1));

            ItemTreeView::mainInstance()->sigCheckToggled().connect(
                bind(&SceneItemManager::onItemCheckToggled, this, _1, _2));

            sceneView = SceneView::mainInstance();
        }

        void onSigOptionsParsed(boost::program_options::variables_map& variables) {

            if(variables.count("model")){
                vector<string> files = variables["model"].as< vector<string> >();
                for(size_t i=0; i < files.size(); ++i){
                    SceneItemPtr item(new SceneItem());
                    if(item->load(files[i], "OSG-IMPORTABLE-SCENE-FILE")){
                        RootItem::mainInstance()->addChildItem(item);
                        ItemTreeView::mainInstance()->checkItem(item, true);
                    }
                }
            }
        }

        void onItemCheckToggled(Item* item, bool isChecked) {
            SceneItem* sceneItem = dynamic_cast<SceneItem*>(item);
            if(sceneItem){
                std::set<SceneItemPtr>::iterator p = visibleSceneItems.find(sceneItem);
                if(isChecked){
                    if(p == visibleSceneItems.end()){
                        sceneView->addSceneObject(sceneItem->sceneObject());
                        sceneView->requestRedraw();
                        visibleSceneItems.insert(sceneItem);
                    }
                } else {
                    if(p != visibleSceneItems.end()){
                        sceneView->removeSceneObject(sceneItem->sceneObject());
                        sceneView->requestRedraw();
                        visibleSceneItems.erase(p);
                    }
                }
            }
        }

        ~SceneItemManager() {
            std::set<SceneItemPtr>::iterator p;
            for(p = visibleSceneItems.begin(); p != visibleSceneItems.end(); ++p){
                SceneItemPtr sceneItem = *p;
                sceneView->removeSceneObject(sceneItem->sceneObject());
            }
        }
    };
}
    

void SceneItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->manage(new SceneItemManager(ext));
        initialized = true;
    }
}


SceneItem::SceneItem()
    : sceneObject_(new SceneObject)
{

}


SceneItem::SceneItem(const SceneItem& org)
    : Item(org),
      sceneObject_(new SceneObject(*org.sceneObject_))
{

}


SceneItem::~SceneItem()
{

}


SceneObject::ReadResult SceneItem::loadScene(const std::string& filename)
{
    return sceneObject_->load(filename);
}


ItemPtr SceneItem::doDuplicate() const
{
    return new SceneItem(*this);
}


void SceneItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Scene file"), filesystem::path(lastAccessedFileName()).leaf());
}


bool SceneItem::store(Archive& archive)
{
    if(!lastAccessedFileName().empty()){
        archive.writeRelocatablePath("file", lastAccessedFileName());
        archive.write("format", lastAccessedFileFormatId());
    }
    return true;
}


bool SceneItem::restore(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.readRelocatablePath("file", filename) && archive.read("format", formatId)){
        if(load(filename, archive.currentParentItem(), formatId)){
            return true;
        }
    }
    return false;
}
