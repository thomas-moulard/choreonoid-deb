/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionItem.h"
#include "BodyItem.h"
#include "KinematicFaultChecker.h"
#include <cnoid/MultiSeqItemCreationPanel>
#include <cnoid/BodyMotionUtil>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <QMessageBox>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

    bool confirm(const std::string& message)
    {
        return (QMessageBox::warning(
                    0, _("Warning"), message.c_str(),
                    QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok) == QMessageBox::Ok);
    }

    bool fileIoSub(BodyMotionItem* item, std::ostream& os, bool loaded, bool isLoading)
    {
        if(!loaded){
            os << item->motion()->ioErrorMessage();
        }
        if(isLoading){
            item->updateChildItemLineup();
        }

        return loaded;
    }
                
    bool loadStandardYamlFormat(BodyMotionItem* item, const std::string& filename, std::ostream& os)
    {
        return fileIoSub(item, os, item->motion()->loadStandardYamlFormat(filename), true);
    }
    
    bool saveAsStandardYamlFormat(BodyMotionItem* item, const std::string& filename, std::ostream& os)
    {
        return fileIoSub(item, os, item->motion()->saveAsStandardYamlFormat(filename), false);
    }

    bool importHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
    {
        if(loadHrpsysSeqFileSet(*item->motion(), filename, os)){
            item->updateChildItemLineup();
            return true;
        }
        return false;
    }
    
    
    bool exportHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
    {
        double frameRate = item->motion()->frameRate();
        if(frameRate != 200.0){
            static format m1(_("The frame rate of a body motion exported as Hrpsys files should be standard value 200, "
                               "but the frame rate of \"%1%\" is %2%. The exported data may cause a problem.\n\n"
                               "Do you continue to export ?"));

            if(!confirm(str(m1 % item->name() % frameRate))){
                return false;
            }
        }

        BodyPtr body;
        BodyItem* bodyItem = item->findOwnerItem<BodyItem>();
        if(bodyItem){
            body = bodyItem->body();
            KinematicFaultChecker* checker = KinematicFaultChecker::instance();
            int numFaults = checker->checkFaults(bodyItem, item, os);
            if(numFaults > 0){
                static string m2(_("A fault has been detected. Please check the report in the MessageView.\n\n"
                                   "Do you continue to export ?"));
                static format m3(_("%1% faults have been detected. Please check the report in the MessageView.\n\n"
                                   "Do you continue to export ?"));

                bool result;

                if(numFaults == 1){
                    result = confirm(m2);
                } else {
                    result = confirm(str(m3 % numFaults));
                }
                
                if(!result){
                    return false;
                }
            }
        }

        if(!item->hasRelativeZmpSeqItem()){
            if(!confirm(_("There is no ZMP data. Do you continue to export ?"))){
                return false;
            }
        }
        
        return saveHrpsysSeqFileSet(*item->motion(), body, filename, os);
    }

    bool bodyMotionItemPreFilter(BodyMotionItem* protoItem, Item* parentItem)
    {
        BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
        if(!bodyItem){
            bodyItem = parentItem->findOwnerItem<BodyItem>();
        }
        if(bodyItem){
            int prevNumJoints = protoItem->jointPosSeq()->numParts();
            int numJoints = bodyItem->body()->numJoints();
            if(numJoints != prevNumJoints){
                protoItem->jointPosSeq()->setNumParts(numJoints, true);
            }
        }
        return true;
    }


    bool bodyMotionItemPostFilter(BodyMotionItem* protoItem, Item* parentItem)
    {
        BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
        if(!bodyItem){
            bodyItem = parentItem->findOwnerItem<BodyItem>();
        }
        if(bodyItem){
            BodyPtr body = bodyItem->body();
            MultiValueSeqPtr qseq = protoItem->jointPosSeq();
            int n = std::min(body->numJoints(), qseq->numParts());
            for(int i=0; i < n; ++i){
                Link* joint = body->joint(i);
                if(joint->defaultJointValue != 0.0){
                    MultiValueSeq::View view = qseq->part(i);
                    std::fill(view.begin(), view.end(), joint->defaultJointValue);
                }
            }
        }
        
        return true;
    }
}


void cnoid::initializeBodyMotionItem(ExtensionManager& ext)
{
    ItemManager& im = ext.itemManager();
    
    im.registerClass<BodyMotionItem>(N_("BodyMotionItem"));

    im.addCreationPanel<BodyMotionItem>(new MultiSeqItemCreationPanel(_("Number of joints")));
    im.addCreationPanelPreFilter<BodyMotionItem>(bodyMotionItemPreFilter);
    im.addCreationPanelPostFilter<BodyMotionItem>(bodyMotionItemPostFilter);

    im.addLoaderAndSaver<BodyMotionItem>(
        _("Body Motion"), "BODY-MOTION-YAML", "yaml",
        bind(loadStandardYamlFormat, _1, _2, _3),  bind(saveAsStandardYamlFormat, _1, _2, _3));

    im.addLoaderAndSaver<BodyMotionItem>(
        _("Hrpsys sequence file set"), "HRPSYS-SEQ-FILE-SET", "pos;vel;acc;hip;waist;gsens;zmp",
        bind(importHrpsysSeqFileSet, _1, _2, _3), bind(exportHrpsysSeqFileSet, _1, _2, _3),
        ItemManager::PRIORITY_CONVERSION);
}


BodyMotionItem::BodyMotionItem()
    : bodyMotion_(new BodyMotion())
{
    initialize();
}


BodyMotionItem::BodyMotionItem(const BodyMotionItem& org)
    : MultiSeqItemBase(org),
      bodyMotion_(new BodyMotion(*org.bodyMotion_))
{
    initialize();
}


void BodyMotionItem::initialize()
{
    jointPosSeqItem_ = new MultiValueSeqItem(bodyMotion_->jointPosSeq());
    jointPosSeqItem_->setName("q");
    addSubItem(jointPosSeqItem_);

    jointPosSeqItem_->sigUpdated().connect(
        bind(&BodyMotionItem::onSubItemUpdated, this, jointPosSeqItem_.get()));

    linkPosSeqItem_ = new MultiAffine3SeqItem(bodyMotion_->linkPosSeq());
    linkPosSeqItem_->setName("p,R");
    addSubItem(linkPosSeqItem_);

    linkPosSeqItem_->sigUpdated().connect(
        bind(&BodyMotionItem::onSubItemUpdated, this, linkPosSeqItem_.get()));

    if(bodyMotion_->hasRelativeZmpSeq()){
        relativeZmpSeqItem(); // create item
    }
    
    unsetAttribute(Item::TREE_EXPANDED_BY_DEFAULT);
}


Vector3SeqItem* BodyMotionItem::relativeZmpSeqItem()
{
    if(!relativeZmpSeqItem_){

        relativeZmpSeqItem_ = new Vector3SeqItem(bodyMotion_->relativeZmpSeq());
        relativeZmpSeqItem_->setName("ZMP");
        addSubItem(relativeZmpSeqItem_);

        relativeZmpSeqItem_->sigUpdated().connect(
            bind(&BodyMotionItem::onSubItemUpdated, this, relativeZmpSeqItem_.get()));
    }
    
    return relativeZmpSeqItem_.get();
}


void BodyMotionItem::updateChildItemLineup()
{
    if(!relativeZmpSeqItem_ && bodyMotion_->hasRelativeZmpSeq()){
        relativeZmpSeqItem(); // create
    }
}


void BodyMotionItem::onSubItemUpdated(Item* childItem)
{
    Item::notifyUpdate();
}


void BodyMotionItem::notifyUpdate()
{
    setInconsistencyWithLastAccessedFile();
    
    jointPosSeqItem_->notifyUpdate();
    linkPosSeqItem_->notifyUpdate();
    if(relativeZmpSeqItem_){
        relativeZmpSeqItem_->notifyUpdate();
    }

    Item::notifyUpdate();
}


ItemPtr BodyMotionItem::doDuplicate() const
{
    return BodyMotionItemPtr(new BodyMotionItem(*this));
}


bool BodyMotionItem::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", lastAccessedFileName());
        archive.write("format", lastAccessedFileFormatId());
        return true;
    }
    return false;
}


bool BodyMotionItem::restore(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        if(load(filename, formatId)){
            return true;
        }
    }
    return false;
}
