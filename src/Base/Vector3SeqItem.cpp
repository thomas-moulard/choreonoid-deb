/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Vector3SeqItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include "gettext.h"

using namespace cnoid;

void Vector3SeqItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<Vector3SeqItem>(N_("Vector3SeqItem"));
}


Vector3SeqItem::Vector3SeqItem()
    : seq_(new Vector3Seq())
{

}


Vector3SeqItem::Vector3SeqItem(Vector3SeqPtr seq)
  : seq_(seq)
{
    
}

Vector3SeqItem::Vector3SeqItem(const Vector3SeqItem& org)
    : Item(org),
      seq_(new Vector3Seq(*org.seq_))
{

}


Vector3SeqItem::~Vector3SeqItem()
{

}


bool Vector3SeqItem::loadPlainFormat(const std::string& filename)
{
    bool loaded = seq_->loadPlainFormat(filename);
    notifyUpdate();
    return loaded;
}


bool Vector3SeqItem::saveAsPlainFormat(const std::string& filename)
{
    return seq_->saveAsPlainFormat(filename);
}


ItemPtr Vector3SeqItem::doDuplicate() const
{
    return new Vector3SeqItem(*this);
}


void Vector3SeqItem::notifyUpdate()
{
    setInconsistencyWithLastAccessedFile();
    Item::notifyUpdate();
}


bool Vector3SeqItem::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", lastAccessedFileName());
        archive.write("format", lastAccessedFileFormatId());
        return true;
    }
    return false;
}


bool Vector3SeqItem::restore(const Archive& archive)
{
    std::string filename, formatId;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        if(load(filename, formatId)){
            return true;
        }
    }
    return false;
}


void Vector3SeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Frame rate"), seq_->frameRate());
    putProperty(_("Number of frames"), seq_->numFrames());
    putProperty(_("Time length"), seq_->numFrames() / seq_->frameRate());
    putProperty(_("Time step"), 1.0 / seq_->frameRate());
}
