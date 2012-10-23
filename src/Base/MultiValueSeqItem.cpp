/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace cnoid;

static bool loadPlainSeqFormat(MultiValueSeqItem* item, const std::string& filename, std::ostream& os)
{
    if(item->seq()->loadPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->ioErrorMessage();
        return false;
    }
}


static bool saveAsPlainSeqFormat(MultiValueSeqItem* item, const std::string& filename, std::ostream& os)
{
    if(item->seq()->saveAsPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->ioErrorMessage();
        return false;
    }
}


template<> void MultiSeqItem<MultiValueSeq>::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiValueSeqItem>(N_("MultiValueSeqItem"));

    ext->itemManager().addCreationPanel<MultiValueSeqItem>(
        new MultiSeqItemCreationPanel(_("Number of values in a frame")));
    
    ext->itemManager().addLoaderAndSaver<MultiValueSeqItem>(
        _("Plain format of a multi value sequence"), "PLAIN-MULTI-VALUE-SEQ", "*",
        bind(loadPlainSeqFormat, _1, _2, _3), bind(saveAsPlainSeqFormat, _1, _2, _3), 
        ItemManager::PRIORITY_CONVERSION);
}

#ifdef WIN32
template class MultiSeqItem<MultiValueSeq>;
#endif
