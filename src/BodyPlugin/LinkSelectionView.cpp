/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkSelectionView.h"
#include "LinkTreeWidget.h"
#include "BodyBar.h"
#include <cassert>
#include <boost/bind.hpp>
#include <QBoxLayout>
#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
    const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

    class LinkSelectionViewImpl
    {
    public:
        LinkSelectionViewImpl(LinkSelectionView* self);
        
        LinkTreeWidget linkTreeWidget;
    };
}

namespace {
    LinkSelectionView* mainLinkSelectionView = 0;
}


LinkSelectionView* LinkSelectionView::mainInstance()
{
    assert(mainLinkSelectionView);
    return mainLinkSelectionView;
}


LinkSelectionView::LinkSelectionView()
{
    impl = new LinkSelectionViewImpl(this);

    if(!mainLinkSelectionView){
        mainLinkSelectionView = this;
    }
}


LinkSelectionViewImpl::LinkSelectionViewImpl(LinkSelectionView* self)
{
    self->setName(N_("Links"));
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    linkTreeWidget.setFrameShape(QFrame::NoFrame);
    linkTreeWidget.enableCache(true);
    linkTreeWidget.enableArchiveOfCurrentBodyItem(true);
    linkTreeWidget.setListingMode(LinkTreeWidget::LINK_LIST);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    vbox->addWidget(linkTreeWidget.listingModeCombo());
    vbox->addWidget(&linkTreeWidget);
    self->setLayout(vbox);

    BodyBar::instance()->sigCurrentBodyItemChanged().connect(
        bind(&LinkTreeWidget::setBodyItem, &linkTreeWidget, _1));
}


LinkSelectionView::~LinkSelectionView()
{
    delete impl;
}


SignalProxy< boost::signal<void()> > LinkSelectionView::sigSelectionChanged(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.sigSelectionChanged(bodyItem);
}


const std::vector<int>& LinkSelectionView::getSelectedLinkIndices(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.getSelectedLinkIndices(bodyItem);
}


const boost::dynamic_bitset<>& LinkSelectionView::getLinkSelection(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.getLinkSelection(bodyItem);
}


bool LinkSelectionView::makeSingleSelection(BodyItemPtr bodyItem, int linkIndex)
{
    return impl->linkTreeWidget.makeSingleSelection(bodyItem, linkIndex);
}


bool LinkSelectionView::storeState(Archive& archive)
{
    return impl->linkTreeWidget.storeState(archive);
}


bool LinkSelectionView::restoreState(const Archive& archive)
{
    return impl->linkTreeWidget.restoreState(archive);
}
