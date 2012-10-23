/**
   @author Shin'ichiro Nakaoka
*/

#include "TreeWidget.h"

using namespace cnoid;

TreeWidget::TreeWidget(QWidget* parent)
    : QTreeWidget(parent)
{
    connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
            this, SLOT(onCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
    
    connect(this, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
            this, SLOT(onItemActivated(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
            this, SLOT(onItemChanged(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemClicked(QTreeWidgetItem*, int)),
            this, SLOT(onItemClicked(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemCollapsed(QTreeWidgetItem*)),
            this, SLOT(onItemCollapsed(QTreeWidgetItem*)));
    
    connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
            this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemEntered(QTreeWidgetItem*, int)),
            this, SLOT(onItemEntered(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemExpanded(QTreeWidgetItem*)),
            this, SLOT(onItemExpanded(QTreeWidgetItem*)));

    connect(this, SIGNAL(itemPressed(QTreeWidgetItem*, int)),
            this, SLOT(onItemPressed(QTreeWidgetItem*, int)));
    
    connect(this, SIGNAL(itemSelectionChanged()),
            this, SLOT(onItemSelectionChanged()));
}


void TreeWidget::onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    sigCurrentItemChanged_(current, previous);
}


void TreeWidget::onItemActivated(QTreeWidgetItem* item, int column)
{
    sigItemActivated_(item, column);
}

void TreeWidget::onItemChanged(QTreeWidgetItem* item, int column)
{
    sigItemChanged_(item, column);
}

void TreeWidget::onItemClicked(QTreeWidgetItem* item, int column)
{
    sigItemClicked_(item, column);
}


void TreeWidget::onItemCollapsed(QTreeWidgetItem* item)
{
    sigItemCollapsed_(item);
}


void TreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
    sigItemDoubleClicked_(item, column);
}


void TreeWidget::onItemEntered(QTreeWidgetItem* item, int column)
{
    sigItemEntered_(item, column);
}


void TreeWidget::onItemExpanded(QTreeWidgetItem* item)
{
    sigItemExpanded_(item);
}


void TreeWidget::onItemPressed(QTreeWidgetItem* item, int column)
{
    sigItemPressed_(item, column);
}


void TreeWidget::onItemSelectionChanged(void)
{
    sigItemSelectionChanged_();
}
