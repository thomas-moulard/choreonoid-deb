/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_PROPERTY_VIEW_H_INCLUDED
#define CNOID_GUIBASE_ITEM_PROPERTY_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

    class ItemPropertyViewImpl;

    class ItemPropertyView : public View
    {
      public:
        static void initialize(ExtensionManager* ext);
        
        ItemPropertyView();
        ~ItemPropertyView();

      private:
        ItemPropertyViewImpl* impl;
    };

}

#endif
