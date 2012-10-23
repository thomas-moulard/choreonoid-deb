/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_WORLD_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_WORLD_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

    class WorldViewImpl;
    
    class WorldView : public cnoid::View
    {
    public:
        WorldView();
        virtual ~WorldView();
        
    private:
        friend class WorldViewImpl;
        WorldViewImpl* impl;
    };
}


#endif
