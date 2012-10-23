
#ifndef CNOID_GUIBASE_VRML_TO_OSG_CONVERTER_H_INCLUDED
#define CNOID_GUIBASE_VRML_TO_OSG_CONVERTER_H_INCLUDED

#include <cnoid/VrmlNodes>
#include <osg/Node>
#include "exportdecl.h"

namespace cnoid {

    class VrmlToOsgConverterImpl;

    class CNOID_EXPORT VrmlToOsgConverter
    {
      public:
        VrmlToOsgConverter();
        ~VrmlToOsgConverter();
        
        osg::Node* convert(VrmlNodePtr vrmlNode);
        
      private:
        VrmlToOsgConverterImpl* impl;
    };
    
};


#endif

