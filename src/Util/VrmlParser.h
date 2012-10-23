/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_PARSER_H_INCLUDED
#define CNOID_UTIL_VRML_PARSER_H_INCLUDED

#include "VrmlNodes.h"
#include "exportdecl.h"

namespace cnoid {

    class VrmlParserImpl;

    /**
       \brief Parser for VRML97 format

       The VrmlParser class reads a VRML97 file and extract its nodes.
    */
    class CNOID_EXPORT VrmlParser
    {
    public:

        /**
           Constructor. This version of constructor do 'load' mehtod 
           after constructing the object.

           \param filename file name of a target VRML97 file.
        */
        VrmlParser(const std::string& filename);
        VrmlParser();
        ~VrmlParser();

        void setProtoInstanceActualNodeExtractionMode(bool isOn);
        void load(const std::string& filename);

        /**
           This method returns the top node of the next node tree written in the file.
        */
        VrmlNodePtr readNode();

    private:
        VrmlParserImpl* impl;
        void init();
    };
};

#endif
