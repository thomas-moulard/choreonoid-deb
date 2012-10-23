/*!
  @file ImageConverter.h
  @brief Header file of Image Converter class
  @author K.FUKUDA
*/

#ifndef CNOID_UTIL_IMAGECONVERTER_H_INCLUDED
#define CNOID_UTIL_IMAGECONVERTER_H_INCLUDED

#include "VrmlNodes.h"
#include "exportdecl.h"

namespace cnoid
{
    class ImageConverter
    {
    private:
        bool initializeSFImage();
        bool loadPNG(const std::string & filePath);
        bool loadJPEG(const std::string & filePath);

    public:
        SFImage* image;
        ImageConverter(void);
        ~ImageConverter(void);

        CNOID_EXPORT SFImage* convert(const std::string& url);
    };
};

#endif

