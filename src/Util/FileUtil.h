/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_FILE_UTIL_H_INCLUDED
#define CNOID_UTIL_FILE_UTIL_H_INCLUDED

#include <boost/filesystem.hpp>
#include <string>
#include "exportdecl.h"

namespace cnoid {

    CNOID_EXPORT void makePathCompact(
        const boost::filesystem::path& path,
        boost::filesystem::path& out_compact);

    CNOID_EXPORT bool findSubDirectory(
        const boost::filesystem::path& directory,
        const boost::filesystem::path& path,
        boost::filesystem::path& out_subdirectory);

    CNOID_EXPORT bool findRelativePath(
        const boost::filesystem::path& from,
        const boost::filesystem::path& to,
        boost::filesystem::path& out_relativePath);

//#ifdef _WIN32
#if 0
    CNOID_EXPORT const std::string toActualPathName(const std::string& pathName);
#else
    inline const std::string& toActualPathName(const std::string& pathName) {
        return pathName;
    }
#endif
    
}

#endif
