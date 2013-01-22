#!/bin/sh
# WARNING: to be run in the package top source directory.

# Remove 3rd-party dependencies as they are not needed.
rm -rf thirdparty
# Remove proprietary plug-ins we cannot package anyway.
rm -rf proprietary
# Comment the add_subdirectory corresponding to the 3rd party directory.
sed -i -e 's|add_subdirectory(thirdparty)|#add_subdirectory(thirdparty)|g' \
    CMakeLists.txt
