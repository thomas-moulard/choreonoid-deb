
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()

add_subdirectory(Util)
add_subdirectory(Base)
add_subdirectory(Collision)
add_subdirectory(Body)

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
else()
  unset(CMAKE_INSTALL_RPATH)
endif()

add_subdirectory(Choreonoid)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

if(MSVC)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
endif()

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
else()
  set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()

add_subdirectory(BodyPlugin)
add_subdirectory(PoseSeqPlugin)
add_subdirectory(GRobotPlugin)

foreach(directory CorbaPlugin MediaPlugin BalancerPlugin)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${directory})
    add_subdirectory(${directory})
  endif()
endforeach()
