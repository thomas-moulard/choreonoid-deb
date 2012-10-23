# @author Shin'ichiro Nakaoka

if(ENABLE_INSTALL_RPATH)
  unset(CMAKE_INSTALL_RPATH)
endif()

if(UNIX)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
endif()

file(GLOB plugins "*Plugin" "cnoid/*Plugin" "sample/*Plugin")

foreach(subdirectory ${plugins})
  if(EXISTS ${subdirectory}/CMakeLists.txt)
    add_subdirectory(${subdirectory})
  endif()
endforeach()
