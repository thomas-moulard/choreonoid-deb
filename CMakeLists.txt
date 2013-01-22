
# @author Shin'ichiro Nakaoka

cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0003 NEW)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

project(Choreonoid)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

set(CNOID_MAJOR_VERSION 1) 
set(CNOID_MINOR_VERSION 1)
set(CNOID_PATCH_VERSION 0)
set(CNOID_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION})
set(CNOID_FULL_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION}.${CNOID_PATCH_VERSION})

include(FindPkgConfig)
include(CheckIncludeFiles)
include(FindPythonLibs)


if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(ADDITIONAL_CXX_FLAGS_RELEASE ${ADDITIONAL_CXX_FLAGS_RELEASE} CACHE STRING "Additional c++ compiler optimization flags")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_CXX_FLAGS_RELEASE} ${ADDITIONAL_CXX_FLAGS_RELEASE}")

if(UNIX)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
endif()

if(MSVC)

  set(ext_compiler_options "/Ob2 /Ox /Oi /Ot /Oy /GT /GS- /arch:SSE /arch:SSE2 /fp:fast")
  set(ext_linker_options "")

  option(MSVC_ENABLE_GLOBAL_OPTIMIZATION "Global optimization with compiler option /GL and linker option /LTCG" ON)
  if(MSVC_ENABLE_GLOBAL_OPTIMIZATION)
    set(ext_compiler_options "${ext_compiler_options} /GL")
    set(ext_linker_options "${ext_linker_options} /LTCG")
  endif()

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${ext_compiler_options}")
  set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
endif()

if(WIN32)
  set(DEFAULT_INSTALL_SDK OFF)
  set(DEFAULT_INSTALL_DEPENDENCIES ON)
else()
  set(DEFAULT_INSTALL_SDK ON)
  set(DEFAULT_INSTALL_DEPENDENCIES OFF)
endif()

option(INSTALL_SDK "Installs SDK files such as header files" ${DEFAULT_INSTALL_SDK})
option(INSTALL_DEPENDENCIES "Installs the external libraries" ${DEFAULT_INSTALL_DEPENDENCIES})

option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" OFF)

if(ENABLE_INSTALL_RPATH)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  #SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
else()
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  SET(CMAKE_INSTALL_RPATH "")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
endif()

# commands
if(UNIX)
  set(RMDIR rm -fr)
elseif(WIN32)
  set(RMDIR rmdir /S/Q)
endif()

# check "dlfcn.h" for using dlopen() and dlclose()
if(UNIX)
  check_include_files(dlfcn.h HAVE_DLFCN_H)
  if(NOT HAVE_DLFCN_H)
    message(FATAL_ERROR "Could not find dlfcn.h")
  endif()
endif()


# gettext
option(CNOID_ENABLE_GETTEXT "Enable the gettext library and translation messages for the internationalization" ON)

if(CNOID_ENABLE_GETTEXT)
  if(WIN32)
    FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows/bin)
  else()
    FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt)
  endif()
  if(NOT GETTEXT_MSGFMT_EXECUTABLE)
    message(FATAL_ERROR "Could not find the msgfmt command and gettext cannot be enabled.")
  endif()
  get_filename_component(GETTEXT_BINARY_DIR ${GETTEXT_MSGFMT_EXECUTABLE} PATH)
  get_filename_component(GETTEXT_DIR ${GETTEXT_BINARY_DIR} PATH)
  set(GETTEXT_INCLUDE_DIR ${GETTEXT_DIR}/include)
  set(GETTEXT_LIBRARY_DIR ${GETTEXT_DIR}/lib)
  include_directories(${GETTEXT_INCLUDE_DIR})
  link_directories(${GETTEXT_LIBRARY_DIR})
  if(WIN32)
    set(GETTEXT_LIBRARIES intl)
  else()
    set(GETTEXT_LIBRARIES "")
  endif()
else()
  set(GETTEXT_LIBRARIES "")
endif()
  

# boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.43", "1.43.0", "1.44", "1.44.0", "1.45", "1.45.0", "1.46", "1.46.0" "1.47" "1.47.0")
find_package(Boost 1.36.0 REQUIRED COMPONENTS system filesystem program_options regex signals thread iostreams date_time python)
if(NOT Boost_FOUND)
  set(BOOST_ROOT ${BOOST_ROOT} CACHE PATH "set the directory of the boost library")
  message(FATAL_ERROR "Boost cannot be found. Please specify the boost top directory to BOOST_ROOT.")
endif()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE BOOST_DISABLE_ASSERTS)

# todo: update filesystem version to 3
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_FILESYSTEM_VERSION=2)


if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_ALL_DYN_LINK ${BOOST_LIB_DIAGNOSTIC})

  #MSVC 2010 problem
  if(MSVC_VERSION EQUAL 1600)
    #add_definitions(-D)

    # for VC++2010 Express Edition
    IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
      SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
    ENDIF()

    set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
      _HAS_CPP0X=0
      _ITERATOR_DEBUG_LEVEL=0 # needed to use boost.MultiArray with the Visual C++ 2010 Debug mode
      BOOST_NO_0X_HDR_TYPEINDEX
      BOOST_NO_0X_HDR_TUPLE
    )
  endif()
endif()

# eigen
option(USE_EXTERNAL_EIGEN "Use the eigen library installed in an external directory" OFF)
if(USE_EXTERNAL_EIGEN)
  if(UNIX)
    pkg_check_modules(EIGEN REQUIRED eigen3)
  elseif(MSVC)
    set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} CACHE PATH "set the header directory of the Eigen library")
  endif()
  include_directories(${EIGEN_INCLUDE_DIRS})
else()
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen3)
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_NO_DEBUG)
if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_DONT_ALIGN)
endif()


# qt
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
#set(QT_USE_QTTEST TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)


# OpenSceneGraph
set(OSG_DIR ${OSG_DIR} CACHE PATH "set the directory of the OpenSceneGraph library")

if(MSVC)
  if(NOT OSG_DIR)
    message(FATAL_ERROR "Please specify the OpenSceneGraph top directory to OSG_DIR.")
  endif()
endif()

if(OSG_DIR)
  include_directories(${OSG_DIR}/include)
  link_directories(${OSG_DIR}/lib)
endif()


# libyaml
if(UNIX)
  set(USE_EXTERNAL_YAML_DEFAULT ON)
elseif(MSVC)
  set(USE_EXTERNAL_YAML_DEFAULT OFF)
endif()
option(USE_EXTERNAL_YAML "Use the yaml library installed in an external directory" ${USE_EXTERNAL_YAML_DEFAULT})

if(USE_EXTERNAL_YAML)
  set(LIBYAML_DIR CACHE PATH "set the top directory of the libyaml")
  if(LIBYAML_DIR)
    include_directories(${LIBYAML_DIR}/include)
    link_directories(${LIBYAML_DIR}/lib)
  endif()
else()
  include_directories(${Choreonoid_SOURCE_DIR}/thirdparty/yaml-0.1.3/include)
endif()

if(UNIX)
  # png
  find_package(PNG REQUIRED)
  include_directories(${PNG_INCLUDE_DIR})
  # jpeg
  find_package(JPEG REQUIRED)
  include_directories(${JPEG_INCLUDE_DIR})
elseif(MSVC)
  set(PNG_LIBRARY libpng)
  set(JPEG_LIBRARY jpeg)
endif()

# doxygen
find_package(Doxygen)

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CNOID_DEBUG)

if(UNIX)
  
  option(PUT_ALL_WARNINGS "put all warnings in compile" OFF)
  if(PUT_ALL_WARNINGS)
    list(APPEND CMAKE_C_FLAGS "-Wall")
    list(APPEND CMAKE_CXX_FLAGS "-Wall")
  endif()

  option(CHECK_UNRESOLVED_SYMBOLS "check unresolved symbols in the object files when creating shared libraries" OFF)
  #mark_as_advanced(CHECK_UNRESOLVED_SYMBOLS)
  if(CHECK_UNRESOLVED_SYMBOLS)
    list(APPEND CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,--warn-unresolved-symbols")
  endif()

elseif(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS NOMINMAX _USE_MATH_DEFINES)
  #list(APPEND CMAKE_CXX_FLAGS "/wd4250 /wd4251 /wd4275")
endif()

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CNOID_DIR ${CMAKE_INSTALL_PREFIX})
set(CNOID_SUBDIR choreonoid-${CNOID_VERSION})
set(CNOID_PLUGIN_SUBDIR lib/${CNOID_SUBDIR})

if(WIN32)
  set(CNOID_HEADER_SUBDIR "include")
  set(CNOID_SHARE_SUBDIR "share")
  set(CNOID_DOC_SUBDIR "share/doc")
else()
  set(CNOID_HEADER_SUBDIR "include/${CNOID_SUBDIR}")
  set(CNOID_SHARE_SUBDIR "share/${CNOID_SUBDIR}")
  set(CNOID_DOC_SUBDIR "share/doc/${CNOID_SUBDIR}")
endif()

set(CNOID_SHARE_DIR "${CNOID_DIR}/${CNOID_SHARE_SUBDIR}")
set(CNOID_LOCALE_SUBDIR "share/locale")


link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})


# Document installaiton
install(FILES NEWS DESTINATION ${CNOID_DOC_SUBDIR})
install(FILES LICENSE DESTINATION ${CNOID_DOC_SUBDIR})


if(MSVC)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/include)
  link_directories(${PROJECT_SOURCE_DIR}/thirdparty/windows/lib)
endif()

#add_subdirectory(thirdparty)


function(apply_common_setting_for_target target)

  set(headers ${ARGV1})

  if(MSVC)
    if(MSVC_IDE)
      if(headers)
        source_group("Header Files" FILES ${headers})
      endif()
      #set_target_properties(${target} PROPERTIES PREFIX "../" IMPORT_PREFIX "../")
    endif()

    get_target_property(existing_compile_flags ${target} COMPILE_FLAGS)
    if(existing_compile_flags STREQUAL existing_compile_flags-NOTFOUND)
      set(existing_compile_flags "")
    endif()
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${existing_compile_flags} /MP /wd4250 /wd4251 /wd4275 /wd4819")

    get_target_property(existing_link_flags ${target} LINK_FLAGS)
    if(existing_link_flags STREQUAL existing_link_flags-NOTFOUND)
      set(existing_link_flags "")
    endif()
    set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} /NODEFAULTLIB:LIBCMT")

    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d)
  endif()

endfunction()

function(apply_common_setting_for_library target)

  set(headers ${ARGV1})
  set_target_properties(${target} PROPERTIES VERSION ${CNOID_VERSION})
  apply_common_setting_for_target(${target} "${headers}")

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    if(headers)
      get_filename_component(subdir ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/src/${subdir})
    endif()
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()

function(apply_common_setting_for_plugin target)

  set(headers ${ARGV1})
  apply_common_setting_for_target(${target} "${headers}")

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    if(headers)
      get_filename_component(subdir ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/src/${subdir})
    endif()
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()


function(make_gettext_mofiles out_mofiles)

  configure_file(${PROJECT_SOURCE_DIR}/src/Util/gettext.h.in ${CMAKE_CURRENT_SOURCE_DIR}/gettext.h)
 
  if(NOT CNOID_ENABLE_GETTEXT)
    set(${out_mofiles} "")
    return()
  endif()

  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)

  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    set(mofile ${PROJECT_BINARY_DIR}/${message_location}/${target}-${CNOID_VERSION}.mo)
    add_custom_command(
      OUTPUT ${mofile}
      COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} -o ${mofile} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mofiles ${mofile})
    install(FILES ${mofile} DESTINATION "${CNOID_LOCALE_SUBDIR}/${lang}/LC_MESSAGES")
  endforeach()

  set(${out_mofiles} ${mofiles} PARENT_SCOPE)

endfunction()


function(install_external_libraries dll_dir lib_dir)

  set(libraries ${ARGV})

  list(REMOVE_AT libraries 0 1)
  if(INSTALL_DEPENDENCIES AND MSVC)
    set(conf general)
    foreach(library ${libraries})
      #message(STATUS ${library})
      if(library STREQUAL general)
	set(conf general)
      elseif(library STREQUAL optimized)
	set(conf optimized)
      elseif(library STREQUAL debug)
	set(conf debug)
      else()
	get_filename_component(filename ${library} NAME_WE)
	if(conf STREQUAL general)
	  if(dll_dir)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin)
	  endif()
	  if(lib_dir AND INSTALL_SDK)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib)
	  endif()
	elseif(conf STREQUAL optimized)
	  if(dll_dir)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin CONFIGURATIONS Release)
	  endif()
	  if(lib_dir AND INSTALL_SDK)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib CONFIGURATIONS Release)
	  endif()
	elseif(conf STREQUAL debug)
	  if(dll_dir)
	    install(PROGRAMS ${dll_dir}/${filename}.dll DESTINATION bin CONFIGURATIONS Debug)
	  endif()
	  if(lib_dir AND INSTALL_SDK)
	    install(PROGRAMS ${lib_dir}/${filename}.lib DESTINATION lib CONFIGURATIONS Debug)
	  endif()
	endif()
      endif()
    endforeach()
  endif()

endfunction()


add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(extplugin)
add_subdirectory(share)


if(EXISTS ${PROJECT_SOURCE_DIR}/admin)
  add_subdirectory(admin)
endif()

configure_file(Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile @ONLY)

if(EXISTS ${PROJECT_SOURCE_DIR}/test)
  if(EXISTS ${PROJECT_SOURCE_DIR}/test/CMakeLists.txt)
    add_subdirectory(test)
  endif()
endif()



# CPack
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Choreonoid")
set(CPACK_PACKAGE_VENDOR "Shin'ichiro Nakaoka, AIST")
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${CNOID_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${CNOID_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${CNOID_PATCH_VERSION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Choreonoid ${CNOID_VERSION}")
set(CPACK_PACKAGE_INSTALL_REGISTORY_KEY "Choreonoid ${CNOID_VERSION}")

set(CPACK_GENERATOR NSIS)
if(WIN32)
  set(CPACK_GENERATOR NSIS)
  set(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}\\\\src\\\\Choreonoid\\\\icon\\\\choreonoid.ico")
  set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\choreonoid.exe")
  set(CPACK_NSIS_DISPLAY_NAME "Choreonoid ${CNOID_VERSION}")
  set(CPACK_NSIS_CONTACT "choreonoid@m.aist.go.jp")
  set(CPACK_NSIS_MODIFY_PATH ON)
  set(CPACK_PACKAGE_EXECUTABLES "choreonoid;Choreonoid")
  set(CPACK_CREATE_DESKTOP_LINKS choreonoid)
  set(CPACK_NSIS_URL_INFO_ABOUT "http://choreonoid.org")
  
  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "!include \\\"FileAssociation.nsh\\\"\n\\\${registerExtension} \\\"\$INSTDIR\\\\bin\\\\choreonoid.exe\\\" \\\".cnoid\\\" \\\"ChoreonoidProject\\\" \nSystem::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'")
  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "\\\${unregisterExtension} \\\".cnoid\\\" \\\"ChoreonoidProject\\\" \nSystem::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'")

elseif(UNIX)
  set(CPACK_GENERATOR DEB)
  set(CPACK_PACKAGE_FILE_NAME "choreonoid_${CNOID_FULL_VERSION}_i386")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libgcc1, libstdc++6, libpng12-0, libjpeg62, libyaml-0-2, zlib1g, libboost-filesystem1.42.0, libboost-date-time1.42.0, libboost-program-options1.42.0, libboost-python1.42.0, libboost-regex1.42.0, libboost-signals1.42.0, libboost-system1.42.0, libboost-thread1.42.0, libqtcore4, libqtgui4, libqt4-test, libqt4-opengl, libopenscenegraph65")
  if(INSTALL_SDK)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libstdc++-dev, libboost-dev, libboost-program-options-dev, libboost-python-dev, libboost-signals-dev, libqt4-dev, libqt4-opengl-dev, libopenscenegraph-dev")
  endif()
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Shin'ichiro Nakaoka")
endif()

#set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 1)

include(CPack)

add_subdirectory(misc)
