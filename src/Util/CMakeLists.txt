
# @author Shin'ichiro Nakaoka

# set(CMAKE_BUILD_TYPE Release)

configure_file(Config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/Config.h)

set(sources
   EasyScanner.cpp
   NullOut.cpp
   SeqBase.cpp
   Vector3Seq.cpp
   MultiAffine3Seq.cpp
   MultiValueSeq.cpp
   PlainSeqFormatLoader.cpp
   Interpolator.cpp
   RangeLimiter.cpp
   TriangleMeshShaper.cpp
   Triangulator.cpp
   ImageConverter.cpp
   FileUtil.cpp
   Utf8.cpp
   VrmlNodes.cpp
   VrmlParser.cpp
   VrmlWriter.cpp
   YamlNodes.cpp
   YamlReader.cpp
   YamlWriter.cpp
   EigenUtil.cpp
  )

set(headers
  EasyScanner.h
  GaussianFilter.h
  ImageConverter.h
  Interpolator.h
  MultiAffine3Seq.h
  MultiSeq.h
  MultiValueSeq.h
  NullOut.h
  PlainSeqFormatLoader.h
  RangeLimiter.h
  Referenced.h
  Seq.h
  SeqBase.h
  TimeMeasure.h
  Sleep.h
  TriangleMeshShaper.h
  Triangulator.h
  Vector3Seq.h
  FileUtil.h
  Utf8.h
  VrmlNodes.h
  VrmlParser.h
  VrmlWriter.h
  YamlNodes.h
  YamlReader.h
  YamlWriter.h
  EigenTypes.h
  EigenUtil.h
  EigenYaml.h
  exportdecl.h
  )

set(target CnoidUtil)

add_library(${target} SHARED ${sources} ${headers})

if(UNIX)
  target_link_libraries(${target}
    yaml ${PNG_LIBRARY} ${JPEG_LIBRARY}
    ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SIGNALS_LIBRARY}
    ${GETTEXT_LIBRARIES}
    )
elseif(MSVC)
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "YAML_DECLARE_STATIC")
  if(USE_EXTERNAL_YAML)
    target_link_libraries(${target}
       optimized yaml debug yamld
       libpng jpeg
       ${GETTEXT_LIBRARIES})
  else()
    target_link_libraries(${target} yaml libpng jpeg ${GETTEXT_LIBRARIES})
  endif()
endif()

apply_common_setting_for_library(${target} "${headers}")

install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
  ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SIGNALS_LIBRARY})

if(CNOID_ENABLE_GETTEXT)
  install_external_libraries(${GETTEXT_BINARY_DIR} ${GETTEXT_LIBRARY_DIR} ${GETTEXT_LIBRARIES})
endif()

