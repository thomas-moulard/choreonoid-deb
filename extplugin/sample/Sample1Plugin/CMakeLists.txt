
option(BUILD_SAMPLE1_SAMPLE "Building a sample plugin \"Sample1Plugin\"" OFF)

if(BUILD_SAMPLE1_SAMPLE)
  set(target CnoidSample1Plugin)
  set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/Sample1Plugin)
  add_library(${target} SHARED ${srcdir}/Sample1Plugin.cpp)
  target_link_libraries(${target} CnoidBodyPlugin)
  apply_common_setting_for_plugin(${target})
endif()
