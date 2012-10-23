
option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)

if(BUILD_HELLO_WORLD_SAMPLE)
  set(target CnoidHelloWorldPlugin)
  set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/HelloWorldPlugin)
  add_library(${target} SHARED ${srcdir}/HelloWorldPlugin.cpp)
  target_link_libraries(${target} CnoidBase)
  apply_common_setting_for_plugin(${target})
endif()
