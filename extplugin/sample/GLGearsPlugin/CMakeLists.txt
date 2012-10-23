
option(BUILD_GL_GEARS_SAMPLE "Building a GL Gears sample plugin" OFF)

if(BUILD_GL_GEARS_SAMPLE)
  set(srcdir ${PROJECT_SOURCE_DIR}/share/sampleplugins/GLGearsPlugin)
  set(sources 
    ${srcdir}/GLGearsPlugin.cpp
    ${srcdir}/GLGearsView.cpp
    )
  set(target CnoidGLGearsPlugin)
  add_library(${target} SHARED ${sources})
  target_link_libraries(${target} CnoidBase)
  apply_common_setting_for_plugin(${target})
endif()
