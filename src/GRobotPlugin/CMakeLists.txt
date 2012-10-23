# @author Shin'ichiro Nakaoka

option(BUILD_GROBOT_PLUGIN "Building GRobotPlugin" OFF)

if(NOT BUILD_GROBOT_PLUGIN)
  return()
endif()

set(target CnoidGRobotPlugin)

make_gettext_mofiles(mofiles)

QT4_ADD_RESOURCES(RC_SRCS GRobotPlugin.qrc)

add_library(${target} SHARED
  GRobotPlugin.cpp
  GRobotBar.cpp
  GRobotControllerItem.cpp
  GRobotController.cpp
  ${mofiles}
  ${RC_SRCS}
  )


if(UNIX)
  target_link_libraries(${target} CnoidBodyPlugin ${Boost_THREAD_LIBRARY} pthread rt)
elseif(MSVC)
  target_link_libraries(${target} CnoidBodyPlugin ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
  set_target_properties(${target} PROPERTIES COMPILE_DEFINITIONS "BOOST_ASIO_DISABLE_STD_ARRAY")
endif()

apply_common_setting_for_plugin(${target})

install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})

# set(CMAKE_BUILD_TYPE Debug)

set(customizer GRobotCustomizer)
add_library(${customizer} SHARED GRobotCustomizer.cpp ${mofile})

set_target_properties(${customizer} PROPERTIES PREFIX "")
set_target_properties(${customizer} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/customizer)

install(TARGETS ${customizer}
  RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug
  LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug)
