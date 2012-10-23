# @author Shin'ichiro Nakaoka

set(target choreonoid)

set(sources main.cpp)

QT4_ADD_RESOURCES(RC_SRCS choreonoid.qrc)

if(WIN32)
  set(sources ${sources} choreonoid.rc)
endif()

add_executable(${target} ${sources} ${RC_SRCS})

target_link_libraries(${target} CnoidUtil CnoidBase)
set_target_properties(${target} PROPERTIES PROJECT_LABEL Application)

if(MSVC)
  option(USE_SUBSYSTEM_CONSOLE "Attaching a console of Windows for debugging" OFF)
  if(NOT USE_SUBSYSTEM_CONSOLE)
    set_target_properties(${target} PROPERTIES WIN32_EXECUTABLE true)
  else()
    set_target_properties(${target} PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")
  endif()
endif()

apply_common_setting_for_target(${target})

if(MSVC)
  set_target_properties(${target} PROPERTIES DEBUG_POSTFIX -debug)
endif()

install(TARGETS ${target} RUNTIME DESTINATION bin CONFIGURATIONS Release Debug)
