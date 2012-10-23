
# @author Shin'ichiro Nakaoka

# set(CMAKE_BUILD_TYPE Debug)

set(sources
  BodyPlugin.cpp
  BodyItem.cpp
  WorldItem.cpp
  BodyMotionItem.cpp
  SimulatorItem.cpp
  KinematicsSimulatorItem.cpp
  DynamicsSimulatorItem.cpp
  BodyMotionEngine.cpp
  KinematicFaultChecker.cpp
  #FilterDialogs.cpp
  SceneBodyManager.cpp
  SceneBody.cpp
  SceneWorld.cpp
  OsgCollision.cpp
  BodyBar.cpp
  KinematicsBar.cpp
  SimulationBar.cpp
  LinkTreeWidget.cpp
  LinkSelectionView.cpp
  BodyLinkView.cpp
  JointSliderView.cpp
  #WorldView.cpp
  MultiAffine3SeqGraphView.cpp
  MultiValueSeqGraphView.cpp
  )

set(headers
  BodyItem.h
  WorldItem.h
  BodyMotionItem.h
  SimulatorItem.h
  KinematicFaultChecker.h
  SceneBodyManager.h
  SceneBody.h
  SceneWorld.h
  BodyBar.h
  KinematicsBar.h
  LinkTreeWidget.h
  LinkSelectionView.h
  exportdecl.h
  gettext.h
  )

QT4_WRAP_CPP(sources
  LinkTreeWidget.h
  )

QT4_ADD_RESOURCES(RC_SRCS BodyPlugin.qrc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(target CnoidBodyPlugin)
make_gettext_mofiles(mofiles)
add_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})
target_link_libraries(${target} CnoidUtil CnoidBase CnoidBody)
apply_common_setting_for_plugin(${target} "${headers}")
