
# @author Shin'ichiro Nakaoka

#set(CMAKE_BUILD_TYPE Debug)

set(target CnoidPoseSeqPlugin)

set(sources
  PoseSeqPlugin.cpp
  Pose.cpp
  PoseSeq.cpp
  PoseSeqInterpolator.cpp
  PronunSymbol.cpp
  PoseFilters.cpp
  LipSyncTranslator.cpp
  PoseSeqItem.cpp
  PoseSeqEngine.cpp
  PoseSeqViewBase.cpp
  #PoseSeqView.cpp
  PoseRollView.cpp
  BodyMotionGenerationBar.cpp
  FcpFileLoader.cpp
)

set(headers
  Pose.h
  PoseSeq.h
  PoseSeqInterpolator.h
  PoseFilters.h
  PoseSeqItem.h
  BodyMotionGenerationBar.h
  exportdecl.h
  gettext.h
  )

QT4_ADD_RESOURCES(RC_SRCS PoseSeqPlugin.qrc)

make_gettext_mofiles(mofiles)
add_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})
target_link_libraries(${target} CnoidUtil CnoidBody CnoidBodyPlugin)
apply_common_setting_for_plugin(${target} "${headers}")
