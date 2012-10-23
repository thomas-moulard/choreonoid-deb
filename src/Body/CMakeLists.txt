
# @author Shin'ichiro Nakaoka

set(target CnoidBody)

set(sources
  Body.cpp
  LeggedBody.cpp
  BodyCustomizerInterface.cpp
  BodyLoader.cpp
  ModelNodeSet.cpp
  Link.cpp
  LinkTraverse.cpp
  LinkPath.cpp
  JointPath.cpp
  LinkGroup.cpp
  ColdetLinkPair.cpp
  Sensor.cpp
  CompositeIK.cpp
  PinDragIK.cpp
  PenetrationBlocker.cpp
  World.cpp
  ForwardDynamics.cpp
  ForwardDynamicsABM.cpp
  ForwardDynamicsCBM.cpp
  InverseDynamics.cpp
  Jacobian.cpp
  MassMatrix.cpp
  ConstraintForceSolver.cpp
  BodyMotion.cpp
  BodyMotionPoseProvider.cpp
  PoseProviderToBodyMotionConverter.cpp
  BodyMotionUtil.cpp
  )

set(headers
  Body.h
  LeggedBody.h
  BodyCustomizerInterface.h
  BodyLoader.h
  ModelNodeSet.h
  Link.h
  LinkTraverse.h
  LinkPath.h
  JointPath.h
  LinkGroup.h
  ColdetLinkPair.h
  Sensor.h
  InverseKinematics.h
  CompositeIK.h
  PinDragIK.h
  PenetrationBlocker.h
  World.h
  ForwardDynamics.h
  ForwardDynamicsABM.h
  ForwardDynamicsCBM.h
  InverseDynamics.h
  Jacobian.h
  MassMatrix.h
  ConstraintForceSolver.h
  PoseProvider.h
  BodyMotion.h
  BodyMotionPoseProvider.h
  PoseProviderToBodyMotionConverter.h
  BodyMotionUtil.h
  exportdecl.h
  )



add_library(${target} SHARED ${sources} ${headers})

if(UNIX)
  target_link_libraries(${target} CnoidUtil CnoidCollision dl)
elseif(MSVC)
  target_link_libraries(${target} CnoidUtil CnoidCollision)
endif()

apply_common_setting_for_library(${target} "${headers}")

# Body Customizers
set(BODY_CUSTOMIZERS ${BODY_CUSTOMIZERS} CACHE FILEPATH "Source files of body customizers")

if(BODY_CUSTOMIZERS)

  foreach(src ${BODY_CUSTOMIZERS})

    get_filename_component(customizer ${src} NAME_WE)
    add_library(${customizer} SHARED ${src})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    set_target_properties(${customizer} PROPERTIES PREFIX "")
    set_target_properties(${customizer} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/customizer)

    install(TARGETS ${customizer}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR}/customizer CONFIGURATIONS Release Debug)

  endforeach()
endif()

install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
