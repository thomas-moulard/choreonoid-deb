
# @author Shin'ichiro Nakaoka

set(sources
  ColdetModel.cpp
  ColdetModelPair.cpp
  CollisionPairInserter.cpp
  TriOverlap.cpp
  SSVTreeCollider.cpp
  DistFuncs.cpp
  Opcode/Ice/IceAABB.cpp
  Opcode/Ice/IceContainer.cpp
  Opcode/Ice/IceIndexedTriangle.cpp
  Opcode/Ice/IceMatrix3x3.cpp
  Opcode/Ice/IceMatrix4x4.cpp
  Opcode/Ice/IceRevisitedRadix.cpp
  Opcode/Ice/IceHPoint.cpp
  Opcode/Ice/IceRandom.cpp
  Opcode/Ice/IcePoint.cpp
  Opcode/OPC_AABBTree.cpp
  Opcode/OPC_BaseModel.cpp
  Opcode/OPC_Collider.cpp
  Opcode/OPC_Common.cpp
  Opcode/OPC_MeshInterface.cpp
  Opcode/OPC_Model.cpp
  Opcode/OPC_OptimizedTree.cpp
  Opcode/OPC_TreeBuilders.cpp
  Opcode/OPC_TreeCollider.cpp
  Opcode/OPC_VolumeCollider.cpp 
  Opcode/OPC_RayCollider.cpp
  Opcode/OPC_SphereCollider.cpp
  Opcode/OPC_Picking.cpp
  )

set(headers
  CollisionData.h
  ColdetModel.h
  ColdetModelPair.h
  CollisionPairInserter.h
  exportdecl.h
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Opcode)

set(target CnoidCollision)
add_library(${target} SHARED ${sources})
apply_common_setting_for_library(${target} "${headers}")
