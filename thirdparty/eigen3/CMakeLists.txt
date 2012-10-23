
if(NOT USE_EXTERNAL_EIGEN)
  if(INSTALL_SDK)
    install(DIRECTORY Eigen DESTINATION include/${CNOID_SUBDIR}
      PATTERN "CMakeLists.txt" EXCLUDE
      PATTERN ".svn" EXCLUDE)
  endif()
endif()
