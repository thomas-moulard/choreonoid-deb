
install(DIRECTORY models projects DESTINATION ${CNOID_SHARE_SUBDIR})

if(INSTALL_SDK)
  install(DIRECTORY sampleplugins DESTINATION ${CNOID_SHARE_SUBDIR})
endif()

