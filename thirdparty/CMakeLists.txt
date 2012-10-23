
# @author Shin'ichiro Nakaoka

add_subdirectory(eigen3)
add_subdirectory(yaml-0.1.3)

if(MSVC)
  add_subdirectory(windows)
endif()
