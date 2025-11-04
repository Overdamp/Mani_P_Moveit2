cmake_minimum_required(VERSION 3.8)
project(mani_p_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
######################################################

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  launch
  meshes
  robot
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
