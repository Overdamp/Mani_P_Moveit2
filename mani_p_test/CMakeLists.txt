cmake_minimum_required(VERSION 3.8)
project(mani_p_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

ament_package()

# Install Python scripts
install(PROGRAMS
  script/smart_filter_node.py
  script/tf_recorder.py
  script/calibration_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY 
    launch 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install Config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY 
    config 
    DESTINATION share/${PROJECT_NAME}
  )
endif()
