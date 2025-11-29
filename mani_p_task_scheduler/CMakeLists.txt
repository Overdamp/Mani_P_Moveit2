cmake_minimum_required(VERSION 3.8)
project(mani_p_task_scheduler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  script/task_scheduler.py
  script/BT_scheduler.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY 
    launch 
    DESTINATION share/${PROJECT_NAME}
  )
endif()
