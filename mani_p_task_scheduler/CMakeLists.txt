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
  script/approach_action_server.py
  script/visual_servo_action_server.py
  script/adjust_level_action_server.py
  script/move_linear_action_server.py
  script/gripper_action_server.py
  script/action_scheduler.py
  script/cube_sorting_bt.py
  script/simple_shelf_mover.py
  script/simple_bt_example.py
  script/BT_scheduler.py    
  script/check_tags.py
  script/shelf_action_server.py
  script/debug_shelf_alignment.py
  script/move_to_named_target_action_server.py
  script/pick_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY 
    launch 
    DESTINATION share/${PROJECT_NAME}
  )
endif()
