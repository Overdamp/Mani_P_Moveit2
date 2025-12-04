cmake_minimum_required(VERSION 3.22)
project(mani_p_moveit_config3)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  script/spawn_shelf_dynamic.py
  script/smart_soft_snap.py
  script/spawn_cubes.py
  script/test_gripper_to_tag.py
  script/approach_tag_smart.py
  script/cartesian_push.py
  script/visual_servo_align.py
  script/coarse2fine_task.py
  script/adjust_level.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY 
    launch 
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
