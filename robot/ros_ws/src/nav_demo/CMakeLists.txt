cmake_minimum_required(VERSION 3.0.2)
project(nav_demo)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  dynamic_reconfigure
  geometry_msgs
  gmapping
  map_server
  message_generation
  move_base
  move_base_msgs
  rospy
  std_msgs
)

add_service_files(FILES GunSweep.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    move_base_msgs
    rospy
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/gun_sweep_server.py
    scripts/mission_controller.py
    scripts/soldier_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch map param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
