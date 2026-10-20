cmake_minimum_required(VERSION 3.0.2)
project(carla_vehicle_keyboard_teleop)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/vehicle_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
