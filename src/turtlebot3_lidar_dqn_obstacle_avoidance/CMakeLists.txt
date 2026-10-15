cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_lidar_dqn_obstacle_avoidance)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs nav_msgs std_msgs gazebo_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/dqn_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
