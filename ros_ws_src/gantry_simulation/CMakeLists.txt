cmake_minimum_required(VERSION 3.0.2)
project(gantry_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz_satellite
  std_msgs
  topological_navigation
  actionlib
  move_base_msgs

)
