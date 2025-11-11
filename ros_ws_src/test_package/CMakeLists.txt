cmake_minimum_required(VERSION 3.0.2)
project(test_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

# Install Python scripts
install(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# You can uncomment the following lines if you have launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
