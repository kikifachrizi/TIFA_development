cmake_minimum_required(VERSION 3.0.2)
project(deli_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)


catkin_package(

)

catkin_install_python(PROGRAMS scripts/read_track.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/robot_pose.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


include_directories(
  ${catkin_INCLUDE_DIRS}
)


