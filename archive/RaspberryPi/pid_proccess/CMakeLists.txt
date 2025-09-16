cmake_minimum_required(VERSION 3.0.2)
project(pid_proccess)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(

)

catkin_install_python(PROGRAMS scripts/data_processor.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/odometry.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/manual_control.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


          
include_directories(
  ${catkin_INCLUDE_DIRS}
)
