cmake_minimum_required(VERSION 3.0.2)
project(tifa_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/ultrasonic_navigation.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_navigation.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_navigation_byTrajectory.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/get_coord.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/csv_to_visualization.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_navigation_topics.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_navigation_topics2.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/db_connect.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/fb_connect.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/host_connect.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/image_connect.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_nav_fb.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})