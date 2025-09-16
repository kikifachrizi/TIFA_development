cmake_minimum_required(VERSION 3.0.2)
project(tifa_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(

)

catkin_install_python(PROGRAMS scripts/manual_control.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/tuning_pid_params.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/odometry.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/rekam.py
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/rekamMeans.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/path2.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/coba_odom2.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/pid_record.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/tifa_teleop_key.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/laserscan_to_pointcloud.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/serial_ultrasonic_reader.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/ultrasonic_visualizer.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/odom_listen.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/imu_odom_serial_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



include_directories(
  ${catkin_INCLUDE_DIRS}
)

