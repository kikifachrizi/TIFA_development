cmake_minimum_required(VERSION 3.8)
project(supabase_bridge_send_goal)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PostgreSQL REQUIRED)  # libpq

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp rclcpp_action geometry_msgs nav2_msgs)
target_include_directories(bridge_node PRIVATE ${PostgreSQL_INCLUDE_DIRS})
target_link_libraries(bridge_node ${PostgreSQL_LIBRARIES})

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
