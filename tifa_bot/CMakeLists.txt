cmake_minimum_required(VERSION 3.8)
project(tifa_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PostgreSQL REQUIRED)

add_executable(supabase_send_goal_node src/supabase_send_goal_node.cpp)
ament_target_dependencies(supabase_send_goal_node
  rclcpp rclcpp_action geometry_msgs nav2_msgs)
target_include_directories(supabase_send_goal_node PRIVATE ${PostgreSQL_INCLUDE_DIRS})
target_link_libraries(supabase_send_goal_node ${PostgreSQL_LIBRARIES})

install(TARGETS supabase_send_goal_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
