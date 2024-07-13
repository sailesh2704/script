cmake_minimum_required(VERSION 3.8)
project(chec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sensor_node src/sensor_node.cpp)
add_executable(switch_node src/switch_node.cpp)
add_executable(emergency_stop src/emergency_stop.cpp)
add_executable(dist_pub src/dist_pub.cpp)
add_executable(switch_pub src/switch_pub.cpp)

ament_target_dependencies(
  sensor_node
  ament_cmake
  rclcpp
  std_msgs
)

ament_target_dependencies(
  switch_node
  ament_cmake
  rclcpp
  std_msgs
)

ament_target_dependencies(
  emergency_stop
  ament_cmake
  rclcpp
  std_msgs
)

ament_target_dependencies(
  dist_pub
  ament_cmake
  rclcpp
  std_msgs
)

ament_target_dependencies(
  switch_pub
  ament_cmake
  rclcpp
  std_msgs
)

install(
  TARGETS sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS switch_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS emergency_stop
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS dist_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS switch_pub
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  ament_cmake
  rclcpp
  std_msgs
)

ament_package()