cmake_minimum_required(VERSION 3.8)
project(ros2_message_flow_testcases)

# Default to C++17 for std::optional
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)

add_library(utils
  src/utils.cpp
)
ament_target_dependencies(utils
  rcpputils
)
target_include_directories(utils PUBLIC "include/")
install(TARGETS
  utils
  DESTINATION lib
)

# Source
add_executable(source
  src/source.cpp
)
ament_target_dependencies(source
  rclcpp
  std_msgs
)
target_link_libraries(source utils)

# 1-to-1
add_executable(one_to_one
  src/one_to_one.cpp
)
ament_target_dependencies(one_to_one
  rclcpp
  std_msgs
)
target_link_libraries(one_to_one utils)

# 1-to-N
add_executable(one_to_n
  src/one_to_n.cpp
)
ament_target_dependencies(one_to_n
  rclcpp
  std_msgs
)
target_link_libraries(one_to_n utils)

# N-to-M
add_executable(n_to_m
  src/n_to_m.cpp
)
ament_target_dependencies(n_to_m
  rclcpp
  std_msgs
)
target_link_libraries(n_to_m utils)

# 2-to-N
add_executable(two_to_n
  src/two_to_n.cpp
)
ament_target_dependencies(two_to_n
  rclcpp
  std_msgs
)
target_link_libraries(two_to_n utils)

# Sink
add_executable(sink
  src/sink.cpp
)
ament_target_dependencies(sink
  rclcpp
  std_msgs
)
target_link_libraries(sink utils)

# Install message link type nodes
install(TARGETS
  source
  one_to_one
  one_to_n
  n_to_m
  two_to_n
  sink
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
