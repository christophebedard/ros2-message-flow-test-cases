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

# Sync 1-to-1
add_executable(sync_one_to_one
  src/sync_one_to_one.cpp
)
ament_target_dependencies(sync_one_to_one
  rclcpp
  std_msgs
)
target_link_libraries(sync_one_to_one utils)

# Sync 1-to-N
add_executable(sync_one_to_n
  src/sync_one_to_n.cpp
)
ament_target_dependencies(sync_one_to_n
  rclcpp
  std_msgs
)
target_link_libraries(sync_one_to_n utils)

# Periodic async N-to-M
add_executable(periodic_async_n_to_m
  src/periodic_async_n_to_m.cpp
)
ament_target_dependencies(periodic_async_n_to_m
  rclcpp
  std_msgs
)
target_link_libraries(periodic_async_n_to_m utils)

# Partially synchronized N-to-M
add_executable(partial_sync_n_to_m
  src/partial_sync_n_to_m.cpp
)
ament_target_dependencies(partial_sync_n_to_m
  rclcpp
  std_msgs
)
target_link_libraries(partial_sync_n_to_m utils)

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
  sync_one_to_one
  sync_one_to_n
  periodic_async_n_to_m
  partial_sync_n_to_m
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
