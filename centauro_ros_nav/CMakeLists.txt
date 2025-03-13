cmake_minimum_required(VERSION 3.8)
project(centauro_ros_nav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(centauro_ros_nav_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  centauro_ros_nav_srvs
  tf2
  tf2_ros
  tf2_eigen
)

add_library(valid_target_selector_manager SHARED
  src/valid_target_selector_manager.cpp
)

ament_target_dependencies(valid_target_selector_manager
  ${dependencies}
)

install(TARGETS valid_target_selector_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(valid_target_selector_node src/valid_target_selector_node.cpp)
target_link_libraries(valid_target_selector_node valid_target_selector_manager)
ament_target_dependencies(valid_target_selector_node ${dependencies})

install(TARGETS
  valid_target_selector_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()