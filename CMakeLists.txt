cmake_minimum_required(VERSION 3.0)
project(kyon_ros_nav)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf tf_conversions  nav_core
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  )


install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
