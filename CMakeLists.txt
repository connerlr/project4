cmake_minimum_required(VERSION 2.8.3)
project(project4)

find_package(PkgConfig REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs 
  message_filters
  sensor_msgs
  laser_geometry
)


catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs
  sensor_msgs
) 
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(project4  src/project4.cpp)
target_link_libraries(project4 ${catkin_LIBRARIES})
target_link_libraries(project4 ${OpenCV_LIBS} ${BULLET_LIBRARIES})
add_dependencies(project4 ${PROJECT_NAME}_generate_messages_cpp)
