cmake_minimum_required(VERSION 3.0.2)
project(synchronized_ros_subscribers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

find_package(Boost REQUIRED COMPONENTS
signals
)

catkin_package()
include_directories(
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)




catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(subscriber src/subscriber.cpp)
add_dependencies(subscriber ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
