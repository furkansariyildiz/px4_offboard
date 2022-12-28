cmake_minimum_required(VERSION 3.0.2)
project(px4_offboard)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES px4_offboard
 CATKIN_DEPENDS roscpp rospy
)


include_directories(
  INCLUDE_DIRS include src 
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

add_executable(px4_offboard src/offboard.cpp)
target_link_libraries(px4_offboard ${catkin_LIBRARIES})

