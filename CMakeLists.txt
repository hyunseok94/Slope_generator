cmake_minimum_required(VERSION 2.8.3)
project(slope_generator_pkgs)

FIND_PACKAGE (Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
   LIBRARIES slope_generator_pkgs
   DEPENDS gazebo_ros roscpp rospy std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
