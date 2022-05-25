cmake_minimum_required(VERSION 3.0.2)
project(rs2pcl)
 
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)
 
 
catkin_package(
)
 
###########
## Build ##
###########
 
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(rs2pcl src/rs2pcl.cpp)
 
 
# PCL part?
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(rs2pcl ${PCL_LIBRARIES})
 
 
target_link_libraries(rs2pcl ${catkin_LIBRARIES})