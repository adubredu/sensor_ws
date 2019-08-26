cmake_minimum_required(VERSION 2.8.3)
project(rplidar_launch)
set(eigen_INCLUDE_DIR /usr/include/eigen3/Eigen)
set(Eigen3_DIR /usr/lib/cmake/eigen3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  pcl_ros
  rospy
  Eigen3
  laser_geometry
  std_msgs
  tf
)
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${eigen_INCLUDE_DIR} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## System dependencies are found with CMake's conventions

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rplidar_launch
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
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


# catkin_add_nosetests(test)
add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
add_executable(stop_when_too_close src/stop_when_too_close.cpp)


target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES} )


target_link_libraries(stop_when_too_close ${catkin_LIBRARIES} )
