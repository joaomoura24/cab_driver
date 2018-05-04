cmake_minimum_required(VERSION 2.8.3)

project(cab_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  std_msgs
  #urdf
  kdl_parser # it apparently contains the eigen package
  roslib
  serial_arm_lib
  roscpp
  serial
  #orocos_kdl
)


#find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES serial_arm_lib
  #CATKIN_DEPENDS roscpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Add executable target motors_driver:
add_executable(motors_driver src/motors_driver.cpp)
target_link_libraries(motors_driver ${catkin_LIBRARIES})
