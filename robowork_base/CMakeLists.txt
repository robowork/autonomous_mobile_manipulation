cmake_minimum_required(VERSION 2.8.3)
project(robowork_base)

find_package(catkin REQUIRED
             COMPONENTS
               controller_manager
               hardware_interface
               diagnostic_updater
               roslaunch
               roscpp
               sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS
#    include
  LIBRARIES
  CATKIN_DEPENDS
    diagnostic_updater
    hardware_interface
    roscpp
    sensor_msgs
  DEPENDS Boost
)

include_directories(#include 
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
