cmake_minimum_required(VERSION 2.8.3)
project(robowork_planning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) #-std=c++14 vs -std=gnu++14

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})
# Comment these in if using custom-compiled PCL and segfaulting
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#STRING(REGEX REPLACE "-march=native" " " PCL_DEFINITIONS_STRIPPED ${PCL_DEFINITIONS})
#add_definitions(${PCL_DEFINITIONS_STRIPPED})

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
    image_geometry
    image_transport
    message_filters
    nodelet
    sensor_msgs
    stereo_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    rosbag
    tf
    tf_conversions
    pcl_ros
    pcl_conversions
    roslaunch
    roscpp
    std_srvs
    rosbag
    eigen_conversions
      octomap_msgs
      octomap_ros
      # octomap_world
      # voxblox
      # voxblox_ros
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_perception
        pluginlib
        geometric_shapes
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

generate_dynamic_reconfigure_options(
  # cfg/XXX.cfg
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
    image_geometry
    image_transport
    message_filters
    nodelet
    sensor_msgs
    stereo_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    rosbag
    tf
    tf_conversions
    pcl_ros
    pcl_conversions
    roscpp
    roslaunch
    dynamic_reconfigure
    std_srvs
    rosbag
      octomap_msgs
      octomap_ros
      # octomap_world
      # voxblox
      # voxblox_ros
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
#    ${PROJECT_NAME}
  DEPENDS
    Boost
)

include_directories(include)
include_directories(SYSTEM
                      include
                      ${catkin_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS})

# Library
add_library(MGI_AprilTag_lib src/robowork_planning/MGI_AprilTag.cpp)
target_link_libraries(MGI_AprilTag_lib ${catkin_LIBRARIES}
                                               ${OpenCV_LIBS})

# Standalone node
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node # ${PROJECT_NAME}
                                          MGI_AprilTag_lib)
roslaunch_add_file_check(launch)

# install(TARGETS robowork_base_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS MGI_AprilTag_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
