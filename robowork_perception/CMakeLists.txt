cmake_minimum_required(VERSION 2.8.3)
project(robowork_perception)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) #-std=c++11 vs -std=gnu++11

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
#    image_proc
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
    # octomap_msgs
    # octomap_ros
    # octomap_world
    # voxblox
    # voxblox_ros
    apriltag_ros
)

find_package(Boost REQUIRED)

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
#    image_proc
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
    #octomap_msgs
    #octomap_ros
    #octomap_world
    #voxblox
    #voxblox_ros
    apriltag_ros
  INCLUDE_DIRS
#    include
  LIBRARIES
#    ${PROJECT_NAME}
    fisheye_camera_model_lib
  DEPENDS
    Boost
)

#include_directories(include)
include_directories(SYSTEM
                      #include
                      ${catkin_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS})

roslaunch_add_file_check(launch)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
