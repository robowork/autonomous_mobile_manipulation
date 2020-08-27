cmake_minimum_required(VERSION 2.8.3)
project(robowork_description)

find_package(catkin REQUIRED
#   COMPONENTS roslaunch
)

catkin_package()

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
