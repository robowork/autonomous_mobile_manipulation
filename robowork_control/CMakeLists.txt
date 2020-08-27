cmake_minimum_required(VERSION 2.8.3)
project(robowork_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS
                        scripts/home_ur5e_SIM
                        scripts/init_ur5e_SIM
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
