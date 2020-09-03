# autonomous_mobile_manipulation

### Pull resources
```
# Setup and pull
mkdir -p $HOME/autonomous_mobile_manipulation_ws/src
cd $HOME/autonomous_mobile_manipulation_ws/src
git clone https://github.com/robowork/autonomous_mobile_manipulation
cd autonomous_mobile_manipulation
git submodule update --init --recursive

# Copy gazebo models
cp -r gazebo_resources/models/* $HOME/.gazebo/models/
```

### Clone required libraries and install them in devel space
```
# GTSAM
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/borglab/gtsam && cd gtsam && git checkout 4.0.2 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# g2o
cd $HOME/autonomous_mobile_manipulation_ws &&  git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && git checkout 20200410_git && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# Libnabo
cd $HOME/autonomous_mobile_manipulation_ws && git clone git://github.com/ethz-asl/libnabo.git && cd libnabo && git checkout 1.0.7 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# Libpointmatcher
cd $HOME/autonomous_mobile_manipulation_ws && git clone git://github.com/ethz-asl/libpointmatcher.git && cd libpointmatcher && git checkout 1.3.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# RTABMAP
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/introlab/rtabmap && cd rtabmap && git checkout 0.20.3-melodic && cd build
CMAKE_PREFIX_PATH=$HOME/autonomous_mobile_manipulation_ws/devel:$CMAKE_PREFIX_PATH cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install 

# FCL
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/flexible-collision-library/fcl && cd fcl && git checkout 0.6.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install 
```

### Build
```
cd $HOME/autonomous_mobile_manipulation_ws/src

catkin config -DCMAKE_BUILD_TYPE=Release -DOMPL_REGISTRATION=OFF 

# Builds have to be informed that above built libraries are installed in devel space and should look in there first:

# Set environment variable each command:
CMAKE_PREFIX_PATH=$HOME/autonomous_mobile_manipulation_ws/devel:$CMAKE_PREFIX_PATH catkin build

# Or export for entire shell session:
export CMAKE_PREFIX_PATH=$HOME/autonomous_mobile_manipulation_ws/devel:$CMAKE_PREFIX_PATH
catkin build 
```

### Note:
In case you run ```catkin clean```, this will clear out the devel space where the above libraries are installed. You have to reinstall them with ``make install``` (no need to rebuild): 
```
cd $HOME/autonomous_mobile_manipulation_ws/gtsam/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/g2o/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/libnabo/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/libpointmatcher/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/rtabmap/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/fcl/build && make install
```

### Simulation using gazebo

```
# Terminal 1 - Launch Simulation 
roslaunch robowork_gazebo bvr_SIM_playpen.launch

# Terminal 2 - Launch MoveIt! move_group planning
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_moveit_config robowork_moveit_planning_execution.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM

# Terminal 3 - Visualization (If not already running for the real robot)
roslaunch robowork_moveit_config moveit_rviz.launch

# Terminal 4 - Launch robowork_planning
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_planning move_group_interface_vAprilTag.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM

# Press 'Next' on Rviz to trigger planning to reach AprilTag
```
