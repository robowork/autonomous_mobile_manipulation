# facet_viewpoints_extractor
## Dependencies
- numpy
- matplotlib
- collada
- open3d
- csv

Use pip3 to install all the above dependencies

Note: this script needs python3 to run. To install packages for current user
append the command with prefix "sudo -H"
```
pip3 install numpy open3d python-csv pycollada matplotlib 
```

## Usage
Use the following command to extract face normals and centers from dae mesh file
and save it to a csv file. 
```
python3 facet_viewpoints_extractor.py -m boat.dae -c boat.csv
```
or
```
python3 facet_viewpoints_extractor.py --mesh boat.dae --csv_file boat.csv
```

# visualize_viewoiubts_from_csv
## Dependencies
- numpy
- rospy
- geometry_msgs
- visualization_msgs

This file can be run using python2 for ROS Melodic and python3 for ROS Noetic
and beyond
Use pip3 to install some of the above dependencies

```
pip3 install numpy python-csv 
```
For installing geometry_msgs and visualization_msgs use the following command:

Note: Replace melodic with your ROS distro
```
sudo apt install ros-melodic-geometry-msgs ros-melodic-visualization-msgs
```

## Usage
Use the following command to read centers and normals from a csv file and publish
them as PoseArray to visualize on RViz.
```
python visualize_viewpoints_from_csv.py -c boat.csv
```
or
```
python visualize_viewpoints_from_csv.py --csv_file boat.csv
```