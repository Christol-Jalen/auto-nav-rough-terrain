# Robot source files

## Overview
This folder contains all the source files for the husky robot. The following tutorial teaches how to map the gazebo world.


## How to Build and Run:

1. Clone the repository and build.
```console
git clone https://github.com/Christol-Jalen/auto-nav-rough-terrain.git
cd auto-nav-rough-terrain
catkin_make
```

2. Mount the sensor to the Husky robot and launch the world & Husky

```console
export HUSKY_URDF_EXTRAS=$HOME/auto-nav-rough-terrain/realsense.urdf.xacro
source devel/setup.bash
roslaunch cpr_office_gazebo office_construction_world.launch platform:=husky
```

3. Launch rviz

```console
source devel/setup.bash
roslaunch husky_viz view_robot.launch
```

4. Launch Gmapping

```console
source devel/setup.bash
roslaunch husky_navigation gmapping.launch
```

5. Launch Octomap

```console
source devel/setup.bash
roslaunch octomap_server octomap_mapping.launch
```

6. Move the robot around and save the maps

```console
rosrun octomap_server octomap_saver octomap_name.bt
rosrun map_server map_saver -f occupancy_name
```

## Useful tools

1. Observing the tf tree
```console
rosrun rqt_tf_tree rqt_tf_tree
```
