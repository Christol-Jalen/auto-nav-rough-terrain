# Robot Navigation Project（MathWorks）



## How to Build and Run:

1. Download the construction office world and cpr accessories from this [link](https://github.com/clearpathrobotics/cpr_gazebo/tree/noetic-devell) and move the two folders to catkin_ws/src

2. Download the Husky robot source code from this [link](https://github.com/husky/husky) and move all folders to catkin_ws/src


3. Create a customized URDF file according to this [link](https://www.clearpathrobotics.com/assets/guides/kinetic/husky/additional_sim_worlds.html) and place the URDF file at $HOME/Desktop/realsense.urdf.xacro


4. Build

```console
$ catkin_make
```

5. Mount the sensor to the Husky robot and launch the world & Husky

```console
$ export HUSKY_URDF_EXTRAS=$HOME/Desktop/realsense.urdf.xacro
$ roslaunch cpr_office_gazebo office_construction_world.launch platform:=husky
```

6. Launch rviz

```console
$ export HUSKY_URDF_EXTRAS=$HOME/Desktop/realsense.urdf.xacro
$ roslaunch husky_viz view_robot.launch
```

7. Launch husky_navigation

```console
$ roslaunch husky_navigation gmapping_demo.launch
```

8. Launch octomap

```console
$ roslaunch octomap_server octomap_mapping.launch
```
