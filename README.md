# Robot Navigation Project（MathWorks）



## How to Build and Run:

1. Create a catikn_ws folder and clone the repository.
```console
git clone https://github.com/Christol-Jalen/Project-ANVRT.git
cd Project-ANVRT
catkin_make
```

2. Mount the sensor to the Husky robot and launch the world & Husky

```console
export HUSKY_URDF_EXTRAS=$HOME/Project-ANVRT/realsense.urdf.xacro
roslaunch cpr_office_gazebo office_construction_world.launch platform:=husky
```

3. Launch rviz

```console
roslaunch husky_viz view_robot.launch
```

4. Launch gmapping SLAM algorithm

```console
roslaunch husky_navigation gmapping.launch
```

5. Launch octomap

```console
roslaunch octomap_server octomap_mapping.launch
```


======================================
```console
rosrun rqt_tf_tree rqt_tf_tree
```
