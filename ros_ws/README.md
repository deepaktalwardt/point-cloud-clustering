# ROS Workspace

First, install ROS for the version of Ubuntu you have following the instructions here: http://wiki.ros.org/ROS/Installation. Please install the full desktop version. Then, build and test the installation.

### Building
```
cd ros_ws
catkin_make
```

After the build is successful, you will see two new folders `build` and `devel` in the workspace. Before we can use ROS packages, we need to source the built variables.

```
source devel/setup.bash
```

### Testing
To test, we will try playing a recorded rosbag. Download a recorded rosbag from the Google Shared Drive, let's say the name of this bag is `bag1.bag`. Then, to play,

```
rosbag play bag1.bag
```
#### Optional Parameters
`-r <rate>`: Speed at which the bag is played.

`-l`: Loop the bag

For more information see the documentation: http://wiki.ros.org/rosbag/Commandline

### Visualization in RViz
First, in a different terminal open RViz using the following command
```
rviz rviz
```

1. Inside the RViz window, click the "Add" button in the bottom left and select "PointCloud2" message type. 
2. Under the "PointCloud2" section, select Topic: "/simulator/lidar"
3. Under "Global Options" set "Fixed Frame" to "velodyne"
4. This should start showing the point clouds on the display. You might want to tweak the "Size(m)" parameter under the "PointCloud2" section to make them more visible. 

## Extracting Objects from PointClouds
This functionality is provided in the `dataset_generation` package in the `extract_point_cloud_objects.h` file. To build the catkin workspace by running `catkin_make` as shown above. Then, source the workspace by `source devel/setup.bash`. 

The library can be tested by running the `extract_point_cloud_objects_node.cpp` using the following command
```
rosrun dataset_generation extract_point_cloud_objects_node
```

This will output the extract PCD files in the `dataset_generation/test/out_pcd` folder. You can view them using the `pcl_viewer` tool.