# Granular classification of 3D Point Cloud objects in the context of Autonomous Driving
This repository contains code for the final project for CMPE 255: Data Mining Spring 2020 course.

# Instructions
## Dataset Collection and Creation
This process is supported only on Ubuntu 18.04 systems that can run Robot Operating System (ROS). 
### Prerequisites
1. Ubuntu 18.04
2. ROS Melodic installation: Follow the installation guide [here](http://wiki.ros.org/ROS/Installation) for the full desktop version.

### Building ROS Workspace
ROS workspace for this project is provided in the `ros_ws` folder. All of the ROS and C++ code lives in this workspace.
```
$ cd ros_ws
$ catkin_make && source devel/setup.bash
```
This will make all the ROS nodes and packages available for you to run.

### Extraction of point clouds and detections from rosbags
1. Download a ROS bag from here: https://drive.google.com/drive/u/1/folders/1o9Vo7HnAgxxuFuwrWheBpOjyCxaTnM-3 (Prof. Rojas has access to our shared drive).
2. In a separate terminal, run `roscore`. This is the ROS Master node.
3. Go to `ros_ws/src/dataset_generation/scripts/bag2files.py` and modify the paths where you would like to save the files to. Also, read the instructions to run the `pointcloud_to_pcd` node in a separate terminal. Please be sure to edit the value of the `_prefix` tag.
4. In another terminal, play the rosbag with `rosbag play <name-of-bag>`. This should begin to extract data from the bag and saving files into the path you specified in bag2files.py file.

### Extract object point clouds from raw PCDs
Once you have extracted the PCDs, you can now extract object point clouds.
1. To extract object clouds, go to `ros_ws/src/dataset_generation/src/extract_point_cloud_objects_node.cpp` and modify the input and output paths. 
2. Then, run 
```
$ catkin_make && source devel/setup.bash && rosrun dataset_generation extract_point_cloud_objects_node
```
3. Extracted objects will now be saved in the output folder specified.
Please see the `ExtractPointCloudObjects` class that is available in `ros_ws/src/dataset_generation/include/dataset_generation/extract_point_cloud_objects.h` for the implementation of how this extraction works.

### Create model PCDs

## If you wish to create the dataset from a rosbag

## If you wish 

## Prediction_analysis.ipynb
This python notebook is for calculating confusion matrix,heatmap,recall and precision for very object in a one json file.  
Giving json file which has atleast 10 points in each object.

### Dependencies for Prediction_analysis.ipynb
python-pcl 0.3.0a1  
pandas 1.0.3    
numpy 1.18.4  
seaborn 0.10.1  
sklearn 0.22

### To Run Prediction_analysis.ipynb
Add path and download test_result folder.   
Getting error in running file, run in pycharm or update sklearn. 


## Visualize_histogram_for_object_clouds.py
This python file is to visualize all points in same object PCDs. 

### Dependencies for Visualize_histogram_for_object_clouds.py
python-pcl 0.3.0a1  
Matplotlib 3.2.1

### To Run Visualize_histogram_for_object_clouds.py
Download out_10_transformed folder and add path.
