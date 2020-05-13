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
To create models from extracted object clouds, you need to concatenate them.
1. Go to `ros_ws/src/dataset_generation/src/visualize_objects.cpp` and modify the input and output paths.
2. Select the object class for which you would like to create the model for. Choose to visualize or visualize and save.
3. Then, run
```
$ catkin_make && source devel/setup.bash && rosrun dataset_generation visualize_objects
```
4. This will visualize/save the model that you created.

Please see the `ExtractPointCloudObjects` class that is available in `ros_ws/src/dataset_generation/include/dataset_generation/extract_point_cloud_objects.h` for the implementation of how this creation works.

### Filter/Preprocess model PCDs
To filter and clean the created model, you will need to remove the ground plane, remove outliers and subsample.
1. Go to `ros_ws/src/dataset_generation/src/filter_point_clouds_node.cpp` and modify the input and output paths and uncomment what you wish to do.
2. Then, run
```
$ catkin_make && source devel/setup.bash && rosrun dataset_generation filter_point_clouds_node
```
3. This will visualize/save the model after preprocessing steps that were chosen.

Please see the `PointCloudProcessing` class that is available in `ros_ws/src/dataset_generation/include/dataset_generation/process_point_clouds.h` for the implementation of how this creation works. This also allows for saving the clouds into a mesh.

## Point Cloud Matching based Classification Algorithms
With the dataset created, you can now run the Point Cloud matching based algorithms.
1. Go to `ros_ws/src/dataset_generation/src/testing_node.cpp` and modify the input/output paths, the classes to use, ICP options and the dataset(s) to use.
2. Then, run
```
$ catkin_make && source devel/setup.bash && rosrun dataset_generation testing_node
```
3. This will save the outputs into JSON files at the specified paths. These files can then be used to generate figures, predictions and plots.

### Analyzing JSON output
1. `analysis/prediction_analysis.ipynb`: This python notebook is for calculating confusion matrix, recall and precision for very object in a one JSON file. You may replace the provided JSON file with the one you created.
  
By default, it loads a provided JSON file which is the output of running ICP on a testset that has atleast 10 points in each object. 

Please be sure to install the following packages before running -
python-pcl 0.3.0a1  
pandas 1.0.3    
numpy 1.18.4  
seaborn 0.10.1  
sklearn 0.22

Getting error in running file? Use Python 3.6+ and update sklearn. 

2. `visualize_histogram_for_object_clouds.py`: This python file is to create histograms of number of points per PCD for each class. You will need to download a dataset of PCD files from Google drive to run this.

Please be sure to install the following packages before running -
python-pcl 0.3.0a1  
Matplotlib 3.2.1

3. `knn_classifier.ipynb`: This python notebook implements the K-Nearest Neighbor Algorithm for classifying point clouds. To run this file you will need to download the dataset of PCD files from Google Drive.

Please be sure to install the following packages before running -
python-pcl 0.3.0a1  
Matplotlib 3.2.1
