# point-cloud-clustering
This repository contains code for Point Cloud Clustering project for CMPE 255

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
