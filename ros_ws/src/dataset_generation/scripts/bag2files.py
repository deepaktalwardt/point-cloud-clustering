#! /usr/bin/env python
import os
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import message_filters
from sensor_msgs.msg import PointCloud2
from lgsvl_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from datetime import datetime

import json
import yaml

class Bag2Files:

    """
    Make sure to run this before in a separate terminal
    $ rosrun pcl_ros pointcloud_to_pcd input:=/sync/lidar _prefix:="/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/pcd/" _binary:=false
    """

    def __init__(self, in_pcl_topic, in_detections_3d_topic, out_sync_lidar_topic, out_folder_path):
        """
        Constructor for the Bag2Files class
        """
        rospy.init_node("bag2files")

        self._in_pcl_topic = in_pcl_topic
        self._in_detections_3d_topic = in_detections_3d_topic
        self._out_sync_lidar_topic = out_sync_lidar_topic
        self._out_folder_path = out_folder_path

        self._out_pcd_path = os.path.join(self._out_folder_path, "pcd")
        self._out_detections_3d_path = os.path.join(self._out_folder_path, "detections_3d")

        if not self.create_output_folders():
            print("Fix folder names and retry")
            return

        print("Saving files to: ")
        print(self._out_pcd_path)
        print(self._out_detections_3d_path)

        self._pcl_pub = rospy.Publisher(out_sync_lidar_topic, PointCloud2, queue_size=5)

        self._count = 0

        print('Bag2Files is running, now run the bag!')

        self.listener()
        rospy.spin()
    
    def create_output_folders(self):
        """
        This function creates output folders if not already created
        """
        try:
            if not os.path.exists(self._out_pcd_path):
                os.makedirs(self._out_pcd_path)
            
            if not os.path.exists(self._out_detections_3d_path):
                os.makedirs(self._out_detections_3d_path)
            return True
        except OSError as e:
            print(e)
            return False
    

    def listener(self):
        """
        This listener function initializes the subscribers and sets up the time synchronizer object
        """
        self._sub_pcl = message_filters.Subscriber(self._in_pcl_topic, PointCloud2)
        self._sub_detections_3d = message_filters.Subscriber(self._in_detections_3d_topic, Detection3DArray)

        # Time Synchronizer to synchronize detections and Point Clouds to within 0.01 seconds
        ts = message_filters.ApproximateTimeSynchronizer([self._sub_pcl, self._sub_detections_3d], 1, 0.01)

        # Call the callback on synchronized messages
        ts.registerCallback(self.callback)
    
    
    def callback(self, pcl_msg, dets_3d_msg):
        """
        This callback method takes in pcl_msg and dets_3d_msg and saves them appropriately
        """
        to_save_as = str(pcl_msg.header.stamp.secs) + str(pcl_msg.header.stamp.nsecs)[:-3]

        if (len(dets_3d_msg.detections) > 0):
            self.save_dets_3d_json(dets_3d_msg, to_save_as)
            self._pcl_pub.publish(pcl_msg)
            self._count += 1
            print("Count: " + str(self._count) + ". Timestamp: " + to_save_as)
        else:
            print("Skipped saving at " + to_save_as + ". No detections found.")
    

    def save_dets_3d_json(self, dets_3d_msg, to_save_as):
        """
        This method is called by the callback and is used to save detections as JSON files to disk
        """
        dets_3d_msg_yaml = yaml.load(str(dets_3d_msg))
        dets_3d_msg_json = json.dumps(dets_3d_msg_yaml, indent=4)

        path_to_file = os.path.join(self._out_detections_3d_path, to_save_as + '.json')

        with open(path_to_file, 'w') as file:
            file.write(dets_3d_msg_json)


if __name__ == "__main__":

    # Define the topics to subscribe to
    in_pcl_topic = '/simulator/lidar'
    in_detections_3d_topic = '/simulator/ground_truth/3d_detections'

    # Define the topic to publish synchronized LiDAR point clouds to
    out_sync_lidar_topic = '/sync/lidar'
    
    # To be used as the folder name
    # bag_name = 'bag5_2020-05-05-15-11-17.bag'
    # bag_name = 'bag6_2020-05-09-19-08-22.bag'
    bag_name = 'testbag1_2020-05-09-19-39-52.bag'
    
    # Path where to save outputs from the bag
    out_folder_path = os.path.join('/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs', bag_name)

    # Create object
    bag_2_files_node = Bag2Files(in_pcl_topic, in_detections_3d_topic, out_sync_lidar_topic, out_folder_path)


