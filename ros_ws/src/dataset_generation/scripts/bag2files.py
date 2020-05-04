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
    $ rosrun pcl_ros pointoud_to_pcd input:=/sync/lidar _prefix:="/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag3_2020-05-01-11-15-05.bag/pcd/" _binary:=false
    """

    def __init__(self, in_pcl_topic, in_detections_3d_topic, out_sync_lidar_topic, out_folder_path):

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
        self._sub_pcl = message_filters.Subscriber(self._in_pcl_topic, PointCloud2)
        self._sub_detections_3d = message_filters.Subscriber(self._in_detections_3d_topic, Detection3DArray)

        ts = message_filters.ApproximateTimeSynchronizer([self._sub_pcl, self._sub_detections_3d], 1, 0.03)

        ts.registerCallback(self.callback)
    
    def callback(self, pcl_msg, dets_3d_msg):

        to_save_as = str(pcl_msg.header.stamp.secs) + str(pcl_msg.header.stamp.nsecs)[:-3]

        if (len(dets_3d_msg.detections) > 0):
            self.save_dets_3d_json(dets_3d_msg, to_save_as)
            self._pcl_pub.publish(pcl_msg)
            self._count += 1
            print("Count: " + str(self._count) + ". Timestamp: " + to_save_as)
        else:
            print("Skipped saving at " + to_save_as + ". No detections found.")
    
    def save_dets_3d_json(self, dets_3d_msg, to_save_as):
        dets_3d_msg_yaml = yaml.load(str(dets_3d_msg))
        dets_3d_msg_json = json.dumps(dets_3d_msg_yaml, indent=4)

        path_to_file = os.path.join(self._out_detections_3d_path, to_save_as + '.json')

        with open(path_to_file, 'w') as file:
            file.write(dets_3d_msg_json)


if __name__ == "__main__":

    in_pcl_topic = '/simulator/lidar'
    in_detections_3d_topic = '/simulator/ground_truth/3d_detections'
    out_sync_lidar_topic = '/sync/lidar'
    
    # bag_name = 'bag1_2020-04-05-09-05-20.bag'
    bag_name = 'bag3_2020-05-01-11-15-05.bag'
    out_folder_path = os.path.join('/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs', bag_name)

    bag_2_files_node = Bag2Files(in_pcl_topic, in_detections_3d_topic, out_sync_lidar_topic, out_folder_path)


