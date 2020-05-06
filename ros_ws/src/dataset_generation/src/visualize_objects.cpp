#include <iostream>
#include <string>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "dataset_generation/extract_point_cloud_objects.h"

int main(int argc, char** argv)
{
    // These folders aren't really useful for visualizing, but needed
    std::string in_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/pcd";
    
    std::string in_folder_dets3d =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/detections_3d";

    std::string out_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_100_transformed";

    dataset_generation::ExtractPointCloudObjects epco(
        in_folder_pcd,
        in_folder_dets3d,
        out_folder_pcd);

    // Code below is needed to concatenate all "Jeep"s into one cloud
    std::string objects_pcd_folder =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_100_transformed";

    epco.concatenate_objects_and_visualize(objects_pcd_folder, "Jeep");

    return 0;
}