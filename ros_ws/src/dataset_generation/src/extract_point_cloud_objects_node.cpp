#include <iostream>
#include <string>
#include "dataset_generation/extract_point_cloud_objects.h"

#include "dataset_generation/json.hpp"

using json = dataset_generation::json;

int main(int argc, char** argv)
{
    std::string in_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/pcd"
    
    std::string in_folder_dets3d =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/dets3d"

    std::string out_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/out_pcd"

    dataset_generation::ExtractPointCloudObjects epco("", "", "");
    
    return 0;
}