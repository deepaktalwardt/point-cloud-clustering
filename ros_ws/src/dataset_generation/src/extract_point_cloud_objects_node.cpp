#include <iostream>
#include <string>
#include "dataset_generation/extract_point_cloud_objects.h"

#include "dataset_generation/json.hpp"

using json = dataset_generation::json;

void visualize_pcd_and_objects(
    dataset_generation::ExtractPointCloudObjects& epco,
    std::string in_pcd_path,
    std::string in_json_path)
{
    pcl::PCLPointCloud2 in_cloud_blob;
    pcl::io::loadPCDFile(in_pcd_path, in_cloud_blob);

    std::ifstream dets3d_file(in_json_path);
    json dets3d_json;
    dets3d_file >> dets3d_json;

    epco.visualize_objects_in_pcd(in_cloud_blob, dets3d_json);
}

int main(int argc, char** argv)
{
    std::string in_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/pcd";
    
    std::string in_folder_dets3d =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/dets3d";

    std::string out_folder_pcd =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/out_pcd";

    dataset_generation::ExtractPointCloudObjects epco(
        in_folder_pcd,
        in_folder_dets3d,
        out_folder_pcd);
    
    // epco.extract_objects_from_all_pcds();

    std::string in_pcd_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/pcd/1586102741930765.pcd";
    
    std::string in_json_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/point-cloud-clustering/ros_ws/src/dataset_generation/test/dets3d/1586102741930765.json";

    visualize_pcd_and_objects(epco, in_pcd_path, in_json_path);

    return 0;
}