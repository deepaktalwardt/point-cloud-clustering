#include <iostream>
#include <string>
#include "dataset_generation/extract_point_cloud_objects.h"

#include "dataset_generation/json.hpp"

using json = dataset_generation::json;

/**
 * Visualizes objects in PCD file, loading PCD and JSON directly from file. 
 * Use this for debugging!
*/
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
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/pcd";
    
    std::string in_folder_dets3d =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/detections_3d";

    std::string out_folder_pcd =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/out_100_transformed";

    dataset_generation::ExtractPointCloudObjects epco(
        in_folder_pcd,
        in_folder_dets3d,
        out_folder_pcd);
    
    int min_nb_points_threshold = 100;

    epco.extract_objects_from_all_pcds(min_nb_points_threshold);

    return 0;
}