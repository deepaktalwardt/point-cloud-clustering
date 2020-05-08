#include <iostream>
#include <string>
#include "dataset_generation/filter_point_clouds.h"

int main()
{
    std::string in_folder_pcd =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep.pcd";
    std::string out_folder_pcd =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep_voxelized,pcd";    
    dataset_generation::PointCloudFiltering pcf(in_folder_pcd,out_folder_pcd);
    pcf.apply_voxel_filter_and_visualize(0.05,0.05,0.05);
    pcf.apply_voxel_filter_and_save(0.05,0.05,0.05);
}
