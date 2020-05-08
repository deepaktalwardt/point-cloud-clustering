#include <iostream>
#include <string>
#include "dataset_generation/filter_point_clouds.h"

int main()
{
    std::string in_folder_pcd =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/";
    std::string out_folder_pcd =
        "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/";    
    dataset_generation::PCDFiltering pcdf(in_folder_pcd,"Jeep",out_folder_pcd);
    pcdf.VoxelFilter(0.05,0.05,0.05);
}