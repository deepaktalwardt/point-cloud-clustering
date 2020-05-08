#include <iostream>
#include <string>

#include "dataset_generation/filter_point_clouds.h"

int main()
{
    std::string in_pcd_file =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/combined_objects_pcd/Hatchback_combined.pcd";

    // std::string out_pcd_file =
    //     "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep_voxelized.pcd";

    dataset_generation::PointCloudFiltering pcf;

    pcf.apply_voxel_filter_and_visualize(in_pcd_file, 0.1, 0.1, 0.1);
    // pcf.apply_voxel_filter_and_save(in_pcd_file, out_pcd_file, 0.05, 0.05, 0.05);
}
