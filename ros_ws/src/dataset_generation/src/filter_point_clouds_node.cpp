#include <iostream>
#include <string>

#include "dataset_generation/process_point_clouds.h"

int main()
{
    // std::string out_pcd_file =
    //     "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep_voxelized.pcd";

    // pcf.apply_voxel_filter_and_save(in_pcd_file, out_pcd_file, 0.05, 0.05, 0.05);
    // std::string in_pcd_file =
    //     "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep.pcd";

    // std::string out_pcd_file =
    //     "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/Jeep_pass_through.pcd";

    std::string in_pcd_file =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/combined_objects_10_pcd/Jeep_10_combined.pcd";
    
    // std::string out_pcd_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/combined_objects_10_pcd/Jeep_10_combined_filtered.pcd";
    
    std::string out_pcd_file =
        "/home/deepak/Downloads/Jeep_10_combined_filtered.pcd";

    std::string out_vtk_file =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/combined_objects_10_pcd/Hatchback_10_combined_filtered.vtk";

    dataset_generation::PointCloudProcessing pcf;

    pcf.apply_voxel_filter_and_visualize(in_pcd_file, 0.05, 0.05, 0.05);
    // pcf.apply_voxel_filter_and_save(in_pcd_file, out_pcd_file, 0.05, 0.05, 0.05);

    // pcf.apply_radial_filter_and_visualize(in_pcd_file, 0.05, 3);
    // pcf.apply_radial_filter_and_save(in_pcd_file, out_pcd_file, 0.1, 7);

    // pcf.apply_ground_removal_and_visualize(in_pcd_file, 0.1, 2);
    // pcf.apply_pass_filter_and_save(in_pcd_file, out_pcd_file,0.15,2);

    // pcp.apply_statistical_outlier_removal_and_save(in_pcd_file, out_pcd_file, 100, 0.5);

    // json mesh_options;
    // mesh_options["k_search"] = 100;
    // mesh_options["search_radius"] = 0.1;
    // mesh_options["mu"] = 10;
    // mesh_options["max_nearest_neighbors"] = 400;
    // mesh_options["max_surface_angle"] = M_PI/4;  // 45 degrees
    // mesh_options["min_surface_angle"] = M_PI/18; // 10 degrees
    // mesh_options["max_angle"] = 2 * M_PI/3;      // 120 degrees
    // mesh_options["normal_consistency"] = false;   

    // pcp.convert_point_cloud_to_mesh(out_pcd_file, out_vtk_file, mesh_options);
}
