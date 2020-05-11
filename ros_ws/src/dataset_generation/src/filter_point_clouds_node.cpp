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

    // std::string in_pcd_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag6_2020-05-09-19-08-22.bag/combined_objects_10/Sedan_combined.pcd";
    
    // std::string out_pcd_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag6_2020-05-09-19-08-22.bag/combined_objects_10/Sedan_combined_filtered.pcd";
    
    // std::string out_vtk_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/combined_objects_10_pcd/Hatchback_10_combined_filtered.vtk";

    dataset_generation::PointCloudProcessing pcp;

    //  pcp.apply_ground_removal_and_visualize(in_pcd_file, 0.1, 2);
    // pcp.apply_ground_removal_and_save(in_pcd_file, out_pcd_file, 0.1, 2);

    // pcp.apply_statistical_outlier_removal_and_visualize(out_pcd_file, 5, 0.3);
    // pcp.apply_statistical_outlier_removal_and_save(out_pcd_file, out_pcd_file, 5, 0.3);

    // pcp.apply_voxel_filter_and_visualize(out_pcd_file, 0.1, 0.1, 0.1);
    // pcp.apply_voxel_filter_and_save(out_pcd_file, out_pcd_file, 0.1, 0.1, 0.1);

    // pcf.apply_voxel_filter_and_visualize(in_pcd_file, 0.05, 0.05, 0.05);
    // pcf.apply_voxel_filter_and_save(in_pcd_file, out_pcd_file, 0.05, 0.05, 0.05);

    // pcf.apply_radial_filter_and_visualize(in_pcd_file, 0.05, 3);
    // pcf.apply_radial_filter_and_save(in_pcd_file, out_pcd_file, 0.1, 7);

    // pcp.apply_ground_removal_and_save(in_pcd_file, out_pcd_file, 0.1, 2);
    // pcf.apply_pass_filter_and_save(in_pcd_file, out_pcd_file,0.15,2);

    // pcp.apply_statistical_outlier_removal_and_visualize(in_pcd_file, 10, 0.5);

    std::string in_pcd_file =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/sources/sources_1/Hatchback_combined_filtered.pcd";

    std::string out_vtk_file =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/sources/sources_1/Hatchback_combined_mesh.vtk";


    json mesh_options;
    mesh_options["k_search"] = 100;
    mesh_options["search_radius"] = 0.5;
    mesh_options["mu"] = 1000;
    mesh_options["max_nearest_neighbors"] = 400;
    mesh_options["max_surface_angle"] = M_PI/1;  // 45 degrees
    mesh_options["min_surface_angle"] = M_PI/18; // 10 degrees
    mesh_options["max_angle"] = 3 * M_PI/3;      // 120 degrees
    mesh_options["normal_consistency"] = false;   

    pcp.convert_point_cloud_to_mesh(in_pcd_file, out_vtk_file, mesh_options);
}
