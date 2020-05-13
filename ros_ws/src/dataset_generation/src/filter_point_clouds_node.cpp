#include <iostream>
#include <string>

#include "dataset_generation/process_point_clouds.h"

/**
 * This node is used for processing/filtering point cloud objects.
 * 
 * All the code below is commented, please uncomment and appropriately change file paths to run this code.
*/

int main()
{
    /******************************************************************************
     * Uncomment to create PointCloudProcessing Object
     * ***************************************************************************/
    // std::string in_pcd_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag6_2020-05-09-19-08-22.bag/combined_objects_10/Jeep_combined.pcd";
    
    // std::string out_pcd_file =
    //     "/home/deepak/Downloads/Jeep_combined_filtered.pcd";

    // std::string out_vtk_file =
    //     "/home/deepak/Downloads/Hatchback_mesh_combined_filtered.vtk";

    // dataset_generation::PointCloudProcessing pcp;

    /**
     * Uncomment to Apply Ground Removal
    */
    // pcp.apply_ground_removal_and_visualize(in_pcd_file, 0.13, 2);
    // pcp.apply_ground_removal_and_save(in_pcd_file, out_pcd_file, 0.13, 2);

    /**
     * Uncomment to Apply Statistical Outlier Removal
    */
    // pcp.apply_statistical_outlier_removal_and_visualize(out_pcd_file, 5, 0.3);
    // pcp.apply_statistical_outlier_removal_and_save(out_pcd_file, out_pcd_file, 5, 0.3);

    /**
     * Uncomment to apply Voxel Filtering
    */
    // pcp.apply_voxel_filter_and_visualize(out_pcd_file, 0.1, 0.1, 0.1);
    // pcp.apply_voxel_filter_and_save(out_pcd_file, out_pcd_file, 0.1, 0.1, 0.1);

    // pcf.apply_voxel_filter_and_visualize(in_pcd_file, 0.05, 0.05, 0.05);
    // pcf.apply_voxel_filter_and_save(in_pcd_file, out_pcd_file, 0.05, 0.05, 0.05);

    /**
     * Uncomment to apply Radial Filtering
    */
    // pcf.apply_radial_filter_and_visualize(in_pcd_file, 0.05, 3);
    // pcf.apply_radial_filter_and_save(in_pcd_file, out_pcd_file, 0.1, 7);


    /********************************************************************
     * Uncomment to save PCDs to Mesh
     * *****************************************************************/
    // std::string in_pcd_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/sources/sources_1/Hatchback_combined_filtered.pcd";

    // std::string out_vtk_file =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/sources/sources_1/Hatchback_combined_mesh.vtk";


    // json mesh_options;
    // mesh_options["k_search"] = 100;
    // mesh_options["search_radius"] = 1.5;
    // mesh_options["mu"] = 10;
    // mesh_options["max_nearest_neighbors"] = 400;
    // mesh_options["max_surface_angle"] = M_PI/4;  // 45 degrees
    // mesh_options["min_surface_angle"] = M_PI/18; // 10 degrees
    // mesh_options["max_angle"] = 2 * M_PI/3;      // 120 degrees
    // mesh_options["normal_consistency"] = false;   

    // pcp.convert_point_cloud_to_mesh(in_pcd_file, out_vtk_file, mesh_options);

    return 0;
}
