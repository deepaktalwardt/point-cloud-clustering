#pragma once

#include <iostream>
#include <thread>
#include <string>
#include <sstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace std::literals::chrono_literals;

namespace dataset_generation
{


class PointCloudFiltering
{
public:
    PointCloudFiltering(
        std::string in_folder_path,
        std::string out_folder_path);

    ~PointCloudFiltering();

    void apply_voxel_filter_and_save(float leaf_size_x, float leaf_size_y, float leaf_size_z);
    void apply_voxel_filter_and_visualize(float leaf_size_x, float leaf_size_y, float leaf_size_z);
private:
    std::string in_folder_path_;
    std::string out_folder_path_;
};
PointCloudFiltering::PointCloudFiltering(
        std::string in_folder_path,
        std::string out_folder_path)
{
    in_folder_path_=in_folder_path;
    out_folder_path_=out_folder_path;
}
PointCloudFiltering::~PointCloudFiltering()
{}

/*
This method is a modified version of the VoxelGrid filtering found at: 
https://pcl-tutorials.readthedocs.io/en/master/voxel_grid.html?highlight=voxelGrid
*/
void PointCloudFiltering::apply_voxel_filter_and_visualize(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    pcl::PCLPointCloud2::Ptr cloud_read (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
    
    //Read PCD files from the path given
    pcl::PCDReader reader;
    reader.read(in_folder_path_, *cloud_read);

    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud (cloud_read);
    vox.setLeafSize (leaf_size_x,leaf_size_y,leaf_size_z);
    vox.filter (*cloud_voxelized);

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_voxelized, *in_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        in_cloud,
        "sample cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); 

    viewer->setRepresentationToWireframeForAllActors();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

}

void PointCloudFiltering::apply_voxel_filter_and_save(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    pcl::PCLPointCloud2::Ptr cloud_read (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
    
    //Read PCD files from the path given
    pcl::PCDReader reader;
    reader.read(in_folder_path_, *cloud_read);

    std::cerr << "PointCloud before filtering: " << cloud_read->width * cloud_read->height 
       << " data points (" << pcl::getFieldsList (*cloud_read) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud (cloud_read);
    vox.setLeafSize (leaf_size_x,leaf_size_y,leaf_size_z);
    vox.filter (*cloud_voxelized);

    std::cerr << "PointCloud after filtering: " << cloud_voxelized->width * cloud_voxelized->height 
       << " data points (" << pcl::getFieldsList (*cloud_voxelized) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write (out_folder_path_, *cloud_voxelized, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

} // namespace dataset_generation
