#pragma once

#include <iostream>
#include <thread>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::literals::chrono_literals;

namespace dataset_generation
{

class PointCloudFiltering
{
public:
    PointCloudFiltering();

    ~PointCloudFiltering();

    pcl::PCLPointCloud2 apply_voxel_filter(
        pcl::PCLPointCloud2::ConstPtr in_cloud,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);

    void apply_voxel_filter_and_save(
        std::string in_pcd_path,
        std::string out_pcd_path,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);
    
    void apply_voxel_filter_and_visualize(
        std::string in_pcd_path,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_ground(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
        const float& min_range,
        const float& max_range);

    void remove_ground_and_visualize(
        std::string in_pcd_path,
        const float& min_range,
        const float& max_range);

    void remove_ground_and_save(
        std::string in_pcd_path,
        std::string out_pcd_path,
        const float& min_range,
        const float& max_range);
};

/**
 * Default Constructor
*/
PointCloudFiltering::PointCloudFiltering() = default;

/**
 * Default Destructor
*/
PointCloudFiltering::~PointCloudFiltering() = default;

/**
 * Applies the voxel filter to in_cloud and returns the filtered cloud
 * 
 * This is a modified version of the VoxelGrid filtering found at: 
 * https://pcl-tutorials.readthedocs.io/en/master/voxel_grid.html?highlight=voxelGrid
*/
pcl::PCLPointCloud2 PointCloudFiltering::apply_voxel_filter(
    pcl::PCLPointCloud2::ConstPtr in_cloud,
    const float& leaf_size_x,
    const float& leaf_size_y,
    const float& leaf_size_z)
{
    pcl::PCLPointCloud2::Ptr cloud_voxelized(new pcl::PCLPointCloud2());

    std::cerr << "PointCloud before filtering: " << in_cloud->width * in_cloud->height 
       << " data points (" << pcl::getFieldsList(*in_cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(in_cloud);
    vox.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    vox.filter(*cloud_voxelized);

    std::cerr << "PointCloud after filtering: " << cloud_voxelized->width * cloud_voxelized->height 
       << " data points (" << pcl::getFieldsList(*cloud_voxelized) << ")." << std::endl;
    
    return *cloud_voxelized;
}

/**
 * Removes all the points on the ground plane from the input point cloud using the
 * PassThrough filter. Returns the filtered Point Cloud pointer.
 * Points that lie between min_range and max_range along the z-axis are KEPT. Everything
 * else is removed.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFiltering::remove_ground(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
    const float& min_range,
    const float& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    std::cerr << "PointCloud before filtering: " << in_cloud->width * in_cloud->height 
       << " data points (" << pcl::getFieldsList(*in_cloud) << ")." << std::endl;


    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (min_range, max_range);
    pass.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
         << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
        
    return cloud_filtered;
}

/**
 * Applies Voxel Filter and visualizes in PCL Visualizer
*/
void PointCloudFiltering::apply_voxel_filter_and_visualize(
    std::string in_pcd_path,
    const float& leaf_size_x,
    const float& leaf_size_y,
    const float& leaf_size_z)
{
    // Create cloud objects
    pcl::PCLPointCloud2::Ptr cloud_read(new pcl::PCLPointCloud2());
    
    //Read PCD file from the path given
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud_read);

    // Apply the filter
    pcl::PCLPointCloud2 cloud_voxelized = apply_voxel_filter(
        cloud_read,
        leaf_size_x,
        leaf_size_y,
        leaf_size_z);

    // Convert to pcl::PointCloud<> object to visualize
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_voxelized, *cloud_to_visualize);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        cloud_to_visualize,
        "Voxelized Point Cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Voxelized Point Cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); 

    // Display in a loop until stopped
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

/**
 * Applies Voxel Filter and saves the resulting PCD file to out_pcd_path
*/
void PointCloudFiltering::apply_voxel_filter_and_save(
    std::string in_pcd_path,
    std::string out_pcd_path,
    const float& leaf_size_x,
    const float& leaf_size_y,
    const float& leaf_size_z)
{
    // Create cloud objects
    pcl::PCLPointCloud2::Ptr cloud_read(new pcl::PCLPointCloud2());
    
    //Read PCD file from the path given
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud_read);

    // Apply the filter
    pcl::PCLPointCloud2 cloud_voxelized = apply_voxel_filter(
        cloud_read,
        leaf_size_x,
        leaf_size_y,
        leaf_size_z);

    // Write to out_pcd_path
    pcl::PCDWriter writer;
    writer.write(out_pcd_path, cloud_voxelized, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

/**
 * Takes in_pcd_path to PCD file, removes ground and saves it into a PCD file at out_pcd_path
*/
void PointCloudFiltering::remove_ground_and_save(
    std::string in_pcd_path,
    std::string out_pcd_path,
    const float& min_range,
    const float& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = remove_ground(
        cloud,
        min_range,
        max_range);

    pcl::PCDWriter writer;
    writer.write(out_pcd_path, *cloud_filtered);
}

/**
 * Takes in_pcd_path to PCD file, removes ground and visualizes using PCLVisualizer. Use for debugging.
*/
void PointCloudFiltering::remove_ground_and_visualize(
    std::string in_pcd_path,
    const float& min_range,
    const float& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);
    
    // Fill in the cloud data
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = remove_ground(
        cloud,
        min_range,
        max_range);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        cloud_filtered,
        "Pass through Filtered point cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Pass through Filtered point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

} // namespace dataset_generation
