#pragma once

#include <iostream>
#include <thread>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::literals::chrono_literals;

namespace dataset_generation
{

class PointCloudProcessing
{
public:
    PointCloudProcessing();

    ~PointCloudProcessing();

    // VoxelGrid Filtering for subsampling
    pcl::PCLPointCloud2::Ptr apply_voxel_filter(
        pcl::PCLPointCloud2::ConstPtr in_cloud,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);

    pcl::PCLPointCloud2::Ptr apply_voxel_filter_and_save(
        const std::string& in_pcd_path,
        const std::string& out_pcd_path,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);
    
    pcl::PCLPointCloud2::Ptr apply_voxel_filter_and_visualize(
        const std::string& in_pcd_path,
        const float& leaf_size_x,
        const float& leaf_size_y,
        const float& leaf_size_z);
    
    // RadialFiltering for removing outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_radial_filter(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
        const float& radius,
        const float& min_nb_neighbors);

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_radial_filter_and_visualize(
        const std::string& in_pcd_path,
        const float& radius,
        const float& min_nb_neighbors);

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_radial_filter_and_save(
        const std::string& in_pcd_path,
        const std::string& out_pcd_path,
        const float& radius,
        const float& min_nb_neighbors);

    // Ground Removal using PassThrough filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_ground_removal(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
        const float& min_range,
        const float& max_range);

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_ground_removal_and_visualize(
        const std::string& in_pcd_path,
        const float& min_range,
        const float& max_range);

    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_ground_removal_and_save(
        const std::string& in_pcd_path,
        const std::string& out_pcd_path,
        const float& min_range,
        const float& max_range);
    
    // Statistical Outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_statistical_outlier_removal(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
        const int& mean_k_value,
        const float& std_dev_mul_threshold);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_statistical_outlier_removal_and_visualize(
        const std::string& in_pcd_path,
        const int& mean_k_value,
        const float& std_dev_mul_threshold);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_statistical_outlier_removal_and_save(
        const std::string& in_pcd_path,
        const std::string& out_pcd_path,
        const int& mean_k_value,
        const float& std_dev_mul_threshold);
};

/**
 * Default Constructor
*/
PointCloudProcessing::PointCloudProcessing() = default;

/**
 * Default Destructor
*/
PointCloudProcessing::~PointCloudProcessing() = default;

/***********************************************************************************************
 * VoxelGrid Filtering
 * ********************************************************************************************/
/**
 * Applies the voxel filter to in_cloud and returns the filtered cloud
 * 
 * This is a modified version of the VoxelGrid filtering found at: 
 * https://pcl-tutorials.readthedocs.io/en/master/voxel_grid.html?highlight=voxelGrid
*/
pcl::PCLPointCloud2::Ptr PointCloudProcessing::apply_voxel_filter(
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
    
    return cloud_voxelized;
}


/**
 * Applies Voxel Filter and visualizes in PCL Visualizer
 * 
 * Returns filtered cloud.
*/
pcl::PCLPointCloud2::Ptr PointCloudProcessing::apply_voxel_filter_and_visualize(
    const std::string& in_pcd_path,
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
    pcl::PCLPointCloud2::Ptr cloud_voxelized = apply_voxel_filter(
        cloud_read,
        leaf_size_x,
        leaf_size_y,
        leaf_size_z);

    // Convert to pcl::PointCloud<> object to visualize
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_voxelized, *cloud_to_visualize);

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

    return cloud_voxelized;
}

/**
 * Applies Voxel Filter and saves the resulting PCD file to out_pcd_path
 * 
 * Returns filtered cloud.
*/
pcl::PCLPointCloud2::Ptr PointCloudProcessing::apply_voxel_filter_and_save(
    const std::string& in_pcd_path,
    const std::string& out_pcd_path,
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
    pcl::PCLPointCloud2::Ptr cloud_voxelized = apply_voxel_filter(
        cloud_read,
        leaf_size_x,
        leaf_size_y,
        leaf_size_z);

    // Write to out_pcd_path
    pcl::PCDWriter writer;
    writer.write(out_pcd_path, cloud_voxelized, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    return cloud_voxelized;
}

/***********************************************************************************************
 * Radial Filtering for Outlier removal
 * ********************************************************************************************/
/**
 * Applies radial filter to the cloud pointer. Explanation for this is available here:
 * https://pcl-tutorials.readthedocs.io/en/master/radius_outlier_removal.html?highlight=radial%20filter
 * 
 * It will remove points from a PointCloud that do not have a given number of neighbors within a 
 * specific radius from their location.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_radial_filter(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    const float& radius,
    const float& min_nb_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
		   << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_nb_neighbors);

	// apply filter
	outrem.filter (*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
		   << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
	
    return cloud_filtered;
}


/**
 * Applies radial filter to the point cloud at in_pcd_path and saves it at out_pcd_path
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_radial_filter_and_save(
    const std::string& in_pcd_path,
    const std::string& out_pcd_path,
    const float& radius,
    const float& min_nb_neighbors)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_radial_filter(
        cloud,
        radius,
        min_nb_neighbors);

    pcl::PCDWriter writer;
    writer.write(out_pcd_path, *cloud_filtered);

    return cloud_filtered;
}

/**
 * Applies radial filter to the point cloud at in_pcd_path and visualizes it in PCLVisualizer
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_radial_filter_and_visualize(
    const std::string& in_pcd_path,
    const float& radius,
    const float& min_nb_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);

    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_radial_filter(
        cloud,
        radius,
        min_nb_neighbors);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        cloud_filtered,
        "Radial Filtered point cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Radial Filtered point cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return cloud_filtered;
}

/***********************************************************************************************
 * Ground Removal using PassThrough Filtering
 * ********************************************************************************************/
/**
 * Removes all the points on the ground plane from the input point cloud using the
 * PassThrough filter. Returns the filtered Point Cloud pointer.
 * Points that lie between min_range and max_range along the z-axis are KEPT. Everything
 * else is removed.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_ground_removal(
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
 * Takes in_pcd_path to PCD file, removes ground and saves it into a PCD file at out_pcd_path
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_ground_removal_and_save(
    const std::string& in_pcd_path,
    const std::string& out_pcd_path,
    const float& min_range,
    const float& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_ground_removal(
        cloud,
        min_range,
        max_range);

    pcl::PCDWriter writer;
    writer.write(out_pcd_path, *cloud_filtered);

    return cloud_filtered;
}


/**
 * Takes in_pcd_path to PCD file, removes ground and visualizes using PCLVisualizer. Use for debugging.
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_ground_removal_and_visualize(
    const std::string& in_pcd_path,
    const float& min_range,
    const float& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);
    
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_ground_removal(
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

    return cloud_filtered;
}

/***********************************************************************************************
 * Statistical Outlier Removal
 * ********************************************************************************************/
/**
 * Removes Outliers from the cloud using StatisticalOutlierRemoval returns the filtered cloud. 
 * The number of neighbors to analyze for each point is set by `mean_k_value`. All points that have a 
 * distance larger than the `std_dev_mul_threshold` * standard deviations of the mean distance 
 * to the query point will be marked as outliers and removed.
 * 
 * Explanation available here: https://pcl-tutorials.readthedocs.io/en/master/statistical_outlier.html
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_statistical_outlier_removal(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_cloud,
    const int& mean_k_value,
    const float& std_dev_mul_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    std::cerr << "PointCloud before filtering: " << in_cloud->width * in_cloud->height 
       << " data points (" << pcl::getFieldsList(*in_cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud);
    sor.setMeanK(mean_k_value);
    sor.setStddevMulThresh(std_dev_mul_threshold);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
         << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
        
    return cloud_filtered;
}

/**
 * Takes in_pcd_path to PCD file, removes outliers using StatisticalOutlierRemoval 
 * and visualizes using PCLVisualizer. Use for debugging.
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_statistical_outlier_removal_and_visualize(
    const std::string& in_pcd_path,
    const int& mean_k_value,
    const float& std_dev_mul_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);
    
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_statistical_outlier_removal(
        cloud,
        mean_k_value,
        std_dev_mul_threshold);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        cloud_filtered,
        "Outliers Removed");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Outliers Removed");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return cloud_filtered;
}

/**
 * Takes in_pcd_path to PCD file, removes outliers using StatisticalOutlierRemoval 
 * and saves to out_pcd_path. Use for debugging.
 * 
 * Returns filtered cloud.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::apply_statistical_outlier_removal_and_save(
    const std::string& in_pcd_path,
    const std::string& out_pcd_path,
    const int& mean_k_value,
    const float& std_dev_mul_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(in_pcd_path, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = apply_statistical_outlier_removal(
        cloud,
        mean_k_value,
        std_dev_mul_threshold);

    pcl::PCDWriter writer;
    writer.write(out_pcd_path, *cloud_filtered);

    return cloud_filtered;
}

} // namespace dataset_generation
