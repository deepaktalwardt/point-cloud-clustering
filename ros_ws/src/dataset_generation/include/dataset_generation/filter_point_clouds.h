#pragma once

#include <dirent.h>
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <unordered_set>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

#include "extract_point_cloud_objects.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dataset_generation/json.hpp"

using namespace std::literals::chrono_literals;

namespace dataset_generation
{


class PCDFiltering
{
public:
    PCDFiltering(
        std::string in_folder_path,
        std::string object_name,
        std::string out_folder_path);

    ~PCDFiltering();

    void VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
private:
    std::string in_folder_path_;
    std::string object_name_;
    std::string out_folder_path_;
};
PCDFiltering::PCDFiltering(
        std::string in_folder_path,
        std::string object_name,
        std::string out_folder_path)
{
    in_folder_path_=in_folder_path;
    object_name_=object_name;
    out_folder_path_=out_folder_path;
}
PCDFiltering::~PCDFiltering()
{}

/*
This method is a modified version of the VoxelGrid filtering found at: 
https://pcl-tutorials.readthedocs.io/en/master/voxel_grid.html?highlight=voxelGrid
*/
void PCDFiltering::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    pcl::PCLPointCloud2::Ptr cloud_read (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
    
    //Read PCD files from the path given
    std::string path=in_folder_path_+object_name_+".pcd";
    pcl::PCDReader reader;
    reader.read(path, *cloud_read);

    std::cerr << "PointCloud before filtering: " << cloud_read->width * cloud_read->height 
       << " data points (" << pcl::getFieldsList (*cloud_read) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud (cloud_read);
    vox.setLeafSize (leaf_size_x,leaf_size_y,leaf_size_z);
    vox.filter (*cloud_voxelized);

    std::cerr << "PointCloud after filtering: " << cloud_voxelized->width * cloud_voxelized->height 
       << " data points (" << pcl::getFieldsList (*cloud_voxelized) << ")." << std::endl;

    std::string save_path = out_folder_path_ + object_name_ +"_voxelized.pcd";
    pcl::PCDWriter writer;
    writer.write (save_path, *cloud_voxelized, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

} // namespace dataset_generation
