#pragma once

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "dataset_generation/json.hpp"

namespace dataset_generation
{

using json = nlohmann::json;

class ExtractPointCloudObjects
{

public:
    ExtractPointCloudObjects(
        std::string in_folder_pcd,
        std::string in_folder_dets3d,
        std::string out_folder_pcd);
    
    void extract_objects_from_all_pcds();
    
    bool extract_objects_from_pcd(
        pcl::PCLPointCloud2::Ptr in_cloud_ptr,
        const json& dets3d_json);

private:
    std::string in_folder_pcd_;
    std::string in_folder_dets3d_;
    std::string out_folder_pcd_;
};

ExtractPointCloudObjects::ExtractPointCloudObjects(
    std::string in_folder_pcd,
    std::string in_folder_dets3d,
    std::string out_folder_pcd):
    in_folder_pcd_(in_folder_pcd),
    in_folder_dets3d_(in_folder_dets3d),
    out_folder_pcd_(out_folder_pcd)
{
}

void ExtractPointCloudObjects::extract_objects_from_all_pcds()
{
}

bool ExtractPointCloudObjects::extract_objects_from_pcd(
    pcl::PCLPointCloud2::Ptr in_cloud_ptr,
    const json& dets3d_json)
{
    // Iterate over detections in dets3d_json

    // Create CropBox for the box

    // Extract indices inside the box

    // Save the new PointCloud
    
}

}   // namespace dataset_generation