#pragma once

#include <dirent.h>
#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <unordered_set>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dataset_generation/json.hpp"

using namespace std::literals::chrono_literals;

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
    
    ~ExtractPointCloudObjects();
    
    void extract_objects_from_all_pcds(
        int min_nb_points_threshold = 50);
    
    void extract_objects_from_pcd(
        pcl::PCLPointCloud2 in_cloud_blob,
        const json& dets3d_json,
        const std::string& pcd_fn,
        int min_nb_points_threshold);
    
    void visualize_objects_in_pcd(
        pcl::PCLPointCloud2 in_cloud_blob,
        const json& dets3d_json);
    
    void visualize_cloud_only(
        pcl::PCLPointCloud2 in_cloud_blob);
    
    void visualize_pcd(
        const std::string& in_pcd_path);
    
    void concatenate_objects_and_visualize(
        const std::string& in_folder_path,
        const std::string& object_name);

    void concatenate_objects_and_save(
        const std::string& in_folder_path,
        const std::string& object_name);

private:
    std::string in_folder_pcd_;
    std::string in_folder_dets3d_;
    std::string out_folder_pcd_;

    std::unordered_set<std::string> pcd_names_set_, dets3d_names_set_;

    int not_common_fn_ = 0;

    std::unordered_map<std::string, int> labels_count_map_;

    std::vector<std::pair<std::string, std::string>> common_fn_vec_;
};

/**
 * Helper function
 * 
 * Finds all files in the `directory` and puts them in the set `s`
*/
void get_files_in_directory(
    const std::string& directory,
    std::unordered_set<std::string>& s)
{
    DIR *dir;
    class dirent *ent;
    class stat st;

    dir = opendir(directory.c_str());
    while ((ent = readdir(dir)) != NULL) {
        const std::string file_name = ent->d_name;
        const std::string full_file_name = directory + "/" + file_name;

        if (file_name[0] == '.')
            continue;

        if (stat(full_file_name.c_str(), &st) == -1)
            continue;

        const bool is_directory = (st.st_mode & S_IFDIR) != 0;

        if (is_directory)
            continue;

        s.insert(file_name);
    }
    closedir(dir);
}

/**
 * Constructor
*/
ExtractPointCloudObjects::ExtractPointCloudObjects(
    std::string in_folder_pcd,
    std::string in_folder_dets3d,
    std::string out_folder_pcd):
    in_folder_pcd_(in_folder_pcd),
    in_folder_dets3d_(in_folder_dets3d),
    out_folder_pcd_(out_folder_pcd)
{   
    // Get list of all files in the directories
    get_files_in_directory(in_folder_pcd_, pcd_names_set_);
    get_files_in_directory(in_folder_dets3d_, dets3d_names_set_);

    std::cout << "Number of PCD files: " << pcd_names_set_.size() << std::endl; 
    std::cout << "Number of JSON files: " << dets3d_names_set_.size() << std::endl;

    // Ensure that all PCD files have their respective JSON files
    int i = 0;
    for (auto it = pcd_names_set_.begin(); it != pcd_names_set_.end(); it++)
    {
        std::string pcd_fn = *it;
        std::string dets3d_fn = pcd_fn.substr(0, pcd_fn.size() - 3) + "json";
        
        // If not found, display on screen and remove that PCD from set
        if (dets3d_names_set_.find(dets3d_fn) == dets3d_names_set_.end())
        {
            std::cout << "Detections for " << pcd_fn << " not found!" << std::endl;
            // pcd_names_set_.erase(pcd_fn);
            not_common_fn_++;
        }
        else
        {
            common_fn_vec_.push_back({pcd_fn, dets3d_fn});
        }
        i++;
    }
    std::cout << "Iterations (i): " << i << ", Skipped: " << not_common_fn_ << std::endl;
    std::cout << "Number of common files: " << common_fn_vec_.size() << std::endl;
}

/**
 * Destructor
*/
ExtractPointCloudObjects::~ExtractPointCloudObjects()
{}

/**
 * This function iterates over all pairs of PCD files and detections and calls the `extract_objects_from_pcd`
 * function to extract and save object PCDs from them. Finally, prints a summary of all PCDs extracted.
*/
void ExtractPointCloudObjects::extract_objects_from_all_pcds(
    int min_nb_points_threshold)
{
    // Iterate over all PCD files, find their corresponding detections
    for (const auto& pair_fn : common_fn_vec_)
    {
        std::string pcd_fn = pair_fn.first;
        std::string dets3d_fn = pair_fn.second;

        std::string pcd_file_path, dets3d_file_path;
        pcd_file_path = in_folder_pcd_ + "/" + pcd_fn;
        dets3d_file_path = in_folder_dets3d_ + "/" + dets3d_fn;

        pcl::PCLPointCloud2 in_cloud_blob;
        pcl::io::loadPCDFile(pcd_file_path, in_cloud_blob);
        
        std::ifstream dets3d_file(dets3d_file_path);
        json dets3d_json;
        dets3d_file >> dets3d_json;

        extract_objects_from_pcd(
            in_cloud_blob,
            dets3d_json,
            pcd_fn,
            min_nb_points_threshold);
    }

    // Print out how many objects were found
    int total_nb = 0;
    std::cout << "Number of objects found: " << std::endl;
    for (auto it = labels_count_map_.begin(); it != labels_count_map_.end(); it++)
    {
        std::cout << it->first << ": " << it->second << std::endl;
        total_nb += it->second;
    }
    std::cout << "Total number of objects found: " << total_nb << std::endl;
}


/**
 * Takes in a single point cloud and its corresponding detections json file, output file name, and a
 * minimum_nb_points_threshold. Ignores all objects that have fewer points than this threshold.
 * Saves all extracted objects as PCD files in `out_folder_pcd_`
*/
void ExtractPointCloudObjects::extract_objects_from_pcd(
    pcl::PCLPointCloud2 in_cloud_blob,
    const json& dets3d_json, 
    const std::string& pcd_fn,
    int min_nb_points_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(in_cloud_blob, *in_cloud);

    int curr_det_idx = 1;
    
    // Iterate over detections in dets3d_json
    int num_dets3d = dets3d_json["detections"].size();
    json dets3d = dets3d_json["detections"];

    for (int i = 0; i < num_dets3d; i++)
    {
        // Get the bounding box
        json bbox = dets3d[i]["bbox"];

        // Create CropBox for the box
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(in_cloud);

        // Get parameters from the json file
        json bbox_pos = bbox["position"]["position"];
        json bbox_ori = bbox["position"]["orientation"];
        json bbox_size = bbox["size"];

        Eigen::Vector3f translation(
            static_cast<float>(bbox_pos["x"]),
            static_cast<float>(bbox_pos["y"]),
            static_cast<float>(bbox_pos["z"]));

        Eigen::Quaternionf quaternion(
            bbox_ori["w"],
            bbox_ori["x"],
            bbox_ori["y"],
            bbox_ori["z"]);
        
        Eigen::Vector3f euler_angles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

        // Align crop box around the object
        crop_box.setTranslation(translation);
        crop_box.setRotation(euler_angles);

        // Define min point with padding
        Eigen::Vector4f min_point(
            (-static_cast<float>(bbox_size["x"]) / 2.0) - 0.5,
            (-static_cast<float>(bbox_size["y"]) / 2.0) - 0.5,
            0.0,
            0.0);

        // Define max point with padding
        Eigen::Vector4f max_point(
            (static_cast<float>(bbox_size["x"]) / 2.0) + 0.5,
            (static_cast<float>(bbox_size["y"]) / 2.0) + 0.5,
            static_cast<float>(bbox_size["z"]) + 1.0,
            0.0);
        
        crop_box.setMin(min_point);
        crop_box.setMax(max_point);

        // Filter and save to out cloud
        pcl::PointCloud<pcl::PointXYZ> out_cloud;
        crop_box.filter(out_cloud);

        // Create output pcd file name
        std::string label = dets3d[i]["label"];

        std::stringstream out_pcd_name;
        out_pcd_name << label << "-" <<
            pcd_fn.substr(0, pcd_fn.size() - 4) << "-" << curr_det_idx << ".pcd";

        // Make sure the number points are above the threshold
        if (out_cloud.size() < min_nb_points_threshold)
        {
            std::cout << "Skipped: " << out_pcd_name.str() <<
                " because number of points is " <<
                out_cloud.size() << " < " << min_nb_points_threshold << std::endl;
            continue;
        }

        // Inverse Transform the extracted cloud
        pcl::PointCloud<pcl::PointXYZ> out_cloud_transformed;

        Eigen::Vector3f translation_inv = -translation;
        Eigen::Quaternionf quaternion_inv(1, 0, 0, 0);

        // First, translate the cloud
        pcl::transformPointCloud(
            out_cloud,
            out_cloud_transformed,
            translation_inv,
            quaternion_inv);
        
        // Then, rotate the cloud
        pcl::transformPointCloud(
            out_cloud_transformed,
            out_cloud_transformed,
            Eigen::Vector3f(0, 0, 0),
            quaternion.inverse());

        std::stringstream out_pcd_path;
        out_pcd_path << out_folder_pcd_ << "/" << out_pcd_name.str();
        
        pcl::io::savePCDFileASCII(out_pcd_path.str(), out_cloud_transformed);

        std::cout << "Extracted: " << out_pcd_name.str() << std::endl;

        // Update index for the next object
        curr_det_idx++;

        // Add to labels_map
        labels_count_map_[label]++;
    }
}

/**
 * Draws bounding boxes around all the objects in the provided point cloud. Uses the detections
 * json file.
*/
void ExtractPointCloudObjects::visualize_objects_in_pcd(
    pcl::PCLPointCloud2 in_cloud_blob,
    const json& dets3d_json)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(in_cloud_blob, *in_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        in_cloud,
        "sample cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); 

    // Iterate over detections in dets3d_json
    int num_dets3d = dets3d_json["detections"].size();
    json dets3d = dets3d_json["detections"];

    for (int i = 0; i < num_dets3d; i++)
    {
        // Get the bounding box
        json bbox = dets3d[i]["bbox"];

        // Set paramaters
        json bbox_pos = bbox["position"]["position"];
        json bbox_ori = bbox["position"]["orientation"];
        json bbox_size = bbox["size"];

        Eigen::Vector3f translation(
            static_cast<float>(bbox_pos["x"]),
            static_cast<float>(bbox_pos["y"]),
            static_cast<float>(bbox_pos["z"]) + 1.4);

        Eigen::Quaternionf quarternion(
            bbox_ori["w"],
            bbox_ori["x"],
            bbox_ori["y"],
            bbox_ori["z"]);
        
        double depth = static_cast<float>(bbox_size["z"]) + 1.0;
        double width = static_cast<float>(bbox_size["x"]) + 1.0;
        double height = static_cast<float>(bbox_size["y"]) + 1.0;

        // Get Label
        std::string label = dets3d[i]["label"];
        
        viewer->addCube(
            translation,
            quarternion,
            width,
            height,
            depth,
            label + "_" + std::to_string(i));
    }

    viewer->setRepresentationToWireframeForAllActors();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

/**
 * Visualize a single point cloud
*/
void ExtractPointCloudObjects::visualize_cloud_only(
    pcl::PCLPointCloud2 in_cloud_blob)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(in_cloud_blob, *in_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(in_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

/**
 * Visualize directly from a PCD
*/
void ExtractPointCloudObjects::visualize_pcd(
    const std::string& in_pcd_path)
{
    pcl::PCLPointCloud2 in_cloud_blob;
    pcl::io::loadPCDFile(in_pcd_path, in_cloud_blob);

    visualize_cloud_only(in_cloud_blob);
}


/**
 * Load all PCD files in the `in_folder_path` of type `object_name` and visualize them
 * all together.
*/
void ExtractPointCloudObjects::concatenate_objects_and_visualize(
    const std::string& in_folder_path,
    const std::string& object_name)
{
    // Get all file names in the directory and put in a set
    std::unordered_set<std::string> pcd_fn_set;
    get_files_in_directory(in_folder_path, pcd_fn_set);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); 
    
    // For each file
    for (const auto& fn : pcd_fn_set)
    {
        // Only compute if file is of `object_name` type
        if (fn.find(object_name) != std::string::npos)
        {
            std::string pcd_file_path = in_folder_path + "/" + fn;
            pcl::PCLPointCloud2 cloud_blob;
            pcl::io::loadPCDFile(pcd_file_path, cloud_blob);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(cloud_blob, *cloud_ptr);

            // Add cloud and set visualization property
            viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, fn);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, fn);

            // Get random color to differentiate between clouds
            float r_color = 0.4 +
                static_cast <float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.4)));
            float g_color = 0.4 +
                static_cast <float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.4)));
            float b_color = 0.4 +
                static_cast <float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.4)));

            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r_color, g_color, b_color, fn);
        }
    }

    

    // Visualization while loop
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void ExtractPointCloudObjects :: concatenate_objects_and_save(
                            const std::string& in_folder_path,
                            const std::string& object_name)
{
    std::unordered_set<std::string> pcd_fn_set;
    get_files_in_directory(in_folder_path, pcd_fn_set);

    // Create a common concatenation object for each PCD file of the detection type
    pcl::PointCloud<pcl::PointXYZ>cloud_all;

    //Concate the pcd files
    for (const auto& fn : pcd_fn_set)
    {
        // Only compute if file is of `object_name` type
        if (fn.find(object_name) != std::string::npos)
        {
            std::string pcd_file_path = in_folder_path + "/" + fn;
            pcl::PCLPointCloud2 cloud_blob;
            pcl::io::loadPCDFile(pcd_file_path, cloud_blob);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(cloud_blob, *cloud_temp);
            //pcl::io::concatenate(cloud_all, cloud_blob,cloud_all);
            cloud_all += *cloud_temp;
        }
    }
    std::string save_path = "/home/parshwa/Desktop/CMPE_255 Project/bag5_2020-05-05-15-11-17.bag-20200506T192755Z-001/bag5_2020-05-05-15-11-17.bag/" + object_name + ".pcd";
    pcl::io::savePCDFile(save_path,cloud_all);
}

}   // namespace dataset_generation