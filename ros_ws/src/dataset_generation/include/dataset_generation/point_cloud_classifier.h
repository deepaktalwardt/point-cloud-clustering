#pragma once

#include <iostream>
#include <unordered_set>
#include <unordered_map>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common_headers.h>

#include "dataset_generation/extract_point_cloud_objects.h"
#include "dataset_generation/json.hpp"

using json = nlohmann::json;

namespace dataset_generation
{

class PointCloudClassifier
{
public:
    // Constructor and Destructor
    PointCloudClassifier(
        const std::string& in_folder_sources,
        const std::string& in_folder_testset,
        std::vector<std::string> classes = {"Hatchback", "Sedan", "Jeep", "SUV"});

    ~PointCloudClassifier();

    // Single Prediction methods
    json predict_with_icp(
        pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
        const std::string& true_class,
        const json& icp_options);
    
    json predict_with_icp_non_linear(
        pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
        const std::string& true_class,
        const json& icp_options);
    
    json predict_with_ndt(
        pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
        const std::string& true_class,
        const json& icp_options);
    
    // Prediction of all 
    json predict_all(
        const std::string& testing_method,
        const json& options);

    json predict_all_with_icp(
        const json& options);
    
    json predict_all_with_icp_non_linear(
        const json& options);
    
    json predict_all_with_ndt(
        const json& options);

private:
    // Private class variables
    std::string in_folder_sources_;
    std::string in_folder_testset_;
    std::vector<std::string> classes_;

    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> source_point_clouds_;
    std::unordered_set<std::string> test_set_fn_;

    // Private functions
    void load_source_point_clouds_();
};

/**
 * Constructor
*/
PointCloudClassifier::PointCloudClassifier(
    const std::string& in_folder_sources,
    const std::string& in_folder_testset,
    std::vector<std::string> classes):
    in_folder_sources_(in_folder_sources),
    in_folder_testset_(in_folder_testset),
    classes_(classes)
{
    // Enumerate all files in the testset
    get_files_in_directory(in_folder_testset, test_set_fn_);
    std::cout << "Number of files in Test set: " << test_set_fn_.size() << std::endl;

    // Loading sources into map
    load_source_point_clouds_();
    std::cout << "Number of classes: " << classes_.size() << std::endl;
    std::cout << "Number of source point clouds: " << source_point_clouds_.size() << std::endl;
}

/**
 * Destructor
*/
PointCloudClassifier::~PointCloudClassifier() = default;

/**
 * Loads source point clouds into map
*/
void PointCloudClassifier::load_source_point_clouds_()
{
    // Populate file names
    std::unordered_set<std::string> source_cloud_fn;
    get_files_in_directory(in_folder_sources_, source_cloud_fn);

    // Populate sources into map
    for (const std::string& obj_class : classes_)
    {
        for (const std::string& fn : source_cloud_fn)
        {
            if (fn.find(obj_class) != std::string::npos)
            {
                std::string pcd_file_path = in_folder_sources_ + "/" + fn;
                pcl::PCLPointCloud2 cloud_blob;
                pcl::io::loadPCDFile(pcd_file_path, cloud_blob);
                pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromPCLPointCloud2(cloud_blob, *source_cloud);
                source_point_clouds_[obj_class] = source_cloud;
            }
        }
    }
}

/***************************************
 * Predict All Functions
 * ************************************/
json PointCloudClassifier::predict_all(
    const std::string& testing_method,
    const json& options)
{
    // Initialize testing results
    json testing_results;
    testing_results["testing_method"] = testing_method;
    int count_tested = 0;

    // Iterate over all testing files
    for (const std::string& fn : test_set_fn_)
    {
        // Get label
        std::string true_class = "";
        for (const std::string& obj_class : classes_)
        {
            if (fn.find(obj_class) != std::string::npos)
            {
                true_class = obj_class;
                break;
            }
        }

        // Skip this one if label not one of the chosen classes
        if (true_class == "")
        {
            std::cout << "Skipped file " << fn << " because label not selected." << std::endl;
            continue;
        }

        // Load point cloud and get prediction
        std::string test_pcd_file_path = in_folder_testset_ + "/" + fn;
        pcl::PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile(test_pcd_file_path, cloud_blob);

        pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(cloud_blob, *test_cloud);

        if (testing_method == "icp")
        {
            testing_results[fn] = predict_with_icp(test_cloud, true_class, options);
        }
        else if (testing_method == "icp_nl")
        {
            testing_results[fn] = predict_with_icp_non_linear(test_cloud, true_class, options);
        }
        else if (testing_method == "ndt")
        {
            testing_results[fn] = predict_with_ndt(test_cloud, true_class, options);
        }
        else
        {
            std::cout << "Testing method not defined! Exiting testing." << std::endl;
            break; 
        }
        count_tested++;
    }
    std::cout << "Files tested: " << count_tested << std::endl;

    return testing_results;
}

/**
 * Predict all function using ICP
*/
json PointCloudClassifier::predict_all_with_icp(
    const json& options)
{
    return predict_all("icp", options);
}

/**
 * Predict all function using ICP Non-linear
*/
json PointCloudClassifier::predict_all_with_icp_non_linear(
    const json& options)
{
    return predict_all("icp_nl", options);
}

/**
 * Predict all function using NDT
*/
json PointCloudClassifier::predict_all_with_ndt(
    const json& options)
{
    return predict_all("ndt", options);
}


/***************************************
 * ICP Related functions
 * ************************************/

json PointCloudClassifier::predict_with_icp(
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
    const std::string& true_class,
    const json& options)
{
    // Create return JSON
    json result;
    result["tests"] = {};
    result["true_label"] = true_class;

    // Create ICP object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // Apply ICP Paramaters
    icp.setInputTarget(test_cloud);
    icp.setTransformationEpsilon(options["transformation_epsilon"]);
    icp.setMaxCorrespondenceDistance(options["max_correspondence_distance"]);
    icp.setMaximumIterations(options["maximum_iterations"]);
    icp.setEuclideanFitnessEpsilon(options["euclidean_fitness_epsilon"]);
    icp.setRANSACOutlierRejectionThreshold(options["RANSAC_outlier_rejection_threshold"]);

    for (auto it = source_point_clouds_.begin(); it != source_point_clouds_.end(); it++)
    {
        icp.setInputSource(it->second);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud_temp;
        icp.align(aligned_cloud_temp);

        json single_result;
        single_result["has_converged"] = icp.hasConverged();
        single_result["fitness_score"] = icp.getFitnessScore();

        result["tests"][it->first] = single_result;
    }
    return result;
}


json PointCloudClassifier::predict_with_icp_non_linear(
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
    const std::string& true_class,
    const json& options)
{
    // Create return JSON
    json result;
    result["tests"] = {};
    result["true_label"] = true_class;

    // Create ICP object
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // Apply ICP Paramaters
    icp.setInputTarget(test_cloud);
    icp.setTransformationEpsilon(options["transformation_epsilon"]);
    icp.setMaxCorrespondenceDistance(options["max_correspondence_distance"]);
    icp.setMaximumIterations(options["maximum_iterations"]);
    icp.setEuclideanFitnessEpsilon(options["euclidean_fitness_epsilon"]);
    icp.setRANSACOutlierRejectionThreshold(options["RANSAC_outlier_rejection_threshold"]);

    for (auto it = source_point_clouds_.begin(); it != source_point_clouds_.end(); it++)
    {
        icp.setInputSource(it->second);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud_temp;
        icp.align(aligned_cloud_temp);

        json single_result;
        single_result["has_converged"] = icp.hasConverged();
        single_result["fitness_score"] = icp.getFitnessScore();

        result["tests"][it->first] = single_result;
    }
    return result;
}


/***************************************
 * NDT Related functions
 * ************************************/

json PointCloudClassifier::predict_with_ndt(
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud,
    const std::string& true_class,
    const json& options)
{
    // Create return JSON
    json result;
    result["tests"] = {};
    result["true_label"] = true_class;

    // Create NDT object
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Apply NDT Paramaters
    ndt.setTransformationEpsilon (options["transformation_epsilon"]);
    ndt.setStepSize(options["step_size"]);
    ndt.setInputTarget(test_cloud);
    ndt.setResolution(options["set_resolution"]);
    ndt.setMaximumIterations(options["maximum_iterations"]);
    

    for (auto it = source_point_clouds_.begin(); it != source_point_clouds_.end(); it++)
    {
        ndt.setInputSource(it->second);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud_temp;
        ndt.align(aligned_cloud_temp);

        json single_result;
        single_result["has_converged"] = ndt.hasConverged();
        single_result["fitness_score"] = ndt.getFitnessScore();

        result["tests"][it->first] = single_result;
    }
    return result;
}

} // namespace dataset_generation


