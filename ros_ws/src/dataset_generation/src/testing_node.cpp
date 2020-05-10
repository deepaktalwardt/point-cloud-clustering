#include <iostream>
#include <string>
#include <vector>
#include <ostream>

#include "dataset_generation/extract_point_cloud_objects.h"
#include "dataset_generation/point_cloud_classifier.h"
#include "dataset_generation/json.hpp"

using json = dataset_generation::json;

int main(int argc, char** argv)
{
    std::string in_folder_sources = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/source_object_clouds";
    
    std::string in_folder_testset = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/sample_test_set";

    dataset_generation::PointCloudClassifier pcc(
        in_folder_sources,
        in_folder_testset);

    // Using ICP
    json icp_options;
    icp_options["transformation_epsilon"] = 1e-4;
    icp_options["max_correspondence_distance"] = 0.1;
    icp_options["maximum_iterations"] = 2;
    icp_options["euclidean_fitness_epsilon"] = 0.01;
    icp_options["RANSAC_outlier_rejection_threshold"] = 1.5;
    
    json results_icp = pcc.predict_all_with_icp(icp_options);
    std::cout << std::setw(4) << results_icp << std::endl; 

    std::string out_json_icp_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_icp1.json";
    std::ofstream out_json_icp_stream(out_json_icp_path);
    out_json_icp_stream << std::setw(4) << results_icp << std::endl;

    // ICP Nonlinear
    json icp_nl_options;
    icp_nl_options["transformation_epsilon"] = 1e-4;
    icp_nl_options["max_correspondence_distance"] = 0.1;
    icp_nl_options["maximum_iterations"] = 2;
    icp_nl_options["euclidean_fitness_epsilon"] = 0.01;
    icp_nl_options["RANSAC_outlier_rejection_threshold"] = 1.5;
    
    json results_icp_nl = pcc.predict_all_with_icp_non_linear(icp_nl_options);
    std::cout << std::setw(4) << results_icp_nl << std::endl;

    std::string out_json_icp_nl_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_icp_nl.json";
    std::ofstream out_json_icp_nl_stream(out_json_icp_nl_path);
    out_json_icp_nl_stream << std::setw(4) << results_icp_nl << std::endl;
    
    // Using NDT
    json ndt_options;
    ndt_options["transformation_epsilon"] = 1e-2;
    ndt_options["maximum_iterations"] = 35;
    ndt_options["step_size"] = 0.1;
    ndt_options["set_resolution"] = 1.0;
    
    json results_ndt = pcc.predict_all_with_ndt(ndt_options);
    std::cout << std::setw(4) << results_ndt << std::endl;

    std::string out_json_ndt_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_ndt.json";
    std::ofstream out_json_ndt_stream(out_json_ndt_path);
    out_json_ndt_stream << std::setw(4) << results_ndt << std::endl;

    return 0;
}
