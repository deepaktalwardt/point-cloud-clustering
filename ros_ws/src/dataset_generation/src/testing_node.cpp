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
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/sources/sources_1";
    
    std::string in_folder_testset = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/out_300_transformed";

    dataset_generation::PointCloudClassifier pcc(
        in_folder_sources,
        in_folder_testset);

    // Using ICP with 300 points+
    json icp_options;
    icp_options["transformation_epsilon"] = 1e-9;
    icp_options["max_correspondence_distance"] = 0.1;
    icp_options["maximum_iterations"] = 50;
    icp_options["euclidean_fitness_epsilon"] = 0.001;
    icp_options["RANSAC_outlier_rejection_threshold"] = 0.2;
    
    json results_icp = pcc.predict_all_with_icp(icp_options);
    // std::cout << std::setw(4) << results_icp << std::endl; 

    std::string out_json_icp_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/test_results/test_results_sources_1/test_results_sources_1_300_icp.json";
    std::ofstream out_json_icp_stream(out_json_icp_path);
    out_json_icp_stream << std::setw(4) << results_icp << std::endl;

    // ICP Nonlinear with 300 points+
    json icp_nl_options;
    icp_nl_options["transformation_epsilon"] = 1e-9;
    icp_nl_options["max_correspondence_distance"] = 0.1;
    icp_nl_options["maximum_iterations"] = 50;
    icp_nl_options["euclidean_fitness_epsilon"] = 0.001;
    icp_nl_options["RANSAC_outlier_rejection_threshold"] = 0.2;
    
    json results_icp_nl = pcc.predict_all_with_icp_non_linear(icp_nl_options);
    // std::cout << std::setw(4) << results_icp_nl << std::endl;

    std::string out_json_icp_nl_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/test_results/test_results_sources_1/test_results_sources_1_300_icp_nl.json";
    std::ofstream out_json_icp_nl_stream(out_json_icp_nl_path);
    out_json_icp_nl_stream << std::setw(4) << results_icp_nl << std::endl;

    /*****************************************************************************************************/
    // ICP with 500 points+
    in_folder_testset = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/out_500_transformed";
    
    dataset_generation::PointCloudClassifier pcc2(
        in_folder_sources,
        in_folder_testset);
    
    // Using ICP with 300 points+
    // json icp_options;
    icp_options["transformation_epsilon"] = 1e-9;
    icp_options["max_correspondence_distance"] = 0.1;
    icp_options["maximum_iterations"] = 2;
    icp_options["euclidean_fitness_epsilon"] = 0.01;
    icp_options["RANSAC_outlier_rejection_threshold"] = 1.5;
    
    json results_icp2 = pcc2.predict_all_with_icp(icp_options);
    // std::cout << std::setw(4) << results_icp << std::endl; 

    std::string out_json_icp_path2 =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/test_results/test_results_sources_1/test_results_sources_1_500_icp.json";
    std::ofstream out_json_icp_stream2(out_json_icp_path2);
    out_json_icp_stream2 << std::setw(4) << results_icp2 << std::endl;

    // ICP Nonlinear with 300 points+
    // json icp_nl_options;
    icp_nl_options["transformation_epsilon"] = 1e-9;
    icp_nl_options["max_correspondence_distance"] = 0.1;
    icp_nl_options["maximum_iterations"] = 50;
    icp_nl_options["euclidean_fitness_epsilon"] = 0.01;
    icp_nl_options["RANSAC_outlier_rejection_threshold"] = 2.5;
    
    json results_icp_nl2 = pcc2.predict_all_with_icp_non_linear(icp_nl_options);
    // std::cout << std::setw(4) << results_icp_nl << std::endl;

    std::string out_json_icp_nl_path2 =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/test_results/test_results_sources_1/test_results_sources_1_500_icp_nl.json";
    std::ofstream out_json_icp_nl_stream2(out_json_icp_nl_path2);
    out_json_icp_nl_stream2 << std::setw(4) << results_icp_nl2 << std::endl;



    /*****************************************************************************************************/
    
    // // Using NDT
    // json ndt_options;
    // ndt_options["transformation_epsilon"] = 1e-9;
    // ndt_options["maximum_iterations"] = 10;
    // ndt_options["step_size"] = 0.1;
    // ndt_options["set_resolution"] = 0.2;

    // ndt_options["transformation_epsilon"] = 1e-5;
    // ndt_options["maximum_iterations"] = 2;
    // ndt_options["step_size"] = 0.1;
    // ndt_options["set_resolution"] = 1.0;
    
    // json results_ndt = pcc.predict_all_with_ndt(ndt_options);
    // // std::cout << std::setw(4) << results_ndt << std::endl;

    // std::string out_json_ndt_path =
    //     "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/testbag1_2020-05-09-19-39-52.bag/test_results/test_results_sources_1/test_results_sources_1_100_ndt.json";
    // std::ofstream out_json_ndt_stream(out_json_ndt_path);
    // out_json_ndt_stream << std::setw(4) << results_ndt << std::endl;

    return 0;
}
