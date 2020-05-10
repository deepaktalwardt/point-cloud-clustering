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

    json icp_options;
    icp_options["transformation_epsilon"] = 1e-4;
    icp_options["max_correspondence_distance"] = 0.1;
    icp_options["maximum_iterations"] = 2;
    icp_options["euclidean_fitness_epsilon"] = 0.01;
    icp_options["RANSAC_outlier_rejection_threshold"] = 1.5;
    
    json results_icp = pcc.predict_all_with_icp(icp_options);
    std::cout << std::setw(4) << results_icp << std::endl; 

    std::string out_json_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_icp1.json";
    std::ofstream out_json_stream(out_json_path);
    out_json_stream << std::setw(4) << results_icp << std::endl;

    return 0;
}