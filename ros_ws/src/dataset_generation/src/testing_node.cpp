#include <iostream>
#include <string>
#include <vector>

#include "dataset_generation/extract_point_cloud_objects.h"
#include "dataset_generation/point_cloud_classifier.h"
#include "dataset_generation/json.hpp"

using json = dataset_generation::json;

int main(int argc, char** argv)
{
    std::string in_folder_sources = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/source_object_clouds";
    
    std::string in_folder_testset = 
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_10_transformed";

    dataset_generation::PointCloudClassifier pcc(
        in_folder_sources,
        in_folder_testset);
    
    // json results_icp = pcc.predict_all_using_icp();
    // json results_ndt = pcc.predict_all_using_ndt();
    
    return 0;
}