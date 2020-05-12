#include <iostream>
#include <string>

#include "dataset_generation/visualize_source_test.h"

int main()
{
    std::string in_pcd_path=
            "/home/parshwa/Desktop/bag6/bag6_2020-05-09-19-08-22.bag/out_10_transformed/Sedan-1589076536162069-3.pcd";
    std::string in_source_path=
            "/home/parshwa/Desktop/bag6/bag6_2020-05-09-19-08-22.bag/combined_objects_10_pcd/Jeep_combined.pcd";
    
    dataset_generation::PointCloudVisualization pcv;
    pcv.compare_source_and_test(in_pcd_path,
                in_source_path,1);

    return 0;
}