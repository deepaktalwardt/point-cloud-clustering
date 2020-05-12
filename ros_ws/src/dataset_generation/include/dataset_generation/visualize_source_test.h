#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

#include "dataset_generation/process_point_clouds.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace dataset_generation
{

class PointCloudVisualization
{
public:
    PointCloudVisualization();
    ~PointCloudVisualization();

    void compare_source_and_test(
        std::string in_pcd_path,
        std::string in_source_path,
        int iterations);
};

PointCloudVisualization::PointCloudVisualization() = default;
PointCloudVisualization::~PointCloudVisualization() = default;

void PointCloudVisualization:: compare_source_and_test(
        std::string in_pcd_path,
        std::string in_source_path,
        int iterations)
{
    dataset_generation::PointCloudProcessing pcp;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudT::Ptr cloud_tr (new PointCloudT); 

    pcl::io::loadPCDFile(in_pcd_path,*test_cloud);
    pcl::io::loadPCDFile(in_source_path,*source_cloud);

    test_cloud=pcp.apply_ground_removal(test_cloud,0.2,2);

    *cloud_tr = *test_cloud;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (test_cloud);
    icp.setInputTarget (source_cloud);
    icp.align (*test_cloud);
    icp.setMaximumIterations (1); 
/*
Some part of this code is commented out for visualization purposes.
Uncomments those lines and adjust parameters to see multi view port
*/

    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    
    // Create two vertically separated viewports  
    // int v1 (0);
    int v2 (0);
    
    // viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    // viewer.createViewPort (1.0, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_h (source_cloud, 255,20,20);
    // viewer.addPointCloud (source_cloud, source_cloud_color_h, "source_cloud_v1", v1);
    viewer.addPointCloud (source_cloud, source_cloud_color_h, "source_cloud_v2", v2);

    // Transformed point cloud is green
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    // viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> test_cloud_color_h (test_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                                (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (test_cloud, test_cloud_color_h, "test_cloud_v2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    // viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    // viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size


  // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

}//namespace dataset_generation