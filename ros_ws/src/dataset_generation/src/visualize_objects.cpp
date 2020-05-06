#include <iostream>
#include <string>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::literals::chrono_literals;

int main(int argc, char** argv)
{
    std::string pcd_fn = "SUV-1588716715266281-2.pcd";

    std::string non_transformed_cloud_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_100_nontransformed/" + pcd_fn;

    std::string transformed_cloud_path =
        "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE 255/Project/raw_msgs/bag5_2020-05-05-15-11-17.bag/out_100_transformed/" + pcd_fn;

    pcl::PCLPointCloud2 non_transformed_cloud_blob, transformed_cloud_blob;
    pcl::io::loadPCDFile(non_transformed_cloud_path, non_transformed_cloud_blob);
    pcl::io::loadPCDFile(transformed_cloud_path, transformed_cloud_blob);

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(non_transformed_cloud_blob, *non_transformed_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(transformed_cloud_blob, *transformed_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(
        non_transformed_cloud,
        "Non-transformed cloud");

    viewer->addPointCloud<pcl::PointXYZ>(
        transformed_cloud,
        "Transformed cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Non-transformed cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Transformed cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); 

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}