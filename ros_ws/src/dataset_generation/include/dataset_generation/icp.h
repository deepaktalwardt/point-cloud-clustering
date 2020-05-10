//
//  icp.h
//  
//
//  Created by Kedar Acharya on 5/9/20.
//

#ifndef icp_h
#define icp_h
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ClusterAlgorithm
{

public:

    ClusterAlgorithm();

    ~ClusterAlgorithm();
    
    int ICP_implement(
        const std::string& directory,
        const std::string& input_pcd_path,
        const std::string& target_name);
    
private:
    const std::string& directory;
    const std::string& input_pcd_path;
    const std::string& target_name;

};

/**
 * Default Constructor
*/
ClusterAlgorithm::ClusterAlgorithm() = default;

/**
 * Default Destructor
*/
 ClusterAlgorithm::~ClusterAlgorithm() = default;

int ICP_implement(
    const std::string& directory,
    const std::string& input_pcd_path,
    const std::string& target_name)
{
    DIR *dir;
    class dirent *ent;
    class stat st;
    int count=0;
    
    dir = opendir(directory.c_str());
    while ((ent = readdir(dir)) != NULL) {
        const std::string file_name = ent->d_name;
        const std::string target_pcd_path = directory + "/" + file_name;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ> ());
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        
        pcl::PCDReader reader;
        reader.read(target_pcd_path, *cloud_target);
        reader.read(input_pcd_path, *in_cloud)
        
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations (iterations);
        icp.setInputSource (in_cloud);
        icp.setInputTarget (cloud_target);
        icp.align (*in_cloud);
        icp.setMaximumIterations (1);
        if (icp.hasConverged () and icp.getFitnessScore() < 0.2)
        {
            if(target_pcd_path.find(target_name) != std::string::npos){
                count++;
            }
        }
    }
    return count;
}
#endif /* icp_h */
