#include <string>
#include <iostream>

#include "grasp_detect/grasp_detector_two_stage.h"
#include "grasp_detect/pointnet_grasp_client.h"
#include "grasp_detect/util/transform.h"
#include "grasp_detect/GraspDetect.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <gpd/config_file.h>
// #include <gpd/grasp_detector.h>

#include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PCL_Cloud;
namespace fs = boost::filesystem;

bool checkFileExists(const std::string &file_name)
{
    std::ifstream file;
    file.open(file_name.c_str());
    if (!file)
    {
        std::cout << "File " + file_name + " could not be found!\n";
        return false;
    }
    file.close();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_grasps_one_stage");

    int start_step = 0;
    if (argc > 1)
    {
        start_step = std::stoi(argv[1]);
    }

    std::string config_filename("/home/waha/catkin_research_ws/src/grasp_detect/config/gpd_parameter_two_stage.cfg");

    // Read parameters from configuration file.
    auto params_manager = std::make_shared<grasp_detect::util::ParamsManager>(config_filename);

    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position = params_manager->getPreprocessCloudParams().camera_position;
    Eigen::Matrix3Xd view_points(3, camera_position.size() / 3);
    for (int i = 0; i < camera_position.size() / 3; i++)
    {
        view_points.col(i) << camera_position[i * 3 + 0], camera_position[i * 3 + 1], camera_position[i * 3 + 2];
    }

    string data_root("/home/waha/catkin_research_ws/src/data/");
    string pcd_folder = data_root + "bullet/pointcloud/";

    string path = pcd_folder;
    if (!fs::exists(path))
        return -1;

    vector<std::pair<string, int>> all_pcd_path;
    vector<string> step_vec;
    vector<string> pose_vec;
    int index = 0;
    for (const auto &entry : fs::directory_iterator(path))
    {
        string pcd_path = entry.path().string();

        vector<string> split_vec;
        boost::split(split_vec, pcd_path, boost::is_any_of("/"));
        string pcd_file_name = split_vec.at(split_vec.size() - 1);
        split_vec.clear();

        boost::split(split_vec, pcd_file_name, boost::is_any_of("_."));

        all_pcd_path.push_back(std::make_pair(pcd_path, index++));
        step_vec.push_back(split_vec.at(1));
        pose_vec.push_back(split_vec.at(2));
    }

    std::sort(all_pcd_path.begin(), all_pcd_path.end());

    for (int i = start_step * view_points.cols(); i < all_pcd_path.size(); i++)
    {
        std::cout << std::endl
                  << "******************************" << std::endl;
        auto data = all_pcd_path[i];
        string pcd_path = data.first;
        int ids = data.second;
        std::cout << "[info] process pcd file " << pcd_path << std::endl;

        grasp_detect::util::Cloud cloud(pcd_path, view_points.col(std::stoi(pose_vec.at(ids))));
        if (cloud.getCloudOriginal()->size() == 0)
        {
            std::cout << "ERROR: Point cloud is empty!" << std::endl;
            return -1;
        }

        grasp_detect::PointNetGraspClient grasp_predict_client("predict_grasp");
        grasp_detect::PointNetClassifyClient classify_client("reclassify_grasp");
        grasp_detect::GraspDetectorTwoStage detector(params_manager, grasp_predict_client, classify_client);

        //　预处理点云
        detector.preprocessPointCloud(cloud);

        // 预测grasp
        std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> handles = detector.detectGrasps(cloud);

        if (!ros::ok())
            return 0;
    }

    return 0;
}
