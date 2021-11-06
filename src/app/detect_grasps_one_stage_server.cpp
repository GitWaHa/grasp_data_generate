#include <string>
#include <iostream>

#include "grasp_detect/grasp_detector_one_stage.h"
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PCL_Cloud;

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

class DetectGraspOneStageServer
{
private:
    ros::NodeHandle nh_;
    // ros::Publisher pub_;
    // ros::Subscriber sub_;
    ros::ServiceServer server_;
    tf::TransformListener listener_;

public:
    DetectGraspOneStageServer()
    {
        server_ = nh_.advertiseService("/grasp_detecter/get_pose", &DetectGraspOneStageServer::serverCallBack, this);
    }
    ~DetectGraspOneStageServer() {}

    bool serverCallBack(grasp_detect::GraspDetect::Request &req, grasp_detect::GraspDetect::Response &res);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_grasps_one_stage_server");

    DetectGraspOneStageServer calculatet_pose;
    ros::Duration(1).sleep();
    ROS_INFO("detect_grasps_one_stage server is started");
    ros::spin();

    return 0;
}

bool DetectGraspOneStageServer::serverCallBack(grasp_detect::GraspDetect::Request &req, grasp_detect::GraspDetect::Response &res)
{
    PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);

    if (req.pcd_path == "")
    {
        pcl::fromROSMsg(req.point_cloud, *rgbdCloud);
        pcl::io::savePCDFileASCII("./poingcloud.pcd", *rgbdCloud);
        cout << "[info]"
             << "savePCD ./poingcloud.pcd";
    }
    else if (checkFileExists(req.pcd_path))
    {
        cout << "[info]"
             << "load " + req.pcd_path;
        cout << pcl::io::loadPCDFile(req.pcd_path, *rgbdCloud);
    }
    else
    {
        cout << "[error]"
             << "PCD file not found";
        return (-1);
    }

    std::string config_filename("/home/waha/catkin_research_ws/src/grasp_detect/config/gpd_parameter_one_stage.cfg");

    if (!checkFileExists(config_filename))
    {
        cout << "[error]"
             << "config file not found!";
        return (-1);
    }

    // Read parameters from configuration file.
    auto params_manager = std::make_shared<grasp_detect::util::ParamsManager>(config_filename);

    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position = params_manager->getPreprocessCloudParams().camera_position;

    Eigen::Matrix3Xd view_points(3, camera_position.size() / 3);
    for (int i = 0; i < camera_position.size() / 3; i++)
    {
        view_points.col(i) << camera_position[i * 3 + 0], camera_position[i * 3 + 1], camera_position[i * 3 + 2];
    }

    // Load point cloud from file.
    // grasp_detect::util::Cloud cloud(pcd_filename, view_points.col(0));
    grasp_detect::util::Cloud cloud(rgbdCloud, view_points.col(0));

    if (cloud.getCloudOriginal()->size() == 0)
    {
        std::cout << "[Error]: Input point cloud is empty or does not exist!\n";
        return (-1);
    }

    grasp_detect::PointNetGraspClient client("predict_grasp");
    grasp_detect::GraspDetectorOneStage detector(params_manager, client);

    //　预处理点云
    detector.preprocessPointCloud(cloud);

    std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> handles = detector.detectGrasps(cloud);

    int handle_count = 0;
    for (int i = 0; i < handles.size(); i++)
    {
        Eigen::Vector3d position = handles.at(i).get()->getPosition();
        auto rotation_matrix = handles.at(i).get()->getOrientation();
        Eigen::Vector3d euler = rotation_matrix.eulerAngles(2, 1, 0);

        Eigen::Quaterniond q(rotation_matrix);

        handle_count++;

        geometry_msgs::PoseStamped grasp_pose_msg;
        grasp_pose_msg.header.frame_id = req.point_cloud.header.frame_id;
        grasp_pose_msg.pose.position.x = position(0);
        grasp_pose_msg.pose.position.y = position(1);
        grasp_pose_msg.pose.position.z = position(2);

        double roll, pitch, yaw;
        roll = euler(2);
        pitch = euler(1);
        yaw = euler(0);

        tf2::Quaternion orientation;
        orientation.setRPY(roll, pitch, yaw);
        grasp_pose_msg.pose.orientation = tf2::toMsg(orientation);
        res.grasp_pose.push_back(tf2::toMsg(grasp_pose_msg));
    }

    if (handle_count == 0)
    {
        cout << "[warning]"
             << "no find handle!";
        return false;
    }
    return true;
}
