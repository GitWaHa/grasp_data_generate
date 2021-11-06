#include <string>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "grasp_detect/util/plot.h"
#include "grasp_detect/util/transform.h"

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace std;
namespace fs = boost::filesystem;

bool readGrasp(const string file_name,
               vector<vector<double>> &center,
               vector<vector<double>> &approach,
               vector<vector<double>> &binnormal,
               vector<vector<double>> &axis);

void plotGrasp(const vector<vector<double>> &center,
               const vector<vector<double>> &approach,
               const vector<vector<double>> &binnormal,
               const vector<vector<double>> &axis,
               grasp_detect::util::Cloud cloud,
               grasp_detect::util::ParamsManager param);

int DoMain(int argc, char *argv[])
{
    std::string config_filename("/home/waha/catkin_research_ws/src/grasp_detect/config/generate_parameter.cfg");

    grasp_detect::util::ParamsManager params_manager(config_filename);

    string data_root("/home/waha/catkin_research_ws/src/data/");
    string camera_pose_filename = data_root + "bullet/camera_pose/camera_pose.txt";
    string pcd_folder = data_root + "bullet/pointcloud/";

    Eigen::Matrix3Xd view_points(3, 4);
    auto camera_pos = params_manager.getPreprocessCloudParams().camera_position;
    view_points.col(0) << camera_pos[0], camera_pos[1], camera_pos[2];
    view_points.col(1) << camera_pos[3], camera_pos[4], camera_pos[5];
    view_points.col(2) << camera_pos[6], camera_pos[7], camera_pos[8];
    view_points.col(3) << camera_pos[9], camera_pos[10], camera_pos[11];

    string path = data_root + "output";
    if (!fs::exists(path))
        return -1;

    for (const auto &entry : fs::directory_iterator(path))
    {
        string grasp_path = entry.path().string();

        vector<string> split_vec;
        boost::split(split_vec, grasp_path, boost::is_any_of("/"));
        string pcd_file_name = split_vec.at(split_vec.size() - 1);
        split_vec.clear();

        boost::split(split_vec, pcd_file_name, boost::is_any_of("_."));
        int view_points_i = std::stoi(split_vec.at(split_vec.size() - 2));

        string str = split_vec.at(0) + "_" + split_vec.at(1) + "_" + split_vec.at(2);
        string pcd_path = pcd_folder + str + ".pcd";

        std::cout << pcd_path << std::endl;
        if (!fs::exists(pcd_path))
            continue;

        grasp_detect::util::Cloud cloud(pcd_path, view_points.col(view_points_i));
        if (cloud.getCloudOriginal()->size() == 0)
        {
            std::cout << "ERROR: Point cloud is empty!" << std::endl;
            return -1;
        }

        std::vector<std::vector<double>> center_v;
        std::vector<std::vector<double>> approach_v;
        std::vector<std::vector<double>> binnormals_v;
        std::vector<std::vector<double>> axis_v;

        std::cout << grasp_path << endl;
        if (!readGrasp(grasp_path, center_v, approach_v, binnormals_v, axis_v))
            return -1;

        // 画出grasp
        plotGrasp(center_v, approach_v, binnormals_v, axis_v, cloud, params_manager);

        // return -1;
    }
}

int main(int argc, char *argv[])
{
    DoMain(argc, argv);

    cout << "end" << endl;

    return 0;
}

// 字符分割
void split(const std::string &s,
           std::vector<std::string> &sv,
           const char delim = ' ')
{
    sv.clear();
    std::istringstream iss(s);
    std::string temp;

    while (std::getline(iss, temp, delim))
    {
        sv.emplace_back(std::move(temp));
    }

    return;
}

bool readGrasp(const string file_name,
               vector<vector<double>> &center,
               vector<vector<double>> &approach,
               vector<vector<double>> &binnormal,
               vector<vector<double>> &axis)
{
    std::ifstream readFile(file_name);
    std::string s;
    std::vector<std::string> sv;

    int count = 0;
    while (std::getline(readFile, s))
    {
        split(s, sv);
        if (sv.size() != 13)
        {
            readFile.close();
            cout << "格式错误!!!\n";
            return false;
        }

        vector<double> tmp1{std::stof(sv[0]), std::stof(sv[1]), std::stof(sv[2])};
        vector<double> tmp2{std::stof(sv[3]), std::stof(sv[4]), std::stof(sv[5])};
        vector<double> tmp3{std::stof(sv[6]), std::stof(sv[7]), std::stof(sv[8])};
        vector<double> tmp4{std::stof(sv[9]), std::stof(sv[10]), std::stof(sv[11])};
        double score = std::stof(sv[12]);

        if (score > 0.4 && count <= 200) //&& count <= 50
        {
            count++;
            center.push_back(tmp1);
            approach.push_back(tmp2);
            binnormal.push_back(tmp3);
            axis.push_back(tmp4);
        }
    }

    readFile.close();
    return true;
}

void plotGrasp(const vector<vector<double>> &center,
               const vector<vector<double>> &approach,
               const vector<vector<double>> &binnormal,
               const vector<vector<double>> &axis,
               grasp_detect::util::Cloud cloud,
               grasp_detect::util::ParamsManager param)
{
    // Create plotter.
    grasp_detect::util::Plot plotter(param);
    const auto &plot_params = param.getPlotParams();

    Eigen::Matrix3Xd pts(3, center.size());
    for (int i = 0; i < center.size(); i++)
    {
        pts.col(i) << center[i][0], center[i][1], center[i][2];
    }

    Eigen::Matrix3Xd normals(3, approach.size());
    for (int i = 0; i < approach.size(); i++)
    {
        normals.col(i) << approach[i][0] * 2, approach[i][1] * 2, approach[i][2] * 2;
    }

    plotter.plotCloud(cloud.getCloudOriginal(), "cloud");

    if (plot_params.is_plot_normals)
        plotter.plotNormals(pts, normals);

    if (plot_params.is_plots_local_axes)
        // 画出grasp坐标系
        plotter.plotLocalAxes(center, approach, binnormal, axis, cloud.getCloudOriginal());

    std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> hand_list(center.size());
    for (int i = 0; i < center.size(); i++)
    {

        Eigen::Vector3d grasp_center(center.at(i).at(0), center.at(i).at(1), center.at(i).at(2));

        Eigen::Vector3d grasp_approach(approach.at(i).at(0), approach.at(i).at(1), approach.at(i).at(2));
        Eigen::Vector3d grasp_axis(axis.at(i).at(0), axis.at(i).at(1), axis.at(i).at(2));

        Eigen::Matrix3d frame = computeRotationMatrixFrame(grasp_approach, grasp_axis);

        grasp_detect::candidate::FingerHand finger_hand;
        hand_list.at(i) = std::make_unique<grasp_detect::candidate::Hand>(grasp_center, frame, finger_hand, param.getHandGeometryParams().outer_diameter_);
    }

    if (plot_params.is_plot_candidates)
        plotter.plotFingers3D(hand_list, cloud.getCloudOriginal(), "test", param.getHandGeometryParams());
}