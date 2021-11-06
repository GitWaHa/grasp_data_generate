#include <string>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "grasp_detect/util/plot.h"
#include "grasp_detect/util/transform.h"

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

#include <unistd.h>
#include <getopt.h>

using namespace std;
namespace fs = boost::filesystem;

bool readGrasp(const string file_name,
               vector<vector<double>> &center,
               vector<vector<double>> &approach,
               vector<vector<double>> &binnormal,
               vector<vector<double>> &axis,
               vector<double> &score);

void plotGrasp(const vector<vector<double>> &center,
               const vector<vector<double>> &approach,
               const vector<vector<double>> &binnormal,
               const vector<vector<double>> &axis,
               const vector<double> &score,
               grasp_detect::util::Cloud cloud,
               grasp_detect::util::ParamsManager param);

int parseCmd(int argc, char *argv[], std::map<string, string> &args)
{
    int o;
    int option_index = 0;
    const char *optstring = "d:c:s:"; // 有三个选项-abc，其中c选项后有冒号，所以后面必须有参数
    static struct option long_options[] = {
        {"data_dir", required_argument, NULL, 'd'},
        {"cfg_dir", required_argument, NULL, 'c'},
        {"strat_step", required_argument, NULL, 's'},
    };
    while ((o = getopt_long(argc, argv, optstring, long_options, &option_index)) != -1)
    {
        switch (o)
        {
        case 'd':
            args.at("data_dir") = optarg;
            printf("opt is data_dir, oprarg is: %s\n", optarg);
            break;
        case 'c':
            args.at("cfg_dir") = optarg;
            printf("opt is cfg_dir, oprarg is: %s\n", optarg);
            break;
        case 's':
            args.at("strat_step") = optarg;
            printf("opt is strat_step, oprarg is: %s\n", optarg);
            break;
        case '?':
            printf("error optopt: %c\n", optopt);
            printf("error opterr: %d\n", opterr);
            break;
        }
    }
    return 0;
}

int DoMain(int argc, char *argv[])
{
    std::map<string, string> args;
    args.insert(std::make_pair("data_dir", "/home/waha/catkin_research_ws/src/data/"));
    args.insert(std::make_pair("cfg_dir", "/home/waha/catkin_research_ws/src/grasp_detect/config/generate_parameter.cfg"));
    args.insert(std::make_pair("strat_step", "0"));
    parseCmd(argc, argv, args);

    int start_step = std::stoi(args.at("strat_step"));
    std::string config_filename(args.at("cfg_dir"));

    grasp_detect::util::ParamsManager params_manager(config_filename);

    string data_root{args.at("data_dir")};
    string camera_pose_filename = data_root + "bullet/camera_pose/camera_pose.txt";
    string pcd_folder = data_root + "bullet/pointcloud/";
    string save_path = data_root + "pointnet_grasp/raw_data";

    Eigen::Matrix3Xd view_points(3, 4);
    auto camera_pos = params_manager.getPreprocessCloudParams().camera_position;
    view_points.col(0) << camera_pos[0], camera_pos[1], camera_pos[2];
    view_points.col(1) << camera_pos[3], camera_pos[4], camera_pos[5];
    view_points.col(2) << camera_pos[6], camera_pos[7], camera_pos[8];
    view_points.col(3) << camera_pos[9], camera_pos[10], camera_pos[11];

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
        // if (split_vec.at(split_vec.size() - 2) != "00")
        //     continue;

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

        string str = "sim_" + step_vec.at(ids) + "_" + pose_vec.at(ids);
        string save_grasp_path = save_path + "/" + str + ".txt";

        std::cout << "[info] read grasp file " << save_grasp_path << std::endl;

        std::vector<std::vector<double>> center_v;
        std::vector<std::vector<double>> approach_v;
        std::vector<std::vector<double>> binnormals_v;
        std::vector<std::vector<double>> axis_v;
        std::vector<double> score;

        if (!readGrasp(save_grasp_path, center_v, approach_v, binnormals_v, axis_v, score))
            return -1;

        // 画出grasp
        plotGrasp(center_v, approach_v, binnormals_v, axis_v, score, cloud, params_manager);
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
               vector<vector<double>> &axis,
               vector<double> &score)
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
        center.push_back(tmp1);
        approach.push_back(tmp2);
        binnormal.push_back(tmp3);
        axis.push_back(tmp4);
        score.push_back(std::stof(sv[12]));
    }

    readFile.close();
    return true;
}

void plotGrasp(const vector<vector<double>> &center,
               const vector<vector<double>> &approach,
               const vector<vector<double>> &binnormal,
               const vector<vector<double>> &axis,
               const vector<double> &score,
               grasp_detect::util::Cloud cloud,
               grasp_detect::util::ParamsManager param)
{
    // Create plotter.
    grasp_detect::util::Plot plotter(param);
    const auto &plot_params = param.getPlotParams();

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
        hand_list.at(i)->setScore(score.at(i));
    }

    if (plot_params.is_plot_candidates)
        plotter.plotFingers3D(hand_list, cloud.getCloudOriginal(), "fingers", param.getHandGeometryParams());

    if (param.getPlotParams().is_plots_antipodal_grasps)
        plotter.plotAntipodalHands(hand_list, cloud.getCloudProcessed(), "Antipodal fingers",
                                   param.getHandGeometryParams(),
                                   param.getGraspPredictParams().min_score);
}