#include <string>
#include <iostream>
#include <fstream>

#include <unistd.h>
#include <getopt.h>

#include <boost/filesystem.hpp>

#include "grasp_detect/util/plot.h"
#include "grasp_detect/util/transform.h"

#include "grasp_detect/grasp_classify_dataset_generate.h"

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace std;
namespace fs = boost::filesystem;

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

    string data_root(args.at("data_dir"));
    string camera_pose_filename = data_root + "bullet/camera_pose/camera_pose.txt";
    string pcd_folder = data_root + "bullet/pointcloud/";
    string read_path = data_root + "pointnet_grasp/raw_data";
    string save_path = data_root + "pointnet_classify/raw_data";

    Eigen::Matrix3Xd view_points(3, 4);
    auto camera_pos = params_manager.getPreprocessCloudParams().camera_position;
    view_points.col(0) << camera_pos[0], camera_pos[1], camera_pos[2];
    view_points.col(1) << camera_pos[3], camera_pos[4], camera_pos[5];
    view_points.col(2) << camera_pos[6], camera_pos[7], camera_pos[8];
    view_points.col(3) << camera_pos[9], camera_pos[10], camera_pos[11];

    if (!fs::exists(save_path))
        fs::create_directories(save_path);

    vector<std::pair<string, int>> all_pcd_path;
    vector<string> step_vec;
    vector<string> pose_vec;
    int index = 0;
    string path = pcd_folder;
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

    // Create plotter.
    grasp_detect::util::Plot plotter(params_manager);

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

        std::string str = "sim_" + step_vec.at(ids) + "_" + pose_vec.at(ids);
        std::string read_grasp_path = read_path + "/" + str + ".txt";
        std::string save_classify_path = save_path + "/" + str + ".txt";

        std::cout << "[info] read grasp file " << read_grasp_path << std::endl;
        std::cout << "[info] save classify file " << save_classify_path << std::endl;

        grasp_detect::GraspClassifyDataGenerate classify_gen(params_manager);
        classify_gen.save(cloud, read_grasp_path, save_classify_path);
    }
}

int main(int argc, char *argv[])
{
    DoMain(argc, argv);

    cout << "end" << endl;

    return 0;
}
