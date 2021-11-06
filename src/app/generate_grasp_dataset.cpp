#include <string>
#include <iostream>
#include <fstream>

#include <unistd.h>
#include <getopt.h>

#include "grasp_detect/util/plot.h"
#include "grasp_detect/candidate/candidates_generator.h"
#include "grasp_detect/grasp_dataset_generate.h"

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>

using namespace std;
namespace fs = boost::filesystem;

int loadMultiViewCould(vector<string> pcd_files, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sum_cloud, Eigen::MatrixXi &camera_source);
Eigen::Matrix3Xd randomSamplingFromCloud(string pcd_filename, int num_samples);

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
    grasp_detect::util::ParamsManager param(config_filename);

    string data_root{args.at("data_dir")};
    string camera_pose_filename = data_root + "bullet/camera_pose/camera_pose.txt";
    string pcd_folder = data_root + "bullet/pointcloud/";
    string save_path = data_root + "pointnet_grasp/raw_data_with_width";

    if (!fs::exists(save_path))
        fs::create_directories(save_path);

    // Create plotter.
    grasp_detect::util::Plot plotter(param);

    Eigen::Matrix3Xd view_points(3, 4);
    auto camera_pos = param.getPreprocessCloudParams().camera_position;
    view_points.col(0) << camera_pos[0], camera_pos[1], camera_pos[2];
    view_points.col(1) << camera_pos[3], camera_pos[4], camera_pos[5];
    view_points.col(2) << camera_pos[6], camera_pos[7], camera_pos[8];
    view_points.col(3) << camera_pos[9], camera_pos[10], camera_pos[11];

    string path = pcd_folder;
    if (!fs::exists(path))
        return -1;

    vector<std::pair<string, string>> all_pcd_path;
    for (const auto &entry : fs::directory_iterator(path))
    {
        string pcd_path = entry.path().string();

        vector<string> split_vec;
        boost::split(split_vec, pcd_path, boost::is_any_of("/"));
        string pcd_file_name = split_vec.at(split_vec.size() - 1);
        split_vec.clear();

        boost::split(split_vec, pcd_file_name, boost::is_any_of("_."));
        if (split_vec.at(split_vec.size() - 2) != "00")
            continue;

        all_pcd_path.push_back(std::make_pair(pcd_path, split_vec.at(1)));
    }

    std::sort(all_pcd_path.begin(), all_pcd_path.end());

    for (int i = start_step; i < all_pcd_path.size(); i++)
    {
        std::cout << std::endl
                  << "******************************" << std::endl;
        auto data = all_pcd_path[i];
        string pcd_path = data.first;
        string step = data.second;
        std::cout << "[info] process pcd file group " << pcd_path << std::endl;

        // 加载多视角点云
        vector<string>
            pcd_files{pcd_folder + "sim_" + step + "_00.pcd",
                      pcd_folder + "sim_" + step + "_01.pcd",
                      pcd_folder + "sim_" + step + "_02.pcd",
                      pcd_folder + "sim_" + step + "_03.pcd"};
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGBA>);
        Eigen::MatrixXi camera_source(4, (*cloud_sum).size());
        loadMultiViewCould(pcd_files, cloud_sum, camera_source);

        grasp_detect::util::Cloud cloud(cloud_sum, camera_source, view_points);
        if (cloud.getCloudOriginal()->size() == 0)
        {
            std::cout << "ERROR: Point cloud is empty!" << std::endl;
            continue;
        }

        grasp_detect::GraspDatasetGenerate grasp_generate(param);

        // 点云预处理
        grasp_generate.preprocessCloud(cloud);
        if (param.getPlotParams().is_plot_normals)
            plotter.plotNormals(cloud, true);

        for (int i = 0; i < pcd_files.size(); i++)
        {
            // 单视角点云随机采样
            int num_samples = param.getCandidatesGeneratorParams().num_samples_;
            auto samples = randomSamplingFromCloud(pcd_files[i], num_samples);
            cloud.setSamples(samples);

            if (param.getPlotParams().is_plot_samples)
                plotter.plotSamples(cloud.getSamples(), cloud.getCloudProcessed());

            // 生成有效的候选
            auto hand_set_list = grasp_generate.generateGraspCandidateSets(cloud);

            // 保存到txt文件
            string str = "sim_" + step + "_" + "0" + std::to_string(i);
            string save_grasp_path = save_path + "/" + str + ".txt";
            string save_pcd_path = "";
            string save_point_path = save_path + "/" + str + ".xyz";

            grasp_generate.saveGrasp(hand_set_list, save_grasp_path, save_pcd_path, save_point_path, num_samples);

            if (param.getPlotParams().is_plot_candidates)
                plotter.plotFingers3D(hand_set_list, cloud.getCloudProcessed(), "fingers", param.getHandGeometryParams());

            if (param.getPlotParams().is_plots_antipodal_grasps)
                plotter.plotAntipodalHands(hand_set_list, cloud.getCloudProcessed(), "Antipodal fingers",
                                           param.getHandGeometryParams(),
                                           param.getGraspPredictParams().min_score);
        }

        std::cout << "******************************" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    DoMain(argc, argv);

    cout << "end" << endl;

    return 0;
}

int loadMultiViewCould(vector<string> pcd_files, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sum_cloud, Eigen::MatrixXi &camera_source)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGBA>);

    vector<int> cloud_size(pcd_files.size());
    for (int i = 0; i < pcd_files.size(); i++)
    {
        string pcd_file = pcd_files.at(i);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_file, *cloud0) != 0)
            return -1;

        cloud_size.at(i) = (*cloud0).size();
        cout << "[info]"
             << "load point num " << (*cloud0).size() << endl;
        (*sum_cloud) += (*cloud0);
    }
    cout << "[info]"
         << "load sum point num " << (*sum_cloud).size() << endl;

    camera_source.resize(pcd_files.size(), (*sum_cloud).size());
    for (int i = 0; i < camera_source.cols(); i++)
    {
        for (int j = cloud_size.size() - 1; j >= 0; j--)
        {
            if (i >= accumulate(cloud_size.begin(), cloud_size.begin() + j, 0))
            {
                camera_source(j, i) = 1;
                break;
            }
        }
    }

    return 0;
}

Eigen::Matrix3Xd randomSamplingFromCloud(string pcd_filename, int num_samples)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_filename, *cloud0) != 0)
        return {3, 0};

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if ((*cloud0).size() < num_samples)
    {
        std::cout << "(*cloud0).size() " << (*cloud0).size() << std::endl;
        pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA> filter;
        filter.setInputCloud(cloud0);
        //建立搜索对象
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree;
        filter.setSearchMethod(kdtree);
        //设置搜索邻域的半径为3cm
        filter.setSearchRadius(0.01);
        // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY,SAMPLE_LOCAL_PLANE
        filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::SAMPLE_LOCAL_PLANE);
        // 采样的半径是
        filter.setUpsamplingRadius(0.01);
        // 采样步数的大小
        filter.setUpsamplingStepSize(0.005);

        filter.process(*filteredCloud);

        std::cout << "(*filteredCloud).size() " << (*filteredCloud).size() << std::endl;
    }
    else
    {
        filteredCloud = cloud0;
    }

    std::vector<int> sample_indices(num_samples);
    pcl::RandomSample<pcl::PointXYZRGBA> random_sample;
    random_sample.setInputCloud(filteredCloud);
    random_sample.setSample(num_samples);
    random_sample.filter(sample_indices);

    Eigen::Matrix3Xd samples(3, num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        int index = sample_indices.at(i);
        samples.col(i) << (*filteredCloud).at(index).x, (*filteredCloud).at(index).y, (*filteredCloud).at(index).z;
    }

    return samples;
}
