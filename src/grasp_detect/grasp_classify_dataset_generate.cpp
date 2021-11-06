#include "grasp_detect/grasp_classify_dataset_generate.h"

namespace grasp_detect
{
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

    GraspClassifyDataGenerate::GraspClassifyDataGenerate(ParamsManager &params) : params_(params),
                                                                                  plotter_(params)
    {
        candidates_generator_ = std::make_unique<grasp_detect::candidate::CandidatesGenerator>(params_);
    }

    GraspClassifyDataGenerate::~GraspClassifyDataGenerate()
    {
    }

    void GraspClassifyDataGenerate::preprocessCloud(grasp_detect::util::Cloud &cloud)
    {
        candidates_generator_->preprocessPointCloud(cloud);
    }

    void GraspClassifyDataGenerate::save(grasp_detect::util::Cloud &cloud, const std::string read_file, const std::string write_file)
    {
        grasp_detect::util::Grasp grasps;
        if (!readGrasp(read_file, grasps))
            return;

        auto sort_index = sortGrasp(grasps.score);

        saveClassifyDateset(cloud, grasps, sort_index, 50, 512, write_file);
    }

    bool GraspClassifyDataGenerate::readGrasp(const std::string file_name, grasp_detect::util::Grasp &grasps)
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
            if (std::stof(sv[12]) > 0.01) //过滤无效的grasp
            {
                std::vector<double> tmp1{std::stof(sv[0]), std::stof(sv[1]), std::stof(sv[2])};
                std::vector<double> tmp2{std::stof(sv[3]), std::stof(sv[4]), std::stof(sv[5])};
                std::vector<double> tmp3{std::stof(sv[6]), std::stof(sv[7]), std::stof(sv[8])};
                std::vector<double> tmp4{std::stof(sv[9]), std::stof(sv[10]), std::stof(sv[11])};

                grasps.center.push_back(tmp1);
                grasps.approach.push_back(tmp2);
                grasps.closure.push_back(tmp3);
                grasps.vertical.push_back(tmp4);
                grasps.score.push_back(std::stof(sv[12]));
            }
        }

        readFile.close();
        return true;
    }

    void GraspClassifyDataGenerate::saveClassifyDateset(grasp_detect::util::Cloud &cloud,
                                                        const grasp_detect::util::Grasp &grasps,
                                                        const std::vector<std::size_t> &sort_index,
                                                        int grasp_num,
                                                        int points_num,
                                                        const std::string save_path)
    {
        const auto &plot_params = params_.getPlotParams();
        // Create KdTree for neighborhood search.
        const auto &cloud_pcl = cloud.getCloudProcessed();
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
        kdtree.setInputCloud(cloud_pcl);

        const Eigen::Matrix3Xd points = cloud_pcl->getMatrixXfMap().block(0, 0, 3, cloud_pcl->size()).cast<double>();

        int count = 0;
        const int pointnet_input_num = points_num;
        const int gap_num = sort_index.size() / grasp_num;

        for (int i = 0; i < sort_index.size(); i += gap_num)
        {
            Eigen::Matrix3d hand_frame(3, 3);
            hand_frame.col(0) << grasps.approach[sort_index[i]][0], grasps.approach[sort_index[i]][1], grasps.approach[sort_index[i]][2];
            hand_frame.col(1) << grasps.closure[sort_index[i]][0], grasps.closure[sort_index[i]][1], grasps.closure[sort_index[i]][2];
            hand_frame.col(2) << grasps.vertical[sort_index[i]][0], grasps.vertical[sort_index[i]][1], grasps.vertical[sort_index[i]][2];

            // extract points in grasp
            auto crop_points = cropPointsInGrasps(points, grasps.center.at(sort_index.at(i)), hand_frame, kdtree);
            // std::cout << grasps.score[sort_index[i]] << std::endl;
            // std::cout << "crop_points" << crop_points.cols() << endl;
            if (crop_points.cols() < pointnet_input_num)
                continue;

            // 画出grasp
            // 画出grasp坐标系
            if (plot_params.is_plots_local_axes)
                plotter_.plotLocalAxes(std::vector<std::vector<double>>{grasps.center.at(sort_index[i])},
                                       std::vector<std::vector<double>>{grasps.approach.at(sort_index[i])},
                                       std::vector<std::vector<double>>{grasps.closure.at(sort_index[i])},
                                       std::vector<std::vector<double>>{grasps.vertical.at(sort_index[i])}, cloud.getCloudOriginal());

            if (plot_params.is_plots_local_axes)
                plotter_.plotSamples(crop_points, cloud.getCloudProcessed());

            // 下采样至固定数量点
            std::vector<int> sample_indices(crop_points.cols());
            std::iota(sample_indices.begin(), sample_indices.end(), 0);
            std::random_shuffle(sample_indices.begin(), sample_indices.end());

            // 保存网络输入point 和　label分数
            std::string new_path = rename(save_path, count);
            // std::cout << new_path << std::endl;
            ofstream OutFile(new_path);
            OutFile << grasps.score[sort_index[i]] << std::endl;

            for (int j = 0; j < pointnet_input_num; j++)
            {
                auto point = crop_points.col(sample_indices[j]);
                OutFile << point.x() << " " << point.y() << " " << point.z() << std::endl;
            }
            count++;
            OutFile.close();
        }
    }

    std::string GraspClassifyDataGenerate::rename(const std::string file_name, int n)
    {
        std::vector<std::string> split_vec;
        boost::split(split_vec, file_name, boost::is_any_of("/."));

        std::string result;
        for (int i = 1; i < split_vec.size() - 1; i++)
        {
            result = result + "/" + split_vec.at(i);
        }
        std::string str = std::to_string(n);
        while (str.size() < 3)
        {
            str = "0" + str;
        }
        result = result + "_" + str + "." + split_vec.back();

        return result;
    }

    std::vector<size_t> GraspClassifyDataGenerate::sortGrasp(const std::vector<double> &score)
    {
        // initialize original index locations
        std::vector<size_t> idx(score.size());
        iota(idx.begin(), idx.end(), 0);

        // sort indexes based on comparing values in v
        std::sort(idx.begin(), idx.end(),
                  [&score](size_t i1, size_t i2)
                  { return score[i1] < score[i2]; });

        return idx;
    }

    Eigen::Matrix3Xd GraspClassifyDataGenerate::cropPointsInGrasps(const Eigen::Matrix3Xd &points,
                                                                   const std::vector<double> &center,
                                                                   const Eigen::Matrix3d &hamd_frame,
                                                                   pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree)
    {
        Eigen::Matrix3Xd points_centered = points;
        Eigen::Vector3d hand_sample(center[0], center[1], center[2]);
        points_centered.colwise() -= hand_sample;
        points_centered = hamd_frame.transpose() * points_centered;

        const auto &hand_geom_params = params_.getHandGeometryParams();
        double nn_radius = std::max(hand_geom_params.outer_diameter_, hand_geom_params.depth_);

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;

        pcl::PointXYZRGBA sample;
        sample.x = hand_sample(0);
        sample.y = hand_sample(1);
        sample.z = hand_sample(2);

        Eigen::Matrix3Xd output(3, 0);
        if (kdtree.radiusSearch(sample, nn_radius, nn_indices, nn_dists) > 0)
        {
            std::vector<int> crop_indexs;
            crop_indexs.reserve(nn_indices.size());
            for (int i : nn_indices)
            {
                const auto &point = points_centered.col(i);
                if (point(0) >= -0 && point(0) <= hand_geom_params.depth_ &&
                    point(1) >= -(hand_geom_params.outer_diameter_ - 2 * hand_geom_params.finger_width_) / 2. && (hand_geom_params.outer_diameter_ - 2 * hand_geom_params.finger_width_) / 2. &&
                    point(2) <= hand_geom_params.height_ && point(2) >= -hand_geom_params.height_)
                {
                    crop_indexs.push_back(i);
                }
            }
            output.resize(3, crop_indexs.size());
            for (int i = 0; i < crop_indexs.size(); i++)
            {
                output.col(i) << points_centered.col(crop_indexs.at(i));
            }
        }
        return output;
    }
} // namespace grasp_detect
