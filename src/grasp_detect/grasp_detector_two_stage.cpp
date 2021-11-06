#include "grasp_detect/grasp_detector_two_stage.h"
#include "grasp_detect/util/grasp.h"

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cstdlib>

namespace grasp_detect
{
    GraspDetectorTwoStage::GraspDetectorTwoStage(std::shared_ptr<grasp_detect::util::ParamsManager> params_manager,
                                                 grasp_detect::PointNetGraspClient &client,
                                                 grasp_detect::PointNetClassifyClient &classify_client)
        : params_manager_(params_manager),
          grasp_predict_client_(client),
          classify_client_(classify_client)
    {
        Eigen::initParallel();

        // 过滤器初始化
        const auto &filter_params = params_manager_->getGraspFilterParams();
        const auto &hand_params = params_manager_->getHandGeometryParams();
        grasp_filter_ = std::make_unique<GraspFilter>(filter_params, hand_params);

        const auto &cluster_params = params_manager_->getGraspClusterParams();
        cluster_ = std::make_unique<GraspCluster>(cluster_params);

        num_selected_ = params_manager_->getSelectGraspNum();

        // kdtree 查询半径
        const auto &hand_geom_params = params_manager_->getHandGeometryParams();
        Eigen::Vector3d hand_dims;
        hand_dims << hand_geom_params.outer_diameter_ / 2 + hand_geom_params.finger_width_,
            hand_geom_params.depth_ / 2 + hand_geom_params.finger_width_,
            hand_geom_params.height_;
        nn_radius_ = hand_dims.maxCoeff();

        // Create plotter.
        const auto &hand_search_params = params_manager_->getHandSearchParams();
        plotter_ = std::make_unique<util::Plot>(hand_search_params.hand_axes_.size(),
                                                hand_search_params.num_orientations_);
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorTwoStage::detectGrasps(util::Cloud &cloud)
    {
        double t0_total = omp_get_wtime();
        const auto &plot_params = params_manager_->getPlotParams();
        const auto &hand_geom_params = params_manager_->getHandGeometryParams();

        // 1. samples pointnet input points
        {
            cloud.subsample(params_manager_->getGraspPredictParams().points_num);

            if (plot_params.is_plot_samples)
            {
                if (cloud.getSamples().cols() > 0)
                    plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
                else if (cloud.getSampleIndices().size() > 0)
                    plotter_->plotSamples(cloud.getSampleIndices(),
                                          cloud.getCloudProcessed());
            }
        }

        // 2.predict grasp candidates
        std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> hand_list;
        {
            cout << "[info]"
                 << "start predict...." << endl;
            hand_list = predictGraspCandidates(cloud);
            if (hand_list.size() == 0)
                return hand_list;
            if (plot_params.is_plot_candidates)
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(), "candidates", params_manager_->getHandGeometryParams());
        }

        // 3. Filter the candidates.
        {
            double t0_filter = omp_get_wtime();
            cout << "[info]"
                 << "start Filter...." << endl;

            hand_list = grasp_filter_->filterGraspsWorkspace(hand_list);

            if (plot_params.is_plot_filtered_candidates)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                        "Filtered Grasps (Aperture, Workspace)", hand_geom_params);
            }
            if (hand_list.size() == 0)
                return hand_list;

            hand_list = grasp_filter_->filterGraspsDirection(hand_list);
            if (plot_params.is_plot_filtered_candidates)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                        "Filtered Grasps (Approach)", hand_geom_params);
            }
            std::cout << "[GraspDetectorTwoStage] fliter used time is " << omp_get_wtime() - t0_filter << std::endl;
            if (hand_list.size() == 0)
            {
                return hand_list;
            }
        }

        // 4. Cluster the grasps.
        {
            double t0_cluster = omp_get_wtime();
            const auto &cluster_params = params_manager_->getGraspClusterParams();
            if (cluster_params.is_clustering)
            {
                hand_list = cluster_->findClusters(hand_list);
                std::cout << "[GraspDetectorTwoStage] found " << (int)hand_list.size() << " clusters" << std::endl;

                if (plot_params.is_plot_clustered_grasps)
                {
                    // const auto &hand_geom_params = params_manager_->getHandGeometryParams();
                    plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                            "Clustered Grasps", hand_geom_params);
                }
            }
            std::cout << "[GraspDetectorTwoStage] clusters used time is " << omp_get_wtime() - t0_cluster << std::endl;
        }

        // 5.第二次精确评分
        {
            double t0_reclassify = omp_get_wtime();
            hand_list = rescoreGrasps(cloud, hand_list, 32);
            std::cout << "[GraspDetectorTwoStage] rescoreGrasps used total time is " << omp_get_wtime() - t0_reclassify << std::endl;
        }

        // 6. Select the <num_selected> highest scoring grasps.
        {
            hand_list = selectGrasps(hand_list);

            if (plot_params.is_plot_selected_grasps)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(), "selected", params_manager_->getHandGeometryParams());
            }
        }

        std::cout << "[GraspDetectorTwoStage] GraspDetectorTwoStage used total time is " << omp_get_wtime() - t0_total << std::endl;

        return hand_list;
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorTwoStage::rescoreGrasps(util::Cloud &cloud,
                                                                                       std::vector<std::unique_ptr<candidate::Hand>> &hands,
                                                                                       int batch_size)
    {
        std::vector<std::unique_ptr<candidate::Hand>> output_handlist;
        // Create KdTree for neighborhood search.
        const auto &cloud_pcl = cloud.getCloudProcessed();
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
        kdtree.setInputCloud(cloud_pcl);

        const Eigen::Matrix3Xd points = cloud_pcl->getMatrixXfMap().block(0, 0, 3, cloud_pcl->size()).cast<double>();

        int count = 0;
        const int pointnet_input_num = 512;
        Eigen::Matrix3Xd input_points(3, pointnet_input_num * batch_size);
        std::vector<int> input_index(batch_size);

        for (int i = 0; i < hands.size(); i++)
        {
            // extract points in grasp
            auto crop_points = cropPointsInGrasps(points, hands.at(i), kdtree);
            // std::cout << "crop_points" << crop_points.cols() << endl;

            // if (crop_points.cols() == 0)
            // {
            //     std::vector<std::unique_ptr<candidate::Hand>> hands_test;
            //     hands_test.push_back(std::move(hands.at(i)));
            //     plotter_->plotFingers3D(hands_test, cloud.getCloudProcessed(), "test", params_manager_->getHandGeometryParams());
            // }
            // std::cout << "[GraspDetectorTwoStage] rescore handlist size is " << crop_points.cols() << std::endl;
            if (crop_points.cols() < pointnet_input_num / 3)
                continue;

            // 采样至固定数量点
            auto sample_indices = fixedSampling(crop_points.cols(), pointnet_input_num);

            for (int j = 0; j < pointnet_input_num; j++)
            {
                input_points.col(count * pointnet_input_num + j) = crop_points.col(sample_indices[j]);
            }
            input_index.at(count) = i;
            count++;

            if (count == batch_size || i == (hands.size() - 1))
            {
                std::vector<double> re_score;
                if (!classify_client_.callService(input_points.block(0, 0, 3, pointnet_input_num * count), count, re_score))
                {
                    ROS_ERROR("call grasp classify server is failed");
                    count = 0;
                    continue;
                }
                for (int j = 0; j < count; j++)
                {
                    hands.at(input_index.at(j))->setScore(re_score.at(j));
                    output_handlist.push_back(std::move(hands.at(input_index.at(j))));
                }
                count = 0;
            }
        }
        if (output_handlist.size() == 0)
            output_handlist = std::move(hands);

        std::cout << "[GraspDetectorTwoStage] rescore handlist size is " << output_handlist.size() << std::endl;
        return output_handlist;
    }

    Eigen::Matrix3Xd GraspDetectorTwoStage::cropPointsInGrasps(const Eigen::Matrix3Xd &points,
                                                               const std::unique_ptr<candidate::Hand> &hand,
                                                               pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const
    {
        const auto &hamd_frame = hand->getFrame();
        const auto &hand_geom_params = params_manager_->getHandGeometryParams();

        auto hand_sample = hand->getSample();
        Eigen::Matrix3Xd points_centered = points;
        points_centered.colwise() -= hand_sample;
        points_centered = hamd_frame.transpose() * points_centered;

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;

        pcl::PointXYZRGBA sample;
        // appraoch 方向前进　hand_geom_params.depth_ / 2　距离作为查询中心
        const auto &hand_approach = hand->getApproach();
        hand_sample = hand_sample + hand_approach * hand_geom_params.depth_ / 2;
        sample.x = hand_sample(0);
        sample.y = hand_sample(1);
        sample.z = hand_sample(2);

        Eigen::Matrix3Xd output(3, 0);
        if (kdtree.radiusSearch(sample, nn_radius_, nn_indices, nn_dists) > 0)
        {
            std::vector<int> crop_indexs;
            crop_indexs.reserve(nn_indices.size());
            for (int i : nn_indices)
            {
                const auto &point = points_centered.col(i);
                if (point(0) >= -0 && point(0) <= hand_geom_params.depth_ &&
                    point(1) >= -(hand_geom_params.outer_diameter_) / 2. && point(1) <= (hand_geom_params.outer_diameter_) / 2. &&
                    point(2) <= hand_geom_params.height_ && point(2) >= -hand_geom_params.height_)
                {
                    if (point(1) >= -(hand_geom_params.outer_diameter_ - 2 * hand_geom_params.finger_width_) / 2. &&
                        point(1) <= (hand_geom_params.outer_diameter_ - 2 * hand_geom_params.finger_width_) / 2.)
                        crop_indexs.push_back(i);
                    else if (point(2) >= -hand_geom_params.height_ / 2. && point(2) <= hand_geom_params.height_ / 2.)
                    { //碰撞
                        // std::cout << "碰撞" << std::endl;
                        return output;
                    }
                    // crop_indexs.push_back(i);
                }
            }
            output.resize(3, crop_indexs.size());
            for (int i = 0; i < crop_indexs.size(); i++)
            {
                output.col(i) << points_centered.col(crop_indexs.at(i));
            }
        }
        // plotter_->plotSamples(output, cloud.getCloudProcessed());

        return output;
    }

    std::vector<int> GraspDetectorTwoStage::fixedSampling(int input_num, int output_num) const
    {
        // 下采样至固定数量点
        std::vector<int> sample_indices(input_num);
        std::iota(sample_indices.begin(), sample_indices.end(), 0);
        std::random_shuffle(sample_indices.begin(), sample_indices.end());

        std::vector<int> result(output_num);
        for (int i = 0; i < output_num; i++)
        {
            result.at(i) = sample_indices.at(i % sample_indices.size());
        }

        return result;
    }

    void GraspDetectorTwoStage::preprocessPointCloud(util::Cloud &cloud)
    {
        printf("[CandidatesGenerator] Processing cloud with %zu points.\n", cloud.getCloudOriginal()->size());
        const auto &preprocess_cloud_params = params_manager_->getPreprocessCloudParams();

        if (preprocess_cloud_params.is_random_sample)
            cloud.randomSample(preprocess_cloud_params.num_random_sample);

        cloud.removeNans();

        cloud.filterWorkspace(preprocess_cloud_params.workspace);
        if (cloud.getCloudProcessed()->size() == 0)
        {
            ROS_WARN("filterWorkspace cloud size is 0");
            return;
        }

        if (preprocess_cloud_params.is_voxelize)
        {
            cloud.voxelizeCloud(preprocess_cloud_params.voxel_size);
        }

        if (preprocess_cloud_params.is_sample_above_plane)
        {
            cloud.sampleAbovePlane();
        }
    }

    void GraspDetectorTwoStage::saveSamplePoint(const util::Cloud &cloud, const std::string file_path)
    {
        std::cout << "[GraspDetectorTwoStage] save samples point to " << file_path << std::endl;

        ofstream OutFile(file_path);

        auto indexs = cloud.getSampleIndices();
        auto point_cloud = cloud.getCloudProcessed();
        for (int i = 0; i < indexs.size(); i++)
        {
            // std::cout <<".."<<std::endl;
            OutFile << setprecision(4)
                    << point_cloud->points[indexs[i]].x << " "
                    << point_cloud->points[indexs[i]].y << " "
                    << point_cloud->points[indexs[i]].z << std::endl;
        }

        OutFile.close();
    }

    void GraspDetectorTwoStage::getSamplePointFromIndex(const util::Cloud &cloud, std::vector<std::vector<double>> &out_points)
    {
        const std::vector<int> &indexs = cloud.getSampleIndices();
        auto &point_cloud = cloud.getCloudProcessed();

        out_points.resize(indexs.size());
        for (int i = 0; i < indexs.size(); i++)
        {
            out_points[i].resize(3);
            out_points[i][0] = point_cloud->points[indexs[i]].x;
            out_points[i][1] = point_cloud->points[indexs[i]].y;
            out_points[i][2] = point_cloud->points[indexs[i]].z;
        }
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorTwoStage::graspToHandList(const grasp_detect::util::Grasp &grasps)
    {
        int grasp_num = grasps.score.size();
        std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> hand_list;
        for (int i = 0; i < grasp_num; i++)
        {
            Eigen::Vector3d grasp_center(grasps.center.at(i).at(0), grasps.center.at(i).at(1), grasps.center.at(i).at(2));
            Eigen::Vector3d grasp_approach(grasps.approach.at(i).at(0), grasps.approach.at(i).at(1), grasps.approach.at(i).at(2));
            Eigen::Vector3d grasp_closure(grasps.closure.at(i).at(0), grasps.closure.at(i).at(1), grasps.closure.at(i).at(2));
            Eigen::Vector3d grasp_vertical(grasps.vertical.at(i).at(0), grasps.vertical.at(i).at(1), grasps.vertical.at(i).at(2));

            Eigen::Matrix3d frame(3, 3);
            frame.col(0) << grasp_approach(0), grasp_approach(1), grasp_approach(2);
            frame.col(1) << grasp_closure(0), grasp_closure(1), grasp_closure(2);
            frame.col(2) << grasp_vertical(0), grasp_vertical(1), grasp_vertical(2);

            grasp_detect::candidate::FingerHand finger_hand;
            hand_list.push_back(std::make_unique<grasp_detect::candidate::Hand>(grasp_center, frame, finger_hand, 0.05));
            hand_list[i]->setScore(grasps.score[i]);
        }

        return hand_list;
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorTwoStage::selectGrasps(
        std::vector<std::unique_ptr<candidate::Hand>> &hands) const
    {
        int middle = std::min((int)hands.size(), num_selected_);
        std::partial_sort(hands.begin(), hands.begin() + middle, hands.end(),
                          isScoreGreater);
        std::vector<std::unique_ptr<candidate::Hand>> hands_out;

        for (int i = 0; i < middle; i++)
        {
            hands_out.push_back(std::move(hands[i]));
            // printf(" grasp #%d, score: %3.4f\n", i, hands_out[i]->getScore());
        }

        std::cout << "[GraspDetectorTwoStage] Selecting the " << hands_out.size() << " highest scoring grasps ...\n";

        // Sort grasps by their score.
        std::sort(hands_out.begin(), hands_out.end(), isScoreGreater);
        std::cout << "[GraspDetectorOneStage] sort the grasps." << std::endl;

        return hands_out;
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorTwoStage::predictGraspCandidates(const util::Cloud &cloud)
    {
        std::vector<std::unique_ptr<candidate::Hand>> hand_list_out;
        std::vector<std::vector<double>> points;
        getSamplePointFromIndex(cloud, points);
        if (points.size() == 0)
        {
            ROS_WARN("points.size() == 0");
            return hand_list_out;
        }

        grasp_detect::util::Grasp grasps;
        const auto &grasp_predict_params = params_manager_->getGraspPredictParams();

        bool is_ok = grasp_predict_client_.callService(points, grasps,
                                                       grasp_predict_params.min_score,
                                                       grasp_predict_params.min_grasp_num,
                                                       grasp_predict_params.max_grasp_num);
        if (!is_ok)
        {
            std::cout << "[GraspDetectorTwoStage] server is filed" << std::endl;
            return std::vector<std::unique_ptr<candidate::Hand>>(0);
        }

        return graspToHandList(grasps);
    }

    void GraspDetectorTwoStage::printStdVector(const std::vector<int> &v,
                                               const std::string &name) const
    {
        printf("%s: ", name.c_str());
        for (int i = 0; i < v.size(); i++)
        {
            printf("%d ", v[i]);
        }
        printf("\n");
    }

    void GraspDetectorTwoStage::printStdVector(const std::vector<double> &v,
                                               const std::string &name) const
    {
        printf("%s: ", name.c_str());
        for (int i = 0; i < v.size(); i++)
        {
            printf("%3.2f ", v[i]);
        }
        printf("\n");
    }

    grasp_detect::util::PointCloudRGB::Ptr GraspDetectorTwoStage::cloudFromMatrix3Xd(Eigen::Matrix3Xd &points)
    {
        grasp_detect::util::PointCloudRGB::Ptr output(new grasp_detect::util::PointCloudRGB);

        pcl::PointXYZRGBA point;
        for (int i = 0; i < points.cols(); i++)
        {
            point.x = points(0, i);
            point.y = points(1, i);
            point.z = points(2, i);

            output->points.push_back(point);
        }

        return output;
    }

} // namespace grasp_detect
