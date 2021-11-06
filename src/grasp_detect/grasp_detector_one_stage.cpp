#include "grasp_detect/grasp_detector_one_stage.h"
#include "grasp_detect/util/grasp.h"

#include <cstdlib>

namespace grasp_detect
{
    GraspDetectorOneStage::GraspDetectorOneStage(std::shared_ptr<grasp_detect::util::ParamsManager> params_manager,
                                                 grasp_detect::PointNetGraspClient &client)
        : params_manager_(params_manager), client_(client)
    {
        Eigen::initParallel();

        // 过滤器初始化
        const auto &filter_params = params_manager_->getGraspFilterParams();
        const auto &hand_params = params_manager_->getHandGeometryParams();
        filter_ = std::make_unique<GraspFilter>(filter_params, hand_params);

        // 聚类初始化
        const auto &cluster_params = params_manager_->getGraspClusterParams();
        cluster_ = std::make_unique<GraspCluster>(cluster_params);

        num_selected_ = params_manager_->getSelectGraspNum();

        // Create plotter.
        const auto &hand_search_params = params_manager_->getHandSearchParams();
        plotter_ = std::make_unique<util::Plot>(hand_search_params.hand_axes_.size(),
                                                hand_search_params.num_orientations_);
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorOneStage::detectGrasps(util::Cloud &cloud)
    {
        double t0_total = omp_get_wtime();
        const auto &plot_params = params_manager_->getPlotParams();
        const auto &hand_geom_params = params_manager_->getHandGeometryParams();

        // 1. samples pointnet input points
        cloud.subsample(params_manager_->getGraspPredictParams().points_num);

        if (plot_params.is_plot_samples)
        {
            if (cloud.getSamples().cols() > 0)
                plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
            else if (cloud.getSampleIndices().size() > 0)
                plotter_->plotSamples(cloud.getSampleIndices(),
                                      cloud.getCloudProcessed());
        }

        // 2.predict grasp candidates
        std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> hand_list;
        {
            hand_list = predictGraspCandidates(cloud);
            if (hand_list.size() == 0)
            {
                std::cout << "[Warning] predict grasp candidates num is 0 " << std::endl;
                return hand_list;
            }
            if (plot_params.is_plot_candidates)
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(), "candidates", params_manager_->getHandGeometryParams());
        }

        // 3. Filter the candidates.
        {
            double t0_filter = omp_get_wtime();

            hand_list = filter_->filterGraspsWorkspace(hand_list);

            if (hand_list.size() == 0)
            {
                return hand_list;
            }
            if (plot_params.is_plot_filtered_candidates)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                        "Filtered Grasps (Aperture, Workspace)", hand_geom_params);
            }

            hand_list = filter_->filterGraspsDirection(hand_list);
            if (plot_params.is_plot_filtered_candidates)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                        "Filtered Grasps (Approach)", hand_geom_params);
            }
        }

        // 4. Cluster the grasps.
        {
            double t0_cluster = omp_get_wtime();
            const auto &cluster_params = params_manager_->getGraspClusterParams();
            // std::vector<std::unique_ptr<candidate::Hand>> clusters;
            if (cluster_params.is_clustering)
            {
                hand_list = cluster_->findClusters(hand_list);
                std::cout << "[GraspDetectorOneStage] found " << (int)hand_list.size() << " clusters" << std::endl;

                if (plot_params.is_plot_clustered_grasps)
                {
                    const auto &hand_geom_params = params_manager_->getHandGeometryParams();
                    plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(),
                                            "Clustered Grasps", hand_geom_params);
                }
            }

            std::cout << "[GraspDetectorOneStage] clusters used time is " << omp_get_wtime() - t0_cluster << std::endl;
        }

        // 5. Select the <num_selected> highest scoring grasps.
        {
            hand_list = selectGrasps(hand_list);

            if (plot_params.is_plot_selected_grasps)
            {
                plotter_->plotFingers3D(hand_list, cloud.getCloudOriginal(), "selected", params_manager_->getHandGeometryParams());
            }
        }

        std::cout << "[GraspDetectorOneStage] GraspDetectorOneStage used total time is " << omp_get_wtime() - t0_total << std::endl;

        return hand_list;
    }

    void GraspDetectorOneStage::preprocessPointCloud(util::Cloud &cloud)
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

    void GraspDetectorOneStage::saveSamplePoint(const util::Cloud &cloud, const std::string file_path)
    {
        std::cout << "[GraspDetectorOneStage] save samples point to " << file_path << std::endl;

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

    void GraspDetectorOneStage::getSamplePointFromIndex(const util::Cloud &cloud, std::vector<std::vector<double>> &out_points)
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

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorOneStage::graspToHandList(const grasp_detect::util::Grasp &grasps)
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

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorOneStage::selectGrasps(
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

        std::cout << "[GraspDetectorOneStage] Selecting the " << hands_out.size() << " highest scoring grasps ...\n";

        // Sort grasps by their score.
        std::sort(hands_out.begin(), hands_out.end(), isScoreGreater);
        std::cout << "[GraspDetectorOneStage] sort the grasps." << std::endl;

        return hands_out;
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspDetectorOneStage::predictGraspCandidates(const util::Cloud &cloud)
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

        bool is_ok = client_.callService(points, grasps,
                                         grasp_predict_params.min_score,
                                         grasp_predict_params.min_grasp_num,
                                         grasp_predict_params.max_grasp_num);
        if (!is_ok)
        {
            ROS_WARN("[GraspDetectorOneStage] grasp server is filed");
            return std::vector<std::unique_ptr<candidate::Hand>>(0);
        }

        return graspToHandList(grasps);
    }

    void GraspDetectorOneStage::printStdVector(const std::vector<int> &v,
                                               const std::string &name) const
    {
        printf("%s: ", name.c_str());
        for (int i = 0; i < v.size(); i++)
        {
            printf("%d ", v[i]);
        }
        printf("\n");
    }

    void GraspDetectorOneStage::printStdVector(const std::vector<double> &v,
                                               const std::string &name) const
    {
        printf("%s: ", name.c_str());
        for (int i = 0; i < v.size(); i++)
        {
            printf("%3.2f ", v[i]);
        }
        printf("\n");
    }

} // namespace grasp_detect
