#include "grasp_detect/util/params_manager.h"

namespace grasp_detect
{
    namespace util
    {
        ParamsManager::ParamsManager(std::string config_name) : cfg_file_(config_name)
        {
            cfg_file_.ExtractKeys();

            readPreprocessCloudParamsFromConfig();

            // hand_geometry
            readHandGeometryParamesFromConfig();
            // HandSearchParams
            readHandSearchParamesFromConfig();

            // CandidatesGenerator
            readCandidatesParamesFromConfig();

            //　plot
            readPlotParamesFromConfig();

            readClusteringParamesConfig();

            readFlterGraspParamesConfig();

            readGraspPredictParamesConfig();
        }

        ParamsManager::~ParamsManager()
        {
        }

        void ParamsManager::readPreprocessCloudParamsFromConfig()
        {
            preprocess_params_.is_voxelize = cfg_file_.getValueOfKey<bool>("voxelize", true);
            preprocess_params_.voxel_size = cfg_file_.getValueOfKey<float>("voxel_size", 0.003);
            preprocess_params_.is_remove_outliers = cfg_file_.getValueOfKey<bool>("remove_outliers", false);
            preprocess_params_.workspace = cfg_file_.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");
            preprocess_params_.camera_position = cfg_file_.getValueOfKeyAsStdVectorDouble("camera_position", "0 0 0");
            preprocess_params_.is_sample_above_plane = cfg_file_.getValueOfKey<bool>("sample_above_plane", false);
            preprocess_params_.is_random_sample = cfg_file_.getValueOfKey<bool>("is_random_sample", true);
            preprocess_params_.num_random_sample = cfg_file_.getValueOfKey<int>("num_random_sample", 20000);
        }

        void ParamsManager::readHandGeometryParamesFromConfig()
        {
            hand_geometry_param_.depth_ = cfg_file_.getValueOfKey("hand_depth", 0.06);
            hand_geometry_param_.finger_width_ = cfg_file_.getValueOfKey("finger_width", 0.01);
            hand_geometry_param_.height_ = cfg_file_.getValueOfKey("hand_height", 0.02);
            hand_geometry_param_.init_bite_ = cfg_file_.getValueOfKey("init_bite", 0.01);
            hand_geometry_param_.outer_diameter_ = cfg_file_.getValueOfKey("hand_outer_diameter", 0.12);
        }

        void ParamsManager::readHandSearchParamesFromConfig()
        {
            hand_search_params_.hand_geometry_ = hand_geometry_param_;

            hand_search_params_.nn_radius_frames_ =
                cfg_file_.getValueOfKey<double>("nn_radius", 0.01);
            hand_search_params_.num_samples_ =
                cfg_file_.getValueOfKey<int>("num_samples", 1000);
            hand_search_params_.num_threads_ =
                cfg_file_.getValueOfKey<int>("num_threads", 1);
            hand_search_params_.num_orientations_ =
                cfg_file_.getValueOfKey<int>("num_orientations", 8);
            hand_search_params_.num_finger_placements_ =
                cfg_file_.getValueOfKey<int>("num_finger_placements", 10);
            hand_search_params_.deepen_hand_ =
                cfg_file_.getValueOfKey<bool>("deepen_hand", true);
            hand_search_params_.hand_axes_ =
                cfg_file_.getValueOfKeyAsStdVectorInt("hand_axes", "2");
            hand_search_params_.friction_coeff_ =
                cfg_file_.getValueOfKey<double>("friction_coeff", 20.0);
            hand_search_params_.min_viable_ =
                cfg_file_.getValueOfKey<int>("min_viable", 6);
        }

        void ParamsManager::readCandidatesParamesFromConfig()
        {
            candidates_params_.num_samples_ =
                cfg_file_.getValueOfKey<int>("num_samples", 1000);
            candidates_params_.num_threads_ =
                cfg_file_.getValueOfKey<int>("num_threads", 1);
            candidates_params_.remove_statistical_outliers_ =
                cfg_file_.getValueOfKey<bool>("remove_outliers", false);
            candidates_params_.sample_above_plane_ =
                cfg_file_.getValueOfKey<bool>("sample_above_plane", false);
            candidates_params_.voxelize_ =
                cfg_file_.getValueOfKey<bool>("voxelize", true);
            candidates_params_.voxel_size_ =
                cfg_file_.getValueOfKey<double>("voxel_size", 0.003);
            candidates_params_.normals_radius_ =
                cfg_file_.getValueOfKey<double>("normals_radius", 0.03);
            candidates_params_.refine_normals_k_ =
                cfg_file_.getValueOfKey<int>("refine_normals_k", 0);
            candidates_params_.workspace_ =
                cfg_file_.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");
        }

        void ParamsManager::readPlotParamesFromConfig()
        {
            plot_param_.is_plot_samples = cfg_file_.getValueOfKey("plot_samples", false);
            plot_param_.is_plot_normals = cfg_file_.getValueOfKey("plot_normals", false);
            plot_param_.is_plot_candidates = cfg_file_.getValueOfKey("plot_candidates", false);
            plot_param_.is_plot_filtered_candidates = cfg_file_.getValueOfKey("plot_filtered_candidates", false);
            plot_param_.is_plots_local_axes = cfg_file_.getValueOfKey("plots_local_axes", false);
            plot_param_.is_plots_antipodal_grasps = cfg_file_.getValueOfKey("plots_antipodal_grasps", false);
            plot_param_.is_plot_clustered_grasps = cfg_file_.getValueOfKey("plot_clustered_grasps", false);
            plot_param_.is_plot_selected_grasps = cfg_file_.getValueOfKey("plot_selected_grasps", false);
        }

        void ParamsManager::readClusteringParamesConfig()
        {
            grasp_cluster_params_.is_clustering = cfg_file_.getValueOfKey("is_clustering", false);
            grasp_cluster_params_.min_inliers = cfg_file_.getValueOfKey<int>("min_inliers", 1);
            grasp_cluster_params_.is_remove_inliers = cfg_file_.getValueOfKey("is_remove_inliers", false);
        }

        void ParamsManager::readFlterGraspParamesConfig()
        {
            grasp_filter_params_.min_aperture = cfg_file_.getValueOfKey<double>("min_aperture", 0.0);
            grasp_filter_params_.max_aperture = cfg_file_.getValueOfKey<double>("max_aperture", 0.07);
            grasp_filter_params_.workspace_grasps = cfg_file_.getValueOfKeyAsStdVectorDouble("workspace_grasps", "-1 1 -1 1 -1 1");

            grasp_filter_params_.is_filter_direction = cfg_file_.getValueOfKey<bool>("is_filter_direction", false);
            grasp_filter_params_.direction = cfg_file_.getValueOfKeyAsStdVectorDouble("direction", "1 0 0");
            grasp_filter_params_.thresh_rad = cfg_file_.getValueOfKey<double>("thresh_rad", 2.0);
        }

        void ParamsManager::readGraspPredictParamesConfig()
        {
            grasp_predict_params_.points_num = cfg_file_.getValueOfKey<int>("points_num", 2048);
            grasp_predict_params_.min_score = cfg_file_.getValueOfKey<double>("min_score", 2048);
            grasp_predict_params_.min_grasp_num = cfg_file_.getValueOfKey<int>("min_grasp_num", 100);
            grasp_predict_params_.max_grasp_num = cfg_file_.getValueOfKey<int>("max_grasp_num", 500);
        }

    } // namespace util
} // namespace grasp_detect
