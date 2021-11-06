#pragma once

#include <iostream>
#include <ostream>
#include "grasp_detect/util/config_file.h"

/*
    参数读取
*/
namespace grasp_detect
{
    namespace util
    {
        /**
         * \brief Params for the Preprocess cloud
         */

        struct PreprocessCloudParams
        {
            bool is_voxelize;
            float voxel_size;
            bool is_remove_outliers;
            std::vector<double> workspace;
            std::vector<double> camera_position;
            bool is_sample_above_plane;
            bool is_random_sample;
            int num_random_sample;

            friend std::ostream &operator<<(std::ostream &stream, const PreprocessCloudParams &data)
            {
                stream << "============ PreprocessCloudParams ======================\n";
                stream << "is_voxelize: " << data.is_voxelize << "\n";
                stream << "voxel_size: " << data.voxel_size << "\n";
                stream << "is_remove_outliers: " << data.is_remove_outliers << "\n";
                stream << "workspace: "
                       << "min_x:" << data.workspace[0] << ",max_x:" << data.workspace[1]
                       << ",min_y:" << data.workspace[2] << ",max_y:" << data.workspace[3]
                       << ",min_z:" << data.workspace[4] << ",max_z:" << data.workspace[5] << "\n";
                stream << "camera_position: "
                       << "x:" << data.camera_position[0] << ",y:" << data.camera_position[1]
                       << ",z:" << data.camera_position[2] << "\n";
                stream << "is_sample_above_plane: " << data.is_sample_above_plane << "\n";
                stream << "is_random_sample: " << data.is_random_sample << "\n";
                stream << "num_random_sample: " << data.num_random_sample << "\n";
                stream << "=================================================\n";
            }
        };

        /**
         * \brief Params for the Plot
         */

        struct PlotParams
        {
            bool is_plot_normals = 1;
            bool is_plot_samples = 1;
            bool is_plot_candidates = 1;
            bool is_plot_filtered_candidates = 1;
            bool is_plot_clustered_grasps = 1;
            bool is_plot_selected_grasps = 1;
            bool is_plots_local_axes;
            bool is_plots_antipodal_grasps;

            friend std::ostream &operator<<(std::ostream &stream, const PlotParams &data)
            {
                stream << "============ HandSearch ======================\n";
                stream << "is_plot_normals: " << data.is_plot_normals << "\n";
                stream << "is_plot_samples: " << data.is_plot_samples << "\n";
                stream << "is_plot_candidates: " << data.is_plot_candidates << "\n";
                stream << "is_plot_filtered_candidates: " << data.is_plot_filtered_candidates << "\n";
                stream << "is_plot_clustered_grasps: " << data.is_plot_clustered_grasps << "\n";
                stream << "is_plot_selected_grasps: " << data.is_plot_selected_grasps << "\n";
                stream << "is_plots_local_axes: " << data.is_plots_local_axes << "\n";
                stream << "is_plots_antipodal_grasps: " << data.is_plots_antipodal_grasps << "\n";
                stream << "=================================================\n";
            }
        };

        /**
             * \brief Params for the candidates generator.
             */
        struct HandGeometryParams
        {
            double finger_width_;   ///< the width of the robot fingers
            double outer_diameter_; ///< the width of the robot hand including fingers
            double depth_;          ///< the hand depth (length of fingers)
            double height_;         ///< the hand extends plus/minus this along the hand axis
            double init_bite_;      ///< the minimum object depth to be covered by the fingers

            friend std::ostream &operator<<(std::ostream &stream, const HandGeometryParams &hand_geometry)
            {
                stream << "============ HAND GEOMETRY ======================\n";
                stream << "finger_width: " << hand_geometry.finger_width_ << "\n";
                stream << "hand_outer_diameter: " << hand_geometry.outer_diameter_ << "\n";
                stream << "hand_depth: " << hand_geometry.depth_ << "\n";
                stream << "hand_height: " << hand_geometry.height_ << "\n";
                stream << "init_bite: " << hand_geometry.init_bite_ << "\n";
                stream << "=================================================\n";
            }
        };

        struct HandSearchParams
        {
            /** LRF estimation parameters */
            double nn_radius_frames_; ///< radius for point neighborhood search for LRF

            /** grasp candidate generation */
            int num_threads_;            ///< the number of CPU threads to be used
            int num_samples_;            ///< the number of samples to be used
            int num_orientations_;       ///< number of hand orientations to evaluate
            int num_finger_placements_;  ///< number of finger placements to evaluate
            std::vector<int> hand_axes_; ///< the axes about which different hand
                                         /// orientations are generated
            bool deepen_hand_;           ///< if the hand is pushed forward onto the object

            /** antipodal grasp check */
            double friction_coeff_; ///< angle of friction cone in degrees
            int min_viable_;        ///< minimum number of points required to be antipodal

            HandGeometryParams hand_geometry_; ///< robot hand geometry

            friend std::ostream &operator<<(std::ostream &stream, const HandSearchParams &data)
            {
                stream << "============ HandSearch ======================\n";
                stream << "nn_radius_frames_: " << data.nn_radius_frames_ << "\n";
                stream << "num_threads_: " << data.num_threads_ << "\n";
                stream << "num_samples_: " << data.num_samples_ << "\n";
                stream << "num_orientations_: " << data.num_orientations_ << "\n";
                stream << "deepen_hand_: " << data.deepen_hand_ << "\n";
                stream << "num_finger_placements_: " << data.num_finger_placements_ << "\n";
                stream << "friction_coeff_: " << data.friction_coeff_ << "\n";
                stream << "min_viable_: " << data.min_viable_ << "\n";
                stream << "hand_geometry_: " << data.hand_geometry_ << "\n";
                stream << "=================================================\n";
            }
        };

        /**
         * \brief Params for the candidates generator.
         */
        struct CandidatesGeneratorParams
        {
            bool remove_statistical_outliers_; ///< if statistical outliers are removed
                                               /// from the point cloud
            bool sample_above_plane_;          ///< if samples are drawn above the support plane
            bool voxelize_;                    ///< if the point cloud gets voxelized
            double voxel_size_;                ///< voxel size
            double normals_radius_;            ///< neighborhood search radius used for normal
                                               ///< estimation
            int refine_normals_k_;             ///< If 0, do not refine. If > 0, this is the number
                                               ///< of neighbors used for refinement.
            int num_samples_;                  ///< the number of samples to be used in the search
            int num_threads_;                  ///< the number of CPU threads to be used in the search
            std::vector<double> workspace_;    ///< the robot's workspace

            friend std::ostream &operator<<(std::ostream &stream, const CandidatesGeneratorParams &data)
            {
                stream << "============ HandSearch ======================\n";
                stream << "remove_statistical_outliers_: " << data.remove_statistical_outliers_ << "\n";
                stream << "sample_above_plane_: " << data.sample_above_plane_ << "\n";
                stream << "voxelize_: " << data.voxelize_ << "\n";
                stream << "voxel_size_: " << data.voxel_size_ << "\n";
                stream << "normals_radius_: " << data.normals_radius_ << "\n";
                stream << "refine_normals_k_: " << data.refine_normals_k_ << "\n";
                stream << "num_samples_: " << data.num_samples_ << "\n";
                stream << "num_threads_: " << data.num_threads_ << "\n";
                stream << "workspace_: "
                       << "min_x,max_x: " << data.workspace_[0] << data.workspace_[1]
                       << "min_y,max_y: " << data.workspace_[2] << data.workspace_[3]
                       << "min_z,max_z: " << data.workspace_[4] << data.workspace_[5] << "\n";
                stream << "=================================================\n";
            }
        };

        /**
         * \brief Params for the clustering.
         */
        struct GraspPredictParams
        {
            int points_num = 2048;   // pointnet input num points
            double min_score;        // Soft conditions, grasp score > min_score
            int min_grasp_num = 100; // Hard conditions, grasp num > min_grasp_num
            int max_grasp_num = 500; // Hard conditions, grasp num < max_grasp_num

            friend std::ostream &operator<<(std::ostream &stream, const GraspPredictParams &data)
            {
                stream << "============ GraspPredictParams ======================\n";
                stream << "points_num: " << data.points_num << "\n";
                stream << "min_score: " << data.min_score << "\n";
                stream << "min_grasp_num: " << data.min_grasp_num << "\n";
                stream << "max_grasp_num: " << data.max_grasp_num << "\n";

                stream << "=================================================\n";
            }
        };

        /**
         * \brief Params for the clustering.
         */
        struct GraspClusterParams
        {
            bool is_clustering;     /// if clustering
            bool is_remove_inliers; /// if remove clusteringed grasp
            int min_inliers;        /// clustering min num grasp

            friend std::ostream &operator<<(std::ostream &stream, const GraspClusterParams &data)
            {
                stream << "============ GraspClusterParams ======================\n";
                stream << "is_clustering: " << data.is_clustering << "\n";
                stream << "is_remove_inliers: " << data.is_remove_inliers << "\n";
                stream << "min_inliers: " << data.min_inliers << "\n";

                stream << "=================================================\n";
            }
        };

        /**
         * \brief Params for the FilterGrasp.
         */
        struct GraspFilterParams
        {
            double min_aperture = 0.0; ///
            double max_aperture = 0.07;
            std::vector<double> workspace_grasps = {-1, 1, 0.5, 1, -1, 1};

            bool is_filter_direction = false;
            std::vector<double> direction = {1, 0, 0};
            double thresh_rad = 2.0;

            friend std::ostream &operator<<(std::ostream &stream, const GraspFilterParams &data)
            {
                stream << "============ GraspFilterParams ======================\n";
                stream << "min_aperture: " << data.min_aperture << "\n";
                stream << "max_aperture: " << data.max_aperture << "\n";
                stream << "workspace_grasps: ";
                for (auto n : data.workspace_grasps)
                    stream << " " << n;
                stream << std::endl;

                stream << "is_filter_direction: " << data.is_filter_direction << "\n";
                stream << "thresh_rad: " << data.thresh_rad << "\n";

                stream << "direction: ";
                for (auto n : data.direction)
                    stream << " " << n;
                stream << std::endl;

                stream << "=================================================\n";
            }
        };

    } // namespace util
} // namespace grasp_detect