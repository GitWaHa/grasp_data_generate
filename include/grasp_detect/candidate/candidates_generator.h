#pragma once

// System
#include <memory>
#include <vector>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom
#include "grasp_detect/candidate/hand.h"
// #include "grasp_detect/candidate/hand_geometry.h"
#include "grasp_detect/candidate/hand_search.h"
#include "grasp_detect/candidate/hand_set.h"

#include "grasp_detect/util/config_file.h"
#include "grasp_detect/util/parameter.h"
#include "grasp_detect/util/params_manager.h"

namespace grasp_detect
{
    namespace candidate
    {

        /**
         *
         * \brief Generate grasp candidates.
         *
         * This class generates grasp candidates by searching for feasible robot hand
         * placements in a point cloud.
         *
         */
        class CandidatesGenerator
        {
        public:
            using HandSearchParams = grasp_detect::util::HandSearchParams;
            using CandidatesGeneratorParams = grasp_detect::util::CandidatesGeneratorParams;
            using PreprocessCloudParams = grasp_detect::util::PreprocessCloudParams;

            /**
             * \brief Constructor.
             * \param params the parameters to be used for the candidate generation
             * \param hand_search_params the parameters to be used for the hand search
             */
            CandidatesGenerator(grasp_detect::util::ParamsManager &params_manage);

            // CandidatesGenerator(const CandidatesGeneratorParams &params,
            //                     const HandSearchParams &hand_search_params);

            /**
             * \brief Preprocess the point cloud.
             * \param cloud_cam the point cloud
             */
            void preprocessPointCloud(util::Cloud &cloud);

            /**
   * \brief Generate grasp candidates given a point cloud.
   * \param cloud_cam the point cloud
   * \return list of grasp candidates
   */
            std::vector<std::unique_ptr<Hand>> generateGraspCandidates(
                const util::Cloud &cloud_cam);

            /**
   * \brief Generate grasp candidate sets given a point cloud.
   * \param cloud_cam the point cloud
   * \return lust of grasp candidate sets
   */
            std::vector<std::unique_ptr<HandSet>> generateGraspCandidateSets(
                const util::Cloud &cloud_cam);

            /**
   * \brief Reevaluate grasp candidates on a given point cloud.
   * \param cloud the point cloud
   * \param grasps the grasps to evaluate
   */
            std::vector<int> reevaluateHypotheses(
                const util::Cloud &cloud, std::vector<std::unique_ptr<Hand>> &grasps);

            /**
   * \brief Set the number of samples.
   * \param num_samples the number of samples
   */
            void setNumSamples(int num_samples) { params_.num_samples_ = num_samples; }

            /**
             * \brief Return the hand search parameters.
             * \return the hand search parameters
             */
            const grasp_detect::util::HandSearchParams &getHandSearchParams() const
            {
                return hand_search_->getParams();
            }

        private:
            std::unique_ptr<candidate::HandSearch> hand_search_;
            grasp_detect::util::ParamsManager &params_manage_;
            CandidatesGeneratorParams params_;
        };

    } // namespace candidate
} // namespace grasp_detect
