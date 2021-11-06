#pragma once

#include <Eigen/Dense>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>

#include <omp.h>

#include <memory>

#include "grasp_detect/candidate/frame_estimator.h"
#include "grasp_detect/candidate/antipodal.h"
#include "grasp_detect/candidate/finger_hand.h"
#include "grasp_detect/candidate/hand.h"
// #include "grasp_detect/candidate/hand_geometry.h"
#include "grasp_detect/candidate/hand_set.h"
#include "grasp_detect/candidate/local_frame.h"

#include "grasp_detect/util/point_list.h"
#include "grasp_detect/util/plot.h"
#include "grasp_detect/util/parameter.h"

namespace grasp_detect
{
    namespace candidate
    {

        typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGB;

        /**
 *
 * \brief Search for grasp candidates.
 *
 * This class searches for grasp candidates in a point cloud by first
 * calculating a local reference frame (LRF) for a point neighborhood, and then
 * finding geometrically feasible robot hand placements. Each feasible grasp
 * candidate is checked for mechanical stability (antipodal grasp or not).
 *
 */
        class HandSearch
        {
        public:
            /**
             * \brief Params for the hand search.
             */
            using HandGeometry = grasp_detect::util::HandGeometryParams;

            /**
             * \brief Constructor.
             * \param params Params for the hand search
             */
            HandSearch(grasp_detect::util::ParamsManager &params) : HandSearch(params.getHandSearchParams())
            {
                plots_local_axes_ = params.getPlotParams().is_plots_local_axes;
            };
            HandSearch(const grasp_detect::util::HandSearchParams &params);

            /**
   * \brief Search robot hand configurations.
   * \param cloud the point cloud
   * \return list of grasp candidate sets
   */
            std::vector<std::unique_ptr<candidate::HandSet>> searchHands(
                const util::Cloud &cloud) const;

            /**
             * \brief Reevaluate a list of grasp candidates.
             * \note Used to calculate ground truth.
             * \param cloud_cam the point cloud
             * \param grasps the list of grasp candidates
             * \return the list of reevaluated grasp candidates
             */
            std::vector<int> reevaluateHypotheses(
                const util::Cloud &cloud_cam,
                std::vector<std::unique_ptr<candidate::Hand>> &grasps,
                bool plot_samples = false) const;

            /**
             * \brief Return the parameters for the hand search.
             * \return params the hand search parameters
             */
            const grasp_detect::util::HandSearchParams &getParams() const { return params_; }

            /**
             * \brief Set the parameters for the hand search.
             * \param params the parameters
             */
            // void setParams(const grasp_detect::util::HandSearchParams &params) { params_ = params; }

        private:
            /**
             * \brief Search robot hand configurations given a list of local reference
             * frames.
             * \param cloud_cam the point cloud
             * \param frames the list of local reference frames
             * \param kdtree the KDTree object used for fast neighborhood search
             * \return the list of robot hand configurations
             */
            std::vector<std::unique_ptr<candidate::HandSet>> evalHands(
                const util::Cloud &cloud_cam,
                const std::vector<candidate::LocalFrame> &frames,
                const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const;

            /**
   * \brief Reevaluate a grasp candidate.
   * \param point_list the point neighborhood associated with the grasp
   * \param hand the grasp
   * \param finger_hand the FingerHand object that describes a valid finger
   * placement
   * \param point_list_cropped the point neigborhood transformed into the hand
   * frame
   */
            bool reevaluateHypothesis(const util::PointList &point_list,
                                      const candidate::Hand &hand,
                                      FingerHand &finger_hand,
                                      util::PointList &point_list_cropped) const;

            /**
   * \brief Calculate the label for a grasp candidate.
   * \param point_list the point neighborhood associated with the grasp
   * \param finger_hand the FingerHand object that describes a valid finger
   * placement
   * \return the label
   */
            int labelHypothesis(const util::PointList &point_list,
                                FingerHand &finger_hand) const;

            /**
   * \brief Convert an Eigen::Vector3d object to a pcl::PointXYZRGBA.
   * \param v the Eigen vector
   * \reutrn the pcl point
   */
            pcl::PointXYZRGBA eigenVectorToPcl(const Eigen::Vector3d &v) const;

            const grasp_detect::util::HandSearchParams &params_; ///< parameters for the hand search

            double nn_radius_; ///< radius for nearest neighbors search

            std::unique_ptr<Antipodal> antipodal_;
            std::unique_ptr<util::Plot> plot_;

            /** plotting parameters (optional, not read in from config file) **/
            bool plots_local_axes_; ///< if the LRFs are plotted

            /** constants for rotation axis */
            static const int ROTATION_AXIS_NORMAL;         ///< normal axis of LRF
            static const int ROTATION_AXIS_BINORMAL;       ///< binormal axis of LRF
            static const int ROTATION_AXIS_CURVATURE_AXIS; ///< curvature axis of LRF
        };

    } // namespace candidate
} // namespace grasp_detect
