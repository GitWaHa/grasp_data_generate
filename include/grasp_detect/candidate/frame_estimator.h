#pragma once

#include <vector>

#include <Eigen/Dense>

#include <pcl/kdtree/kdtree.h>

#include <omp.h>

#include "grasp_detect/candidate/local_frame.h"
#include "grasp_detect/util/cloud.h"

namespace grasp_detect
{
    namespace candidate
    {

        typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

        /**
         *
         * \brief Estimate local reference frames.
         *
         * This class estimates local reference frames (LRFs) for point neighborhoods.
         *
         */
        class FrameEstimator
        {
        public:
            /**
   * \brief Constructor.
   * \param num_threads the number of CPU threads to be used
   */
            FrameEstimator(int num_threads) : num_threads_(num_threads) {}

            /**
   * \brief Calculate local reference frames given a list of point cloud
   * indices.
   * \param cloud_cam the point cloud
   * \param indices the list of indices into the point cloud
   * \param radius the radius for the point neighborhood search
   * \param kdtree the kdtree used for faster neighborhood search
   * \return the list of local reference frames
   */
            std::vector<LocalFrame> calculateLocalFrames(
                const util::Cloud &cloud_cam, const std::vector<int> &indices,
                double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const;

            /**
   * \brief Calculate local reference frames given a list of (x,y,z) samples.
   * \param cloud_cam the point cloud
   * \param samples the list of (x,y,z) samples
   * \param radius the radius for the point neighborhood search
   * \param kdtree the kdtree used for faster neighborhood search
   * \return the list of local reference frames
   */
            std::vector<LocalFrame> calculateLocalFrames(
                const util::Cloud &cloud_cam, const Eigen::Matrix3Xd &samples,
                double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const;

            /**
   * \brief Calculate a local reference frame given a list of surface normals.
   * \param normals the list of surface normals
   * \param sample the center of the point neighborhood
   * \param radius the radius of the point neighborhood
   * \param kdtree the kdtree used for faster neighborhood search
   * \return the local reference frame
   */
            std::unique_ptr<LocalFrame> calculateFrame(
                const Eigen::Matrix3Xd &normals, const Eigen::Vector3d &sample,
                double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const;

        private:
            /**
   * \brief Convert an Eigen::Vector3d object to a pcl::PointXYZRGBA.
   * \param v the Eigen vector
   * \reutrn the pcl point
   */
            pcl::PointXYZRGBA eigenVectorToPcl(const Eigen::Vector3d &v) const;

            int num_threads_; ///< number of CPU threads to be used for calculating local
                              /// reference frames
        };

    } // namespace candidate
} // namespace grasp_detect

