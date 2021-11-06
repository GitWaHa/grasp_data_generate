#pragma once

#include <Eigen/Dense>

#include <vector>

#include "grasp_detect/util/eigen_utils.h"

namespace grasp_detect
{
	namespace util
	{

		/**
		 *
		 * \brief List of points
		 *
		 * Stores a list of *n* points (3 x n matrix) together with their surface
		 * normals
		 * (3 x n matrix). Also keeps information about which camera sees which point
		 * by storing the view points (3 x k matrix), i.e., locations, of *k*
		 * cameras, and for each of the *n* points, if the point is seen or not from
		 * a camera (k x n matrix). If point *i* is seen from camera *j*, then
		 * `cam_source(i,j)` = 1. Otherwise, `cam_source(i,j)` = 0.
		 *
		 */
		class PointList
		{
		public:
			/**
   * \brief Default constructor.
   */
			PointList() {}

			/**
   * \brief Construct a list of n points.
   * \param points the points (3 x n)
   * \param normals the surface normals associated with the points (3 x n)
   * \param cam_source the camera source for each point (k x n)
   * \param view_points the origins of the cameras that saw the points (3 x k)
   */
			PointList(const Eigen::Matrix3Xd &points, const Eigen::Matrix3Xd &normals,
					  const Eigen::MatrixXi &cam_source,
					  const Eigen::Matrix3Xd &view_points)
				: points_(points),
				  normals_(normals),
				  cam_source_(cam_source),
				  view_points_(view_points) {}

			/**
   * \brief Constructor.
   * \param size number of points
   * \param num_cams number of cameras that observed the points
   */
			PointList(int size, int num_cams);

			/**
			 * \brief Slice the point list given a set of indices.
			 * \param indices the indices to be sliced
			 * \return the point list containing the points given by the indices
			 */
			PointList slice(const std::vector<int> &indices) const;

			/**
   * \brief Transform a point list to a robot hand frame.
   * \param centroid the origin of the frame
   * \param rotation the orientation of the frame (3 x 3 rotation matrix)
   * \return the point list transformed into the hand frame
   */
			PointList transformToHandFrame(const Eigen::Vector3d &centroid,
										   const Eigen::Matrix3d &rotation) const;

			/**
   * \brief Rotate a point list.
   * \param rotation the 3 x 3 rotation matrix
   * \return the rotated point list
   */
			PointList rotatePointList(const Eigen::Matrix3d &rotation) const;

			/**
   * \brief Crop the points by the height of the robot hand.
   * \param height the robot hand height
   * \param dim the dimension of the points corresponding to the height
   */
			PointList cropByHandHeight(double height, int dim = 2) const;

			/**
   * \brief Return the camera source matrix.
   * \return the camera source matrix (size: k x n)
   */
			const Eigen::MatrixXi &getCamSource() const { return cam_source_; }

			/**
   * \brief Set the camera source matrix.
   * \param cam_source the camera source matrix (size: k x n)
   */
			void setCamSource(const Eigen::MatrixXi &cam_source)
			{
				cam_source_ = cam_source;
			}

			/**
   * \brief Return the surface normals.
   * \return the surface normals (size: 3 x n)
   */
			const Eigen::Matrix3Xd &getNormals() const { return normals_; }

			/**
   * \brief Set the surface normals.
   * \param normals the surface normals (size: 3 x n)
   */
			void setNormals(const Eigen::Matrix3Xd &normals) { normals_ = normals; }

			/**
   * \brief Return the points.
   * \return the points (size: 3 x n)
   */
			const Eigen::Matrix3Xd &getPoints() const { return points_; }

			/**
   * \brief Set the points.
   * \param points the points (size: 3 x n)
   */
			void setPoints(const Eigen::Matrix3Xd &points) { points_ = points; }

			/**
   * \brief Return the size of the list.
   * \return the number of points in the list
   */
			int size() const { return points_.cols(); }

			/**
   * \brief Return the view points of the cameras.
   * \return the view points (size: 3 x k)
   */
			const Eigen::Matrix3Xd &getViewPoints() const { return view_points_; }

			/**
   * \brief Set the view points of the cameras.
   * \param points the view points (size: 3 x k)
   */
			void setViewPoints(const Eigen::Matrix3Xd &view_points)
			{
				view_points_ = view_points;
			}

			// The following macro makes sure that pointers are aligned correctly.
			// See
			// https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html.
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		private:
			Eigen::Matrix3Xd points_;
			Eigen::Matrix3Xd normals_;
			Eigen::MatrixXi cam_source_; // camera source (k x n matrix of 1s and 0s)
			Eigen::Matrix3Xd view_points_;
		};

	} // namespace util
} // namespace grasp_detect

