#pragma once

#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "grasp_detect/util/point_list.h"

namespace grasp_detect
{
	namespace candidate
	{

		/**
		 *
		 * \brief Check if a grasp is antipodal.
		 *
		 * This class checks if a grasp candidate satisfies the antipodal condition.
		 *
		 */
		class Antipodal
		{
		public:
			Antipodal(double friction_coeff, int min_viable)
				: friction_coeff_(friction_coeff), min_viable_(min_viable) {}

			int evaluateGrasp(const util::PointList &point_list,
							  double extremal_thresh, int lateral_axis,
							  int forward_axis, int vertical_axis, double &score) const;

			/**
			 * \brief Check if a grasp is antipodal.
			 * \param point_list the list of points associated with the grasp
			 * \param extremal_threshold
			 * \param lateral_axis the closing direction of the robot hand
			 * \param forward_axis the forward direction of the robot hand
			 * \param vertical_axis the vertical direction of the robot hand
			 * \return 0 if it's not antipodal, 1 if one finger is antipodal, 2 if the
			 * grasp is antipodal
			 */
			int evaluateGrasp(const util::PointList &point_list, double extremal_thresh,
							  int lateral_axis = 0, int forward_axis = 1,
							  int vertical_axis = 2) const;

			/**
			 * \brief Check if a grasp is antipodal.
			 * \note Deprecated method.
			 * \param normals the set of surface normals associated with the grasp
			 * \param thresh_half the threshold to consider the grasp half-antipodal
			 * \param thresh_full the threshold to conisder the grasp full-antipodal
			 */
			int evaluateGrasp(const Eigen::Matrix3Xd &normals, double thresh_half,
							  double thresh_full) const;

			double friction_coeff_; ///< angle of friction cone in degrees
			int min_viable_;		///< minimum number of points on each side to be antipodal

			static const int NO_GRASP;	 // normals point not toward any finger
			static const int HALF_GRASP; // normals point towards one finger
			static const int FULL_GRASP; // normals point towards both fingers
		};

	} // namespace candidate
} // namespace grasp_detect
