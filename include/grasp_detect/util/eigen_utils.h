#pragma once

#include <Eigen/Dense>

#include <vector>

namespace grasp_detect
{
    namespace util
    {

        /**
 *
 * \brief Utility functions for the Eigen matrix library
 *
 * This namespace contains utility functions for the Eigen matrix library such
 * as slicing matrices and rounding vectors.
 *
 */
        namespace EigenUtils
        {
            /**
 * \brief Slice a given matrix given a set of column indices.
 * \param mat the matrix to be sliced
 * \param indices set of column indices
 * \return the columns of the given matrix contained in the indices set
 */
            Eigen::Matrix3Xd sliceMatrix(const Eigen::Matrix3Xd &mat,
                                         const std::vector<int> &indices);

            /**
 * \brief Slice a given matrix given a set of column indices.
 * \param mat the matrix to be sliced
 * \param indices set of column indices
 * \return the columns of the given matrix contained in the indices set
 */
            Eigen::MatrixXi sliceMatrix(const Eigen::MatrixXi &mat,
                                        const std::vector<int> &indices);

            /**
 * \brief Round the elements of a floating point vector to the nearest, smaller
 * integers.
 * \param a the vector to be rounded down
 * \return the vector containing the rounded down elements
 */
            Eigen::Vector3i floorVector(const Eigen::Vector3f &a);
        } // namespace EigenUtils

    } // namespace util
} // namespace grasp_detect
