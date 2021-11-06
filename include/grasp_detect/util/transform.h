#pragma once

#include <iostream>

#include <Eigen/Dense>

Eigen::Matrix3d computeRotationMatrixFrame(const Eigen::Vector3d &approach, const Eigen::Vector3d &axis);