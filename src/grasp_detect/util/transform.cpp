#include "grasp_detect/util/transform.h"

//　计算两个向量的夹角
double AngleFromTwoVector(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    // 单位弧度
    return atan2(vectorBefore.cross(vectorAfter).norm(), vectorBefore.transpose() * vectorAfter);
}

inline Eigen::Vector3d calculateRotAxis(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    return Eigen::Vector3d(vectorBefore.y() * vectorAfter.z() - vectorBefore.z() * vectorAfter.y(),
                           vectorBefore.z() * vectorAfter.y() - vectorBefore.x() * vectorAfter.z(),
                           vectorBefore.x() * vectorAfter.y() - vectorBefore.y() * vectorAfter.x());
}

//计算两个向量的旋转矩阵
Eigen::Matrix3d computeRotationMatrixTwoVector(const Eigen::Vector3d &vectorBefore, const Eigen::Vector3d &vectorAfter)
{
    Eigen::Vector3d vector = calculateRotAxis(vectorBefore, vectorAfter);
    double angle = AngleFromTwoVector(vectorBefore, vectorAfter);
    Eigen::AngleAxisd rotationVector(angle, vector.normalized());
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rotMatrix = rotationVector.toRotationMatrix(); //所求旋转矩阵

    return rotMatrix;
}

/**
 * \brief 根据坐标系向量，求旋转矩阵
 * \param approach approach
 * \param axis axis
 */
Eigen::Matrix3d computeRotationMatrixFrame(const Eigen::Vector3d &approach, const Eigen::Vector3d &axis)
{
    /*根据approach向量计算旋转矩阵，此时只能保证approach与机械臂抓取参考坐标系x轴重合*/
    Eigen::Vector3d temp(1, 0, 0);
    Eigen::Matrix3d rotation_matrix_1 = computeRotationMatrixTwoVector(temp, approach);

    /*根据　axis　向量计算与x轴重合后的z轴与axis的角度*/
    //求角度正负
    int flag = (rotation_matrix_1.inverse() * axis)(1) < 0 ? 1 : -1;
    temp << 0, 0, 1;
    Eigen::Vector3d axis_z_1 = rotation_matrix_1 * temp;
    double roll3_angle2 = AngleFromTwoVector(axis_z_1, axis);

    //欧拉角转换为旋转矩阵
    Eigen::Vector3d ea(0, 0, flag * roll3_angle2);
    Eigen::Matrix3d rotation_matrix_2;
    rotation_matrix_2 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    // cout << "rotation matrix =\n" << rotation_matrix_2 << endl;

    /*最终旋转矩阵*/
    rotation_matrix_2 = rotation_matrix_1 * rotation_matrix_2;

    return rotation_matrix_2;
}