#pragma once

// System
#include <iostream>
#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <fstream>

#include <boost/filesystem.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include <opencv2/core/core.hpp>

#include "grasp_detect/candidate/candidates_generator.h"
#include "grasp_detect/candidate/hand_set.h"

#include "grasp_detect/util/config_file.h" // 配置文件读取
#include "grasp_detect/util/cloud.h"	   // 点云处理相关
#include "grasp_detect/util/plot.h"		   // 画图
#include "grasp_detect/util/params_manager.h"
#include "grasp_detect/util/grasp.h"

namespace grasp_detect
{

	/**
     *
     * \brief Detect grasp poses in point clouds.
     *
     * This class detects grasp poses in a point clouds by first creating a large
     * set of grasp candidates, and then classifying each of them as a grasp or not.
     *
     */
	class GraspClassifyDataGenerate
	{
	public:
		using ParamsManager = grasp_detect::util::ParamsManager;

	public:
		GraspClassifyDataGenerate(ParamsManager &params);
		~GraspClassifyDataGenerate();

		void preprocessCloud(grasp_detect::util::Cloud &cloud);

		bool readGrasp(const std::string file_name, grasp_detect::util::Grasp &grasps);

		void saveClassifyDateset(grasp_detect::util::Cloud &cloud,
								 const grasp_detect::util::Grasp &grasps,
								 const std::vector<std::size_t> &sort_index,
								 int grasp_num,
								 int points_num,
								 const std::string save_path);

		void save(grasp_detect::util::Cloud &cloud, const std::string read_file, const std::string write_file);

	private:
		std::vector<size_t> sortGrasp(const std::vector<double> &score);
		Eigen::Matrix3Xd cropPointsInGrasps(const Eigen::Matrix3Xd &points,
											const std::vector<double> &center,
											const Eigen::Matrix3d &hamd_frame,
											pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree);
		std::string rename(const std::string file_name, int n);

		ParamsManager &params_;
		grasp_detect::util::Plot plotter_;
		std::unique_ptr<grasp_detect::candidate::CandidatesGenerator> candidates_generator_;
	};

} // namespace grasp_detect
