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
	class GraspDatasetGenerate
	{
	public:
		using ParamsManager = grasp_detect::util::ParamsManager;

	public:
		GraspDatasetGenerate(ParamsManager &params);
		~GraspDatasetGenerate();

		void preprocessCloud(grasp_detect::util::Cloud &cloud);

		std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> generateGraspCandidates(const grasp_detect::util::Cloud &cloud_cam);
		std::vector<std::unique_ptr<grasp_detect::candidate::HandSet>> generateGraspCandidateSets(const grasp_detect::util::Cloud &cloud_cam);

		void saveGrasp(std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> &hand_list, std::string file_name);
		void saveGrasp(std::vector<std::unique_ptr<grasp_detect::candidate::HandSet>> &hand_set_list,
					   std::string grasp_file_name,
					   std::string pcd_file_name,
					   std::string point_file_name, int num);

	private:
		ParamsManager &params_;
		grasp_detect::util::Plot plotter_;
		std::unique_ptr<grasp_detect::candidate::CandidatesGenerator> candidates_generator_;
	};

} // namespace grasp_detect
