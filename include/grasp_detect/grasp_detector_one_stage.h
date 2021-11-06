#pragma once

// System
#include <algorithm>
#include <memory>
#include <vector>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include <opencv2/core/core.hpp>

#include "grasp_detect/candidate/candidates_generator.h"
#include "grasp_detect/candidate/hand_set.h"
#include "grasp_detect/util/params_manager.h"

#include "grasp_detect/util/config_file.h" // 配置文件读取
#include "grasp_detect/util/cloud.h"	   // 点云处理相关
#include "grasp_detect/util/plot.h"		   // 画图
#include "grasp_detect/util/transform.h"

#include "grasp_detect/pointnet_grasp_client.h"
#include "grasp_detect/grasp_cluster.h"
#include "grasp_detect/grasp_filter.h"

#include <ros/ros.h> //ros
#include <std_srvs/SetBool.h>

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
	class GraspDetectorOneStage
	{
	public:
		/**
         * \brief Constructor.
         * \param node ROS node handle
         */
		GraspDetectorOneStage(std::shared_ptr<grasp_detect::util::ParamsManager> params_manager, grasp_detect::PointNetGraspClient &client);

		/**
         * \brief Detect grasps in a point cloud.
         * \param cloud_cam the point cloud
         * \return list of grasps
         */
		std::vector<std::unique_ptr<candidate::Hand>> detectGrasps(util::Cloud &cloud);

		/**
           * \brief Preprocess the point cloud.
           * \param cloud_cam the point cloud
           */
		void preprocessPointCloud(util::Cloud &cloud);

		/**
           * \brief save Sample Point.
           * \param file_name save file path
           */
		void saveSamplePoint(const util::Cloud &cloud, const std::string file_path);

		void getSamplePointFromIndex(const util::Cloud &cloud, std::vector<std::vector<double>> &out_points);

		std::vector<std::unique_ptr<candidate::Hand>> graspToHandList(const grasp_detect::util::Grasp &grasps);

		/**
           * \brief predict grasp candidates.
           * \param cloud the point cloud
           * \return the list of grasp candidates
           */
		std::vector<std::unique_ptr<candidate::Hand>> predictGraspCandidates(const util::Cloud &cloud);

		/**
		   * \brief Select the k highest scoring grasps.
		   * \param hands the grasps
		   * \return the k highest scoring grasps
		   */
		std::vector<std::unique_ptr<candidate::Hand>> selectGrasps(
			std::vector<std::unique_ptr<candidate::Hand>> &hands) const;

		/**
           * \brief Compare the scores of two given grasps.
           * \param hand1 the first grasp to be compared
           * \param hand1 the second grasp to be compared
           * \return `true` if \param hand1 has a larger score than \param hand2,
           * `false` otherwise
           */
		static bool isScoreGreater(const std::unique_ptr<candidate::Hand> &hand1,
								   const std::unique_ptr<candidate::Hand> &hand2)
		{
			return hand1->getScore() > hand2->getScore();
		}

	private:
		void printStdVector(const std::vector<int> &v, const std::string &name) const;

		void printStdVector(const std::vector<double> &v,
							const std::string &name) const;

		std::shared_ptr<grasp_detect::util::ParamsManager> params_manager_; //ParamsManager
		grasp_detect::PointNetGraspClient &client_;							//used to call PointNet server

		std::unique_ptr<GraspFilter> filter_;	//过滤
		std::unique_ptr<GraspCluster> cluster_; //聚类
		std::unique_ptr<util::Plot> plotter_;	//画图

		// selection parameters
		int num_selected_; ///< the number of selected grasps
	};

} // namespace grasp_detect
