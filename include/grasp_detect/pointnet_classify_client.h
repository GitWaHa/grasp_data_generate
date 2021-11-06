#pragma once

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

#include "grasp_detect/util/grasp.h"
#include <Eigen/Dense>

namespace grasp_detect
{

	class PointNetClassifyClient
	{
	public:
		PointNetClassifyClient(std::string service_name);
		~PointNetClassifyClient();

		bool callService(const Eigen::Matrix3Xd &points, int batch_size, std::vector<double> &scores);

	private:
		/* data */
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
	};

} // namespace grasp_detect
