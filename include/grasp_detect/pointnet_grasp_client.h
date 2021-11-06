#pragma once

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

#include "grasp_detect/util/grasp.h"

namespace grasp_detect
{

	class PointNetGraspClient
	{
	public:
		PointNetGraspClient(std::string service_name);
		~PointNetGraspClient();

		bool callService(std::vector<std::vector<double>> &points, grasp_detect::util::Grasp &grasps, double min_score = 0.5, int min_num = 100, int max_num = 500);

	private:
		/* data */
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
	};

} // namespace grasp_detect
