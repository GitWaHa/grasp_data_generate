#include "grasp_detect/grasp_filter.h"
using namespace std;

namespace grasp_detect
{
    std::vector<std::unique_ptr<candidate::Hand>> GraspFilter::filterGraspsWorkspace(std::vector<std::unique_ptr<candidate::Hand>> &hand_list)
    {
        std::vector<std::unique_ptr<candidate::Hand>> hand_list_out;
        printf("[GraspFilter] Filtering grasps outside of workspace ...\n");

        for (int j = 0; j < hand_list.size(); j++)
        {
            double half_width = 0.5 * hand_geometry_params_.outer_diameter_;
            Eigen::Vector3d left_bottom =
                hand_list[j]->getPosition() + half_width * hand_list[j]->getBinormal();
            Eigen::Vector3d right_bottom =
                hand_list[j]->getPosition() - half_width * hand_list[j]->getBinormal();
            Eigen::Vector3d left_top =
                left_bottom + hand_geometry_params_.depth_ * hand_list[j]->getApproach();
            Eigen::Vector3d right_top =
                left_bottom + hand_geometry_params_.depth_ * hand_list[j]->getApproach();
            Eigen::Vector3d approach =
                hand_list[j]->getPosition() - 0.05 * hand_list[j]->getApproach();
            Eigen::VectorXd x(5), y(5), z(5);
            x << left_bottom(0), right_bottom(0), left_top(0), right_top(0),
                approach(0);
            y << left_bottom(1), right_bottom(1), left_top(1), right_top(1),
                approach(1);
            z << left_bottom(2), right_bottom(2), left_top(2), right_top(2),
                approach(2);

            // Ensure the object fits into the hand and avoid grasps outside the workspace.
            const auto &workspace = grasp_filter_params_.workspace_grasps;
            if (hand_list[j]->getGraspWidth() >= grasp_filter_params_.min_aperture &&
                hand_list[j]->getGraspWidth() <= grasp_filter_params_.max_aperture &&
                x.minCoeff() >= workspace[0] && x.maxCoeff() <= workspace[1] &&
                y.minCoeff() >= workspace[2] && y.maxCoeff() <= workspace[3] &&
                z.minCoeff() >= workspace[4] && z.maxCoeff() <= workspace[5])
            {
                hand_list_out.push_back(std::move(hand_list[j]));
            }
        }

        std::cout << "[GraspFilter] Number of grasp candidates within workspace and gripper width: " << hand_list_out.size() << std::endl;

        return hand_list_out;
    }

    std::vector<std::unique_ptr<candidate::Hand>> GraspFilter::filterGraspsDirection(
        std::vector<std::unique_ptr<candidate::Hand>> &hand_list)
    {
        std::vector<std::unique_ptr<candidate::Hand>> hand_list_out;

        const Eigen::Vector3d direction(grasp_filter_params_.direction[0], grasp_filter_params_.direction[1], grasp_filter_params_.direction[2]);

        for (int i = 0; i < hand_list.size(); i++)
        {
            double angle = acos(direction.transpose() * hand_list[i]->getApproach()) / M_PI * 180;
            if (angle < grasp_filter_params_.thresh_rad)
            {
                hand_list_out.push_back(std::move(hand_list[i]));
            }
        }

        std::cout << "[GraspFilter] Number of grasp candidates within Direction: " << hand_list_out.size() << std::endl;

        return hand_list_out;
    }

}
