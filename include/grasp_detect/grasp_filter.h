#pragma once
#include <vector>
#include <memory>

#include "grasp_detect/util/parameter.h"
#include "grasp_detect/candidate/hand_set.h"

namespace grasp_detect
{

    /**
     *
     * \brief 过滤不必要的抓取候选
     *
     */
    class GraspFilter
    {
    public:
        GraspFilter(const util::GraspFilterParams &filter_params, const util::HandGeometryParams &hand_params)
            : grasp_filter_params_(filter_params), hand_geometry_params_(hand_params){};

        ~GraspFilter(){};

        /**
         * Filter grasps based on the robot's workspace.
         * \param hand_set_list list of grasp candidate sets
         * \param workspace the robot's workspace as a 3D cube, centered at the origin
         * \param thresh_rad the angle in radians above which grasps are filtered
         * \return list of grasps after filtering
         */
        std::vector<std::unique_ptr<candidate::Hand>> filterGraspsWorkspace(
            std::vector<std::unique_ptr<candidate::Hand>> &hand_list);
        /**
         * Filter grasps based on their approach direction.
         * \param hand_set_list list of grasp candidate sets
         * \param direction the direction used for filtering
         * \param thresh_rad the angle in radians above which grasps are filtered
         * \return list of grasps after filtering
         */
        std::vector<std::unique_ptr<candidate::Hand>> filterGraspsDirection(
            std::vector<std::unique_ptr<candidate::Hand>> &hand_list);

    private:
        const util::GraspFilterParams &grasp_filter_params_;
        const util::HandGeometryParams &hand_geometry_params_;
    };
}