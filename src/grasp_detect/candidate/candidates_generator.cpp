#include "grasp_detect/candidate/candidates_generator.h"
#include <iostream>

namespace grasp_detect
{
    namespace candidate
    {

        CandidatesGenerator::CandidatesGenerator(grasp_detect::util::ParamsManager &params_manage)
            : params_manage_(params_manage), params_(params_manage.getCandidatesGeneratorParams())
        {
            Eigen::initParallel();

            hand_search_ = std::make_unique<candidate::HandSearch>(params_manage);
        }

        // CandidatesGenerator::CandidatesGenerator(
        //     const CandidatesGeneratorParams &params, const HandSearchParams &hand_search_params)
        //     : params_(params)
        // {
        //     Eigen::initParallel();

        //     hand_search_ = std::make_unique<candidate::HandSearch>(hand_search_params);
        // }

        void CandidatesGenerator::preprocessPointCloud(util::Cloud &cloud)
        {
            printf("[CandidatesGenerator] Processing cloud with %zu points.\n", cloud.getCloudOriginal()->size());

            if (params_manage_.getPreprocessCloudParams().is_random_sample)
                cloud.randomSample(params_manage_.getPreprocessCloudParams().num_random_sample);

            cloud.removeNans();

            cloud.filterWorkspace(params_.workspace_);

            if (params_.voxelize_)
            {
                cloud.voxelizeCloud(params_.voxel_size_);
            }

            cloud.calculateNormals(params_.num_threads_, params_.normals_radius_);

            if (params_.refine_normals_k_ > 0)
            {
                cloud.refineNormals(params_.refine_normals_k_);
            }

            if (params_.sample_above_plane_)
            {
                cloud.sampleAbovePlane();
            }

            cloud.subsample(params_.num_samples_);
        }

        std::vector<std::unique_ptr<Hand>> CandidatesGenerator::generateGraspCandidates(
            const util::Cloud &cloud_cam)
        {
            // Find sets of grasp candidates.
            std::vector<std::unique_ptr<HandSet>> hand_set_list =
                hand_search_->searchHands(cloud_cam);
            printf("[CandidatesGenerator] Evaluated %d hand sets with %d potential hand poses.\n",
                   (int)hand_set_list.size(),
                   (int)(hand_set_list.size() * hand_set_list[0]->getHands().size()));

            // Extract the grasp candidates.
            std::vector<std::unique_ptr<Hand>> candidates;
            for (int i = 0; i < hand_set_list.size(); i++)
            {
                for (int j = 0; j < hand_set_list[i]->getHands().size(); j++)
                {
                    if (!hand_set_list[i]->getHands()[j]->isHalfAntipodal())
                        std::cout << "[CandidatesGenerator] not HalfAntipodal" << std::endl;
                    if (hand_set_list[i]->getIsValid()(j) && hand_set_list[i]->getHands()[j]->isHalfAntipodal())
                    {
                        candidates.push_back(std::move(hand_set_list[i]->getHands()[j]));
                    }
                }
            }
            std::cout << "[CandidatesGenerator] Generated " << candidates.size() << " grasp candidates.\n";

            return candidates;
        }

        std::vector<std::unique_ptr<HandSet>>
        CandidatesGenerator::generateGraspCandidateSets(const util::Cloud &cloud_cam)
        {
            // Find sets of grasp candidates.
            std::vector<std::unique_ptr<HandSet>> hand_set_list =
                hand_search_->searchHands(cloud_cam);

            return hand_set_list;
        }

        std::vector<int> CandidatesGenerator::reevaluateHypotheses(
            const util::Cloud &cloud, std::vector<std::unique_ptr<Hand>> &grasps)
        {
            return hand_search_->reevaluateHypotheses(cloud, grasps);
        }

    } // namespace candidate
} // namespace grasp_detect
