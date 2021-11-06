#include "grasp_detect/grasp_dataset_generate.h"

namespace grasp_detect
{
    GraspDatasetGenerate::GraspDatasetGenerate(ParamsManager &params) : params_(params),
                                                                        plotter_(params)
    {
        candidates_generator_ = std::make_unique<grasp_detect::candidate::CandidatesGenerator>(params_);
    }

    GraspDatasetGenerate::~GraspDatasetGenerate()
    {
    }

    void GraspDatasetGenerate::preprocessCloud(grasp_detect::util::Cloud &cloud)
    {
        candidates_generator_->preprocessPointCloud(cloud);
    }

    std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> GraspDatasetGenerate::generateGraspCandidates(const grasp_detect::util::Cloud &cloud_cam)
    {
        return candidates_generator_->generateGraspCandidates(cloud_cam);
    }

    std::vector<std::unique_ptr<grasp_detect::candidate::HandSet>> GraspDatasetGenerate::generateGraspCandidateSets(const grasp_detect::util::Cloud &cloud_cam)
    {
        return candidates_generator_->generateGraspCandidateSets(cloud_cam);
    }

    void GraspDatasetGenerate::saveGrasp(std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> &hand_list, std::string file_name)
    {
        std::cout << "[GraspDatasetGenerate] save grasp to " << file_name << std::endl;

        ofstream OutFile(file_name);
        for (int i = 0; i < hand_list.size(); i++)
        {
            if (hand_list[i]->isFullAntipodal()) //|| hand_list[i]->isHalfAntipodal()
            {
                auto center = hand_list[i]->getPosition();
                auto rot = hand_list[i]->getOrientation();
                auto approach = hand_list[i]->getApproach();
                auto binormal = hand_list[i]->getBinormal();
                auto axis = hand_list[i]->getAxis();
                float score = hand_list[i]->getScore();
                // OutFile << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                //         << rot(0, 0) << " " << rot(0, 1) << " " << rot(0, 2) << " "
                //         << rot(1, 0) << " " << rot(1, 1) << " " << rot(1, 2) << " "
                //         << rot(2, 0) << " " << rot(2, 1) << " " << rot(2, 2) << " " << endl;
                OutFile << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                        << approach(0) << " " << approach(1) << " " << approach(2) << " "
                        << binormal(0) << " " << binormal(1) << " " << binormal(2) << " "
                        << axis(0) << " " << axis(1) << " " << axis(2) << " " << score << endl;
            }
        }
        OutFile.close();
    }

    void GraspDatasetGenerate::saveGrasp(std::vector<std::unique_ptr<grasp_detect::candidate::HandSet>> &hand_set_list,
                                         std::string grasp_file_name,
                                         std::string pcd_file_name,
                                         std::string point_file_name, int num)
    {
        std::cout << "[GraspDatasetGenerate] save grasp to " << grasp_file_name << std::endl;
        // std::cout << "[GraspDatasetGenerate] save grasp pcd to " << pcd_file_name << std::endl;
        pcl::PointCloud<pcl::PointXYZ> save_cloud;

        ofstream OutFile(grasp_file_name);
        ofstream OutFile2(point_file_name);

        std::vector<int> count(5);
        int padding_count = num - hand_set_list.size();
        // Eigen::Vector3d pre_center;
        // Eigen::Vector3d pre_approach;
        // Eigen::Vector3d pre_closure;
        // Eigen::Vector3d pre_vertical;
        // double pre_score;
        for (int i = 0; i < hand_set_list.size(); i++)
        {
            // std::cout << "[hand_set_list.size()] " << hand_set_list.size() << std::endl;
            std::vector<std::unique_ptr<grasp_detect::candidate::Hand>> &hand_list = hand_set_list.at(i)->getHands();
            for (int j = 0; j < hand_list.size(); j++)
            {
                auto sample = hand_list[j]->getSample();
                pcl::PointXYZ point(sample[0], sample[1], sample[2]);
                save_cloud.push_back(point);

                auto center = hand_list[j]->getPosition();
                // auto rot = hand_list[j]->getOrientation();
                auto approach = hand_list[j]->getApproach();
                auto closure = hand_list[j]->getBinormal();
                auto vertical = hand_list[j]->getAxis();
                float score = hand_list[j]->getScore();
                float width = hand_list[j]->getGraspWidth();
                // pre_center = center;
                // pre_approach = approach;
                // pre_closure = closure;
                // pre_vertical = vertical;
                // pre_score = score;
                if (hand_set_list.at(i)->getIsValid()[j] == false)
                    count[0]++;
                else if (score > 0.9)
                    count[4]++;
                else if (score > 0.7)
                    count[3]++;
                else if (score > 0.5)
                    count[2]++;
                else
                    count[1]++;

                if (std::isnan(center(0)) || std::isinf(center(0)))
                {
                    std::cout << "[error] number is nan" << std::endl;
                    std::cout << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                              << approach(0) << " " << approach(1) << " " << approach(2) << " "
                              << closure(0) << " " << closure(1) << " " << closure(2) << " "
                              << vertical(0) << " " << vertical(1) << " " << vertical(2) << " " << score << " " << width << endl;
                    OutFile << setprecision(4) << point.x << " " << point.y << " " << point.z << " "
                            << 0 << " " << 0 << " " << 1 << " "
                            << 1 << " " << 0 << " " << 0 << " "
                            << 0 << " " << 1 << " " << 0 << " " << 0 << " " << 0 << endl;
                }
                else if (!hand_set_list.at(i)->getIsValid()[j])
                {
                    OutFile << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                            << approach(0) << " " << approach(1) << " " << approach(2) << " "
                            << closure(0) << " " << closure(1) << " " << closure(2) << " "
                            << vertical(0) << " " << vertical(1) << " " << vertical(2) << " " << score << " " << width << endl;
                }
                else
                {
                    OutFile << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                            << approach(0) << " " << approach(1) << " " << approach(2) << " "
                            << closure(0) << " " << closure(1) << " " << closure(2) << " "
                            << vertical(0) << " " << vertical(1) << " " << vertical(2) << " " << score << " " << width << endl;
                }

                if (padding_count != 0)
                {
                    OutFile << setprecision(4) << center(0) << " " << center(1) << " " << center(2) << " "
                            << approach(0) << " " << approach(1) << " " << approach(2) << " "
                            << closure(0) << " " << closure(1) << " " << closure(2) << " "
                            << vertical(0) << " " << vertical(1) << " " << vertical(2) << " " << score << " " << width << endl;
                    OutFile2 << point.x << " " << point.y << " " << point.z << endl;
                    padding_count--;
                }

                OutFile2 << point.x << " " << point.y << " " << point.z << endl;
            }
        }
        std::cout << "[GraspDatasetGenerate] score = 0 grasp num is " << count[0] << endl;
        std::cout << "[GraspDatasetGenerate] score > 0.1 grasp num is " << count[1] << endl;
        std::cout << "[GraspDatasetGenerate] score > 0.5 grasp num is " << count[2] << endl;
        std::cout << "[GraspDatasetGenerate] score > 0.7 grasp num is " << count[3] << endl;
        std::cout << "[GraspDatasetGenerate] score > 0.9 grasp num is " << count[4] << endl;

        if (pcd_file_name != "")
            pcl::io::savePCDFile(pcd_file_name, save_cloud);

        OutFile.close();
        OutFile2.close();
    }
} // namespace grasp_detect
