#include "grasp_detect/pointnet_grasp_client.h"
#include "grasp_detect/PointNetGrasp.h"

namespace grasp_detect
{

    PointNetGraspClient::PointNetGraspClient(std::string service_name)
    {
        client_ = nh_.serviceClient<grasp_detect::PointNetGrasp>(service_name);
    }
    PointNetGraspClient::~PointNetGraspClient()
    {
    }

    bool PointNetGraspClient::callService(std::vector<std::vector<double>> &points, grasp_detect::util::Grasp &grasps, double min_score, int min_num, int max_num)
    {
        std::cout << "[PointNetGraspClient] ----------- client is called----------" << std::endl;

        grasp_detect::PointNetGrasp::Request requst;
        requst.header.frame_id = "base_link";
        requst.min_score = min_score;
        requst.min_num = min_num;
        requst.max_num = max_num;
        requst.points.resize(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            requst.points[i].x = points[i][0];
            requst.points[i].y = points[i][1];
            requst.points[i].z = points[i][2];
        }

        grasp_detect::PointNetGrasp::Response response;

        bool is_ok = client_.call(requst, response);

        if (!is_ok)
        {
            return false;
            ROS_ERROR("client is not ok");
        }

        int grasp_size = response.approach.size();
        grasps.score.reserve(grasp_size);
        grasps.center.reserve(grasp_size);
        grasps.approach.reserve(grasp_size);
        grasps.closure.reserve(grasp_size);
        grasps.vertical.reserve(grasp_size);

        for (int i = 0; i < grasp_size; i++)
        {
            grasps.score.push_back(response.score[i]);
            grasps.center.push_back(std::vector<double>{response.center[i].x, response.center[i].y, response.center[i].z});
            grasps.approach.push_back(std::vector<double>{response.approach[i].x, response.approach[i].y, response.approach[i].z});
            grasps.closure.push_back(std::vector<double>{response.closure[i].x, response.closure[i].y, response.closure[i].z});
            grasps.vertical.push_back(std::vector<double>{response.vertical[i].x, response.vertical[i].y, response.vertical[i].z});
        }

        std::cout << "[PointNetGraspClient] valid grasp num is " << grasps.score.size() << std::endl;

        return true;
    }

} // namespace grasp_detect