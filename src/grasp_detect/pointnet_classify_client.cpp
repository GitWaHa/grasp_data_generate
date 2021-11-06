#include "grasp_detect/pointnet_classify_client.h"
#include "grasp_detect/PointNetClassify.h"

namespace grasp_detect
{

    PointNetClassifyClient::PointNetClassifyClient(std::string service_name)
    {
        client_ = nh_.serviceClient<grasp_detect::PointNetClassify>(service_name);
    }
    PointNetClassifyClient::~PointNetClassifyClient()
    {
    }

    bool PointNetClassifyClient::callService(const Eigen::Matrix3Xd &points, int batch_size, std::vector<double> &scores)
    {
        std::cout << "[PointNetClassifyClient] ----------- client is called----------" << std::endl;

        grasp_detect::PointNetClassify::Request requst;
        requst.header.frame_id = "base_link";
        requst.points.resize(points.cols());
        requst.batch_size = batch_size;

        for (int i = 0; i < points.cols(); i++)
        {
            requst.points[i].x = points(0, i);
            requst.points[i].y = points(1, i);
            requst.points[i].z = points(2, i);
        }

        grasp_detect::PointNetClassify::Response response;

        bool is_ok = client_.call(requst, response);

        if (!is_ok)
        {
            ROS_ERROR("PointNetClassify server is not ok");
            return false;
        }

        int grasp_size = response.score.size();
        scores.resize(0);
        scores.reserve(grasp_size);

        for (int i = 0; i < grasp_size; i++)
        {
            scores.push_back(response.score.at(i));
        }

        std::cout << "[PointNetClassifyClient] re score grasp num is " << scores.size() << std::endl;

        return true;
    }

} // namespace grasp_detect