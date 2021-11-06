#include <iostream>
#include <ros/ros.h>

#include <std_srvs/SetBool.h>
#include "tsdf_fusion/tsdf_cuda.cuh"

class Demo
{
public:
    Demo(std::string service_name);
    ~Demo();

    bool serverCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    /* data */
    ros::NodeHandle nh_;
    ros::ServiceServer server_;
};

Demo::Demo(std::string service_name)
{
    // 不使用bind，模板函数可以自动推导参数类型
    server_ = nh_.advertiseService(service_name, &Demo::serverCallBack, this);

    // 使用bind，必须指定参数类型
    // server_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(service_name, boost::bind(&Demo::serverCallBack, this, _1, _2));
}
Demo::~Demo()
{
}

bool Demo::serverCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    std::cout << "-----------tsdf server start----------" << std::endl;

    std::cout << req << std::endl;

    res.success = true;
    res.message = "is ok";
    TSDF_Fusion();

    std::cout << "-----------tsdf server end----------" << std::endl;

    return true;
}

int main(int argn, char **argv)
{
    ros::init(argn, argv, "server_demo");

    Demo demo("test_service_server");

    ros::spin();
    return 0;
}