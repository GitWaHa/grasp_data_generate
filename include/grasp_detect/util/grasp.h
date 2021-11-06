#pragma once

#include <iostream>
#include <vector>

namespace grasp_detect
{
    namespace util
    {
        struct Grasp
        {
            std::vector<double> score;
            std::vector<std::vector<double>> center;   //grasp 中心点坐标
            std::vector<std::vector<double>> approach; //grasp approach 接近方向
            std::vector<std::vector<double>> closure;  //grasp closure 闭合方向
            std::vector<std::vector<double>> vertical; //grasp vertical 与　approach、closure　垂直方向
        };
    } // namespace util
} // namespace grasp_detect
