/*
  @ descriptor: 对应的图像以及预积分
 */
#ifndef LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_IMAGE_FRAME_HPP
#define LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_IMAGE_FRAME_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

#include"lidar_localization/vins_location/tools/core.hpp"
//pre_intergration
#include "lidar_localization/vins_location/tools/pre_integration.hpp"
namespace lidar_localization
{
    class ImageFrame{
        public:
            ImageFrame()=default;
            ImageFrame(const std::map<int , Eigen::Matrix<double, 7, 1>>& _points, double _time):
            points(_points),
            time(_time),
            is_key_frame(false)
            {

            }

        public:
            PreIntergration* pre_intergration;
            double time;
            std::map<int , Eigen::Matrix<double, 7, 1>>  points;
            Eigen::Matrix3d R;
            Eigen::Vector3d T;
            bool is_key_frame;
    };
} // namespace 


#endif