/*
* @ descriptor ：用于记录所有图片识别的特征点相关信息
*/

#ifndef LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_PER_FRAME_FEATURESIFO_HPP
#define LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_PER_FRAME_FEATURESIFO_HPP
#include <iostream>
#include "lidar_localization/vins_location/tools/core.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
namespace lidar_localization
{
    class PerFrameFeaturesInfo{
        
        public:
            PerFrameFeaturesInfo(const int& _feature_id, const int& _start_frame):
            feature_id(_feature_id),
            start_frame(_start_frame)
            {
                                
            }
        
        public:
            int feature_id = -1;
            int start_frame = -1;
            std::vector<Pts_Info_frame> per_features_info;

        
    };
} // namespace 




#endif