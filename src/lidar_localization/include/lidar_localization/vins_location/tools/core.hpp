/*
* @ descriptor : 一些比较好的stuct typedef
*/
#ifndef LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_CORE_HPP
#define LIDAR_LOCALIZATION_VINS_LOCALIZATION_TOOLS_CORE_HPP

#include "lidar_localization/sensor_data/image_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
namespace lidar_localization{


    
    struct Pts_Info_frame{
          
          Eigen::Matrix<double,7 ,1> pts_info;
          int frame;

    };
    // typedef Eigen::Matrix<double,7 ,1> feature_info;
    struct IMG_MSG{
        double img_time;
        double imu_time;
        std::vector<Eigen::Vector3d> points;
        std::vector<int> id_of_points;
        std::vector<float> u_of_points;
        std::vector<float> v_of_points;
        std::vector<float> velocity_x_of_points;
        std::vector<float> velocity_y_of_points;
  };

    typedef std::vector<std::pair<std::vector<IMUData>,std::shared_ptr<IMG_MSG>>> vins_measurement;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
}

#endif
