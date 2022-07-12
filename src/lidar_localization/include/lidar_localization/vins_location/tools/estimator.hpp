/*
*
*@descriptor:  预积分 以及 直接法 的一些东西
*/
#ifndef LIDAR_LOCALIZATION_ESTIMATOR_HPP_
#define LIDAR_LOCALIZATION_ESTIMATOR_HPP_


#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include <opencv2/core/eigen.hpp>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/image_data.hpp"
//tools
//pre integratio
#include "lidar_localization/vins_location/tools/pre_integration.hpp"
//feature_per fram
#include "lidar_localization/vins_location/tools/per_frame_featuresInfo.hpp"
//yaml
#include <yaml-cpp/yaml.h>
//
#include "lidar_localization/vins_location/tools/core.hpp"
//imageframe
#include "lidar_localization/vins_location/tools/image_frame.hpp"
//utility
#include "lidar_localization/vins_location/tools/utility.h"
#include "lidar_localization/vins_location/tools/initial_sfm.h"


namespace lidar_localization{
    const int WINDOW_SIZE = 10;
    class Estimator{
        public:

            Estimator();

            bool processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
            bool processImage(const std::map<int , Eigen::Matrix<double, 7, 1>>  &feature_, double time);
        public:
            std::map<double, ImageFrame> all_image_frame;
        private:
            bool InitWithYamlConfig();
            bool addFeatureCheckParallax(const std::map<int , Eigen::Matrix<double, 7, 1>>  &feature_);
            double compensatedParallax2(const PerFrameFeaturesInfo& it_per_id);
            bool initialStructure();
            bool clearState();
            bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
            bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation);
            //边缘化
            bool slideWindow();
            //边缘化次新的帧
            bool slideWindowNew();
            
        private:
            
            //每一帧特征点的信息
            std::list<PerFrameFeaturesInfo> feature_list;

            //Imu预积分
            PreIntergration* pre_integrations[(WINDOW_SIZE + 1)];
            PreIntergration* temp_intergration;

            double Images_time[(WINDOW_SIZE+1)];
            Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
            Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
            Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
            Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];

            std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
            std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
            std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

            //当前图像帧的数量
            int  current_frame_size = 0;

            // noise 
            double ACC_N = 0;
            double GYR_N = 0;
            Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
            double ACC_W = 0;
            double GYR_W = 0;

            //边缘化标志
            bool marginalization_flag;

             bool first_imu = false;
             Eigen::Vector3d acc_0;
             Eigen::Vector3d gyr_0;

             double MIN_PARALLAX = 10.0/702.0;

            //图像外参
            Eigen::Matrix3d ric;
            Eigen::Vector3d tic;


            //线性化初始化以及 非线性优化
            SolverFlag solver_flag;
    };
}

#endif