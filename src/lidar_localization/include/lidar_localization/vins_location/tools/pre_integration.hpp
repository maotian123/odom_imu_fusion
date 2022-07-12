/*
* @desciptor : 主要用与预积分
*/


#ifndef LIDAR_LOCALIZATION_VINS_LOCATION_FRONT_END_PRE_INTEGRATION_HPP_
#define  LIDAR_LOCALIZATION_VINS_LOCATION_FRONT_END_PRE_INTEGRATION_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>
namespace lidar_localization
{
    class PreIntergration{
        
        public:
            PreIntergration() = default;
            PreIntergration(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
            const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg,
            const double ACC_N, const double GYR_N,const double ACC_W ,const double GYR_W);
    
            void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1);
            

            void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);
    
            void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
        public:
            double dt;
            Eigen::Vector3d acc_0, gyr_0;
            Eigen::Vector3d acc_1, gyr_1;

            const Eigen::Vector3d linearized_acc, linearized_gyr;
            Eigen::Vector3d linearized_ba, linearized_bg;

            Eigen::Matrix<double, 15, 15> jacobian, covariance;
            Eigen::Matrix<double, 15, 15> step_jacobian;
            Eigen::Matrix<double, 15, 18> step_V;
            Eigen::Matrix<double, 18, 18> noise;

            double sum_dt;
            Eigen::Vector3d delta_p;
            Eigen::Quaterniond delta_q;
            Eigen::Vector3d delta_v;

            std::vector<double> dt_buf;
            std::vector<Eigen::Vector3d> acc_buf;
            std::vector<Eigen::Vector3d> gyr_buf;
    };

    
} // namespace lida


#endif
