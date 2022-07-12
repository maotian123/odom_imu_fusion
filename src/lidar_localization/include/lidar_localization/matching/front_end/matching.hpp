/*
 * @Description: lidar localization frontend, interface
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
typedef std::vector<std::pair<std::deque<IMUData>,CloudData>> LidarMeasurement;
class Matching {
public:
    Matching();

    bool HasInited(void);
    bool HasNewGlobalMap(void);
    bool HasNewLocalMap(void);

    Eigen::Matrix4f GetInitPose(void);
    CloudData::CLOUD_PTR& GetGlobalMap(void);
    CloudData::CLOUD_PTR& GetLocalMap(void);
    CloudData::CLOUD_PTR& GetCurrentScan(void);

    bool Update(
      const CloudData& cloud_data, 
      const LidarMeasurement& measurement,
      Eigen::Matrix4f& laser_pose, Eigen::Matrix4f &map_matching_pose
    );

    bool UpdatePreposition(const LidarMeasurement& measurement, Eigen::Matrix4f&predict_pose);
     //imu intergrate tools 获取角度的偏差
    bool GetAngularDelta( const size_t index_curr, const size_t index_prev, Eigen::Vector3d &angular_delta);
    inline Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel);
 //更新旋转矩阵
    void UpdateOrientation( const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);
    bool GetVelocityDelta(
        const size_t index_curr, const size_t index_prev,
        const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
        double &delta_t, Eigen::Vector3d &velocity_delta
      );

    Eigen::Vector3d GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &R
    );
    void UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta);

    
    bool SetGNSSPose(const Eigen::Matrix4f& init_pose);
    bool SetScanContextPose(const CloudData& init_scan);

    bool Save();
    bool MissMatch();
private:
    bool InitWithConfig();
    // 
    // point cloud map & measurement processors:
    // 
    bool InitFilter(
      const YAML::Node &config_node, std::string filter_user, 
      std::shared_ptr<CloudFilterInterface>& filter_ptr
    );
    bool InitLocalMapSegmenter(const YAML::Node& config_node);
    bool InitPointCloudProcessors(const YAML::Node& config_node);
    //
    // global map:
    //
    bool InitGlobalMap(const YAML::Node& config_node);
    //
    // lidar frontend for relative pose estimation:
    //
    bool InitRegistration(const YAML::Node& config_node, std::shared_ptr<RegistrationInterface>& registration_ptr);
    //
    // map matcher:
    //
    bool InitScanContextManager(const YAML::Node& config_node);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool ResetLocalMap(float x, float y, float z);
    void RefreshState(const Eigen::Matrix4d &last_pose_,const Eigen::Matrix4d& current_pose_, const double& delte_time);
private:
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;

    std::string scan_context_index_path;
    bool record_scancontext = false;

    KeyFrame current_frame_;

    bool miss_matching_signal = false;

    std::deque<IMUData> imu_data_buff_;
     //imu bias
    Eigen::Vector3d angular_vel_bias_;
    // a. gravity constant:
    Eigen::Vector3d G_;

    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acc_bias_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_FRONTEND_MATCHING_HPP_