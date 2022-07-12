/*
 * @Description: LIO localization frontend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include "lidar_localization/matching/front_end/matching_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    //
    // subscribers:
    // 
    // a. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/IMU_data_CH110", 1000000);
    // b. lidar pose in map frame:
    // gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    //
    // publishers:
    // 
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    
    // d. estimated lidar pose in map frame, lidar frontend:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odometry", "/map", "/lidar", 100);
    // e. estimated lidar pose in map frame, map matching:
    map_matching_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/map_matching_odometry", "/map", "/lidar", 100);

    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() {
    // update global map if necessary:
    if (
        matching_ptr_->HasNewGlobalMap() && 
        global_map_pub_ptr_->HasSubscribers()
    ) {
        global_map_pub_ptr_->Publish( matching_ptr_->GetGlobalMap() );
    }

    // update local map if necessary:
    if (
        matching_ptr_->HasNewLocalMap() && 
        local_map_pub_ptr_->HasSubscribers()
    ) {
        local_map_pub_ptr_->Publish( matching_ptr_->GetLocalMap() );
    }

    // read inputs:
    ReadData();

    while( HasData() ) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

        if ( UpdateMatching() ) {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData() {
    // pipe lidar measurements and reference pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    // std::cout << " cloud_data_buff_ size : " << cloud_data_buff_.size() << std::endl;
    return true;
}

bool MatchingFlow::HasData() {
    if ( cloud_data_buff_.empty() )
        return false;
    
    if (imu_data_buff_.empty())
        return false;

    if ( matching_ptr_->HasInited() )
        return true;

    return true;
}

bool MatchingFlow::ValidData() {
    
    current_cloud_data_ = cloud_data_buff_.front();
    
    if  ( matching_ptr_->HasInited() ) {
        cloud_data_buff_.pop_front();
        measurement_ = GetMeasurement();
        // std::cout << " measurement imu size : " << measurement_[0].first.size() << " cloud time : " << measurement_[0].second.time << std::endl;
        return true;
    }
    
    
    //避免第二帧激光雷达之前累积了太多了imu数据 / 第一帧是在重定位
    GetMeasurement();
    cloud_data_buff_.pop_front();

    return true;
}

bool MatchingFlow::UpdateMatching() {

     if (!matching_ptr_->HasInited()) {
        // first try to init using scan context query:
        if (
            matching_ptr_->SetScanContextPose(current_cloud_data_)
        ) {
            Eigen::Matrix4f init_pose = matching_ptr_->GetInitPose();

            // evaluate deviation from GNSS/IMU:
            float deviation = (
                init_pose.block<3, 1>(0, 3) - current_gnss_data_.pose.block<3, 1>(0, 3)
            ).norm();

            // prompt:
            LOG(INFO) << "Scan Context Localization Init Succeeded. Deviation between GNSS/IMU: " 
                      << deviation
                      << std::endl;
        } 
        // if failed, fall back to GNSS/IMU init:
        else {
            matching_ptr_->SetGNSSPose(current_gnss_data_.pose);

            LOG(INFO) << "Scan Context Localization Init Failed. Fallback to GNSS/IMU." 
                      << std::endl;
        }
    }

    return matching_ptr_->Update(
        current_cloud_data_, 
        measurement_,
        laser_odometry_, map_matching_odometry_
    );
}

bool MatchingFlow::PublishData() {
    const double &timestamp_synced = current_cloud_data_.time;
    
     laser_odom_pub_ptr_->Publish(laser_odometry_, timestamp_synced);
    
    map_matching_odom_pub_ptr_->Publish(map_matching_odometry_, timestamp_synced);

    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    // laser_tf_pub_ptr_->SendTransform(laser_odometry_, timestamp_synced);
    return true;
}


bool MatchingFlow::Save(){
    matching_ptr_->Save();
}


bool MatchingFlow::MissMatch(){
    matching_ptr_->MissMatch();
}
    
/**
 * @brief 获取对应的lidar和imu数据集合 用于做积分
 * 
 */
LidarMeasurement MatchingFlow::GetMeasurement(){

    LidarMeasurement measurenment_;
    std::deque<IMUData> imu_msgs;
    const double lidar_time = current_cloud_data_.time;
    while(!imu_data_buff_.empty()){
        current_imu_data_ = imu_data_buff_.front();
        const double imu_time = current_imu_data_.time;
        if(imu_time <= lidar_time){ 
            imu_msgs.push_back(current_imu_data_);
            imu_data_buff_.pop_front();
        }
        else{
            break;
        }        
    }
    
    measurenment_.emplace_back(imu_msgs, current_cloud_data_);

    return measurenment_;
}

} // namespace lidar_localization
