/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscribers:
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/IMU_data_CH110", 1000000);

    // publishers:
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    

}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;
    
    while(HasData()) {
    

        if (!ValidData())
            continue;

        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    static std::deque<IMUData> unsynced_imu_;

    // fetch lidar measurements from buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);

    
    if (cloud_data_buff_.size() == 0)
        return false;
    
    // use timestamp of lidar measurement as reference:
    //double cloud_time = cloud_data_buff_.front().time;

    //bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    // static bool sensor_inited = false;
    // if (!sensor_inited) {
    //     if (!valid_imu) {
    //         cloud_data_buff_.pop_front();
    //         return false;
    //     }
    //     sensor_inited = true;
    // }

    return true;
}


bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;


    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    // printf("diff_imu_time : %f : \n", diff_imu_time);
    // printf("current_cloud_data_.time : %f\n",current_cloud_data_.time);
    // printf("current_imu_data_.time : %f\n",current_imu_data_.time);
    //
    // this check assumes the frequency of lidar is 10Hz:
    //
    if (diff_imu_time < -0.005) {
        printf("hope this thing not happen \n");
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.005) {
        imu_data_buff_.pop_front();
        return false;
    }
    // printf("diff imu time : %f\n", diff_imu_time);

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
  
    return true;
}

bool DataPretreatFlow::PublishData() {
    // take lidar measurement time as synced timestamp:
    const double &timestamp_synced = current_cloud_data_.time;
    
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, timestamp_synced);
    imu_pub_ptr_->Publish(current_imu_data_, current_imu_data_.time);

    
    return true;
}

}
