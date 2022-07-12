/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */

#include "lidar_localization/data_pretreat/data_pretreat_camera_imu_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatCameraImuFlow::DataPretreatCameraImuFlow(ros::NodeHandle& nh) {
    // subscribers:
    image_sub_ptr_ = std::make_shared<ImageSubscriber>(nh, "/zed/zed_node/rgb/image_rect_color", 100, 0);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 100);
    // b. OXTS IMU:

    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    image_pub_ptr_ = std::make_shared<ImagePublisher>(nh, "/synced_image", "/image_link",100);

}

bool DataPretreatCameraImuFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        PublishData();
    }

    return true;
}

bool DataPretreatCameraImuFlow::ReadData() {
    
    
    imu_sub_ptr_->ParseData(imu_data_buff_);
    image_sub_ptr_->ParseData(image_data_buff_);

    if(image_data_buff_.size() == 0)
        return false;
        
    return true;
}


bool DataPretreatCameraImuFlow::HasData() {
  
    if (imu_data_buff_.size() == 0)
        return false;
    
    if (image_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatCameraImuFlow::ValidData() {
   current_image_data_ = image_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();

    double diff_imu_time = current_image_data_.time - current_imu_data_.time;
    //
    // this check assumes the frequency of lidar is 10Hz:
    //
    if (diff_imu_time < -0.005) {
        image_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.005) {
        imu_data_buff_.pop_front();
        return false;
    }

    image_data_buff_.pop_front();
    imu_data_buff_.pop_front();
  

    return true;
}

bool DataPretreatCameraImuFlow::PublishData() {
    // const double &timestamp_synced = current_image_data_.time;

    imu_pub_ptr_->Publish(current_imu_data_,current_imu_data_.time);
    image_pub_ptr_->Publish(current_image_data_.image_mat,current_image_data_.time);
    return true;
}

}
