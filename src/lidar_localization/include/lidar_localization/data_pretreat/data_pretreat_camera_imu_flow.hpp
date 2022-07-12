/*s
 * @Description: 数据预处理模块，包括时间同步(视觉和imu)
 * @Author: Tian Yi Hong
 * @Date: 2022-05-5 17:57:22
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_CAMERA_IMU_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_CAMERA_IMU_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/image_subscriber.hpp"
// publisher
//a. synced camera measurement
// b. synced IMU measurement
#include "lidar_localization/publisher/imu_publisher.hpp"
#include "lidar_localization/publisher/image_publisher.hpp"

namespace lidar_localization {
class DataPretreatCameraImuFlow {
  public:
    DataPretreatCameraImuFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool PublishData();

	private:
		// subscriber
		std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<ImageSubscriber> image_sub_ptr_;
        // publisher
        std::shared_ptr<IMUPublisher> imu_pub_ptr_;
        std::shared_ptr<ImagePublisher> image_pub_ptr_;
        

        std::deque<IMUData> imu_data_buff_;
        std::deque<ImageData> image_data_buff_;
        
        IMUData current_imu_data_;
        ImageData current_image_data_;

        
        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif
