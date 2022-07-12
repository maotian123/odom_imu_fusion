
#ifndef LIDAR_LOCALIZATION_VINS_LOCATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_VINS_LOCATION_FLOW_HPP_

#include <ros/ros.h>

// subscriber
#include "lidar_localization/subscriber/image_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/odometry_publisher.hpp"

// estimator
#include "lidar_localization/vins_location/tools/estimator.hpp"

// feature tacker
#include "lidar_localization/vins_location/tools/feature_tracker.hpp"

//typedef struct

#include "lidar_localization/vins_location/tools/core.hpp"

//thread
#include <thread>
#include <time.h>
#include <map>


namespace lidar_localization {
    
class VinsLocationFlow {
  public:
    VinsLocationFlow(ros::NodeHandle& nh);
   
    bool Run();
    void HandleImageData();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();

    bool ProcessBackend();

    vins_measurement VinsMeasurementGet();
    bool VinsInitial();


    //some debug tools
    void print_measurement(const vins_measurement&  measurements);
  private:
    //
    // estimator
    //
    //
    //tools 主要用于imu估计
    std::shared_ptr<Estimator> estimator_ptr_;
    std::shared_ptr<FeatureTraker> feature_tracker_ptr_;

    // subscribers:
    //  
    std::shared_ptr<ImageSubscriber> image_sub_sync_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_sync_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;

    
    


    std::deque<std::shared_ptr<IMG_MSG>> img_features_buff_; //只可以读
    //
    //
    //
    std::deque<ImageData> image_data_sync_buff_;
    std::deque<IMUData> imu_data_sync_buff_;
    std::deque<IMUData> imu_data_buff_;
    
    IMUData current_imu_data;
    ImageData current_image_data;

    //imu image 组合
    
    //
    // publishers:
    //
   

    
    //thread
    std::mutex feature_buf;
    std::mutex estimator_buf;

     
   
};

} // namespace lidar_localization

#endif 
