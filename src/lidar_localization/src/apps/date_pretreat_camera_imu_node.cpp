/*
 * @Description: ROS node for sensor measurement pre-processing
 * @Author: Ren Qian
 * Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"
#include "lidar_localization/data_pretreat/data_pretreat_camera_imu_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include <stdio.h>

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;
     
    ros::init(argc, argv, "data_pretreat_camera_imu_node");
    ros::NodeHandle nh;
    std::string cloud_topic;
 
     std::shared_ptr<DataPretreatCameraImuFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatCameraImuFlow>(nh);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}
