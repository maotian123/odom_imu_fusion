/*
 * @Description: frontend node for lio localization
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/matching/front_end/matching_flow.hpp"
#include <lidar_localization/saveScanContext.h>

#include "std_msgs/Bool.h"
using namespace lidar_localization;


bool save_scan_context = false;
bool miss_matching_signal = false;
bool SaveScanContextCb(saveScanContext::Request &request, saveScanContext::Response &response) {
    save_scan_context = true;
    response.succeed = true;
    return response.succeed;
}

void MissMatchingSignal(const std_msgs::Bool::ConstPtr &msg){
    miss_matching_signal = true;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lio_matching_node");
    ros::NodeHandle nh;

    // subscribe to:
    //     a. undistorted lidar measurements
    //     b. GNSS position

    // publish:
    //     a. relative pose estimation
    //     b. map matching estimation
    // this provides input to sliding window backend
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);


     // register service for scan context save:
    ros::ServiceServer service = nh.advertiseService("save_scan_context", SaveScanContextCb);

    ros::Subscriber miss_matching_sub = nh.subscribe("miss_matching_signal",1,&MissMatchingSignal);
    
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        if (save_scan_context) {
            save_scan_context = false;
            matching_flow_ptr->Save();
        }

        if(miss_matching_signal){
            miss_matching_signal = false;
            matching_flow_ptr->MissMatch();

        }
        rate.sleep();
    }

    return 0;
}
