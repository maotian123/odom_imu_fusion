#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/vins_location/front_end/vins_location_flow.hpp"

#include <thread>

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "vins_location_node");
    ros::NodeHandle nh;

    // subscribe to:
    //     a. undistorted lidar measurements
    //     b. GNSS position

    // publish:
    //     a. relative pose estimation
    //     b. map matching estimation
    // this provides input to sliding window backend
    std::shared_ptr<VinsLocationFlow> vins_location_flow_ptr = std::make_shared<VinsLocationFlow>(nh);

    std::thread thd_HandleImageData(&VinsLocationFlow::HandleImageData, vins_location_flow_ptr);
   // thd_HandleImageData.join();
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        vins_location_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
