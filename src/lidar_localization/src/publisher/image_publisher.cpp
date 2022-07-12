/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/image_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
ImagePublisher::ImagePublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise(topic_name, buff_size);
}

void ImagePublisher::Publish(ImageData::IMAGE_MAT&  image_mat, double time) {
    ros::Time ros_time(time);
    PublishData(image_mat, ros_time);
}

void ImagePublisher::Publish(ImageData::IMAGE_MAT&  image_mat) {
    ros::Time time = ros::Time::now();
    PublishData(image_mat, time);
}

void ImagePublisher::PublishData(ImageData::IMAGE_MAT&  image_mat, ros::Time time) {

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_mat).toImageMsg();

        image_msg->header.stamp = time;
        image_msg->header.frame_id = frame_id_;

        publisher_.publish(image_msg);
}

bool ImagePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization