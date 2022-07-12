#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <ros/ros.h>


#include <deque>
#include <mutex>
#include <thread>

#include <iostream>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

//sensor data
#include   "lidar_localization/sensor_data/image_data.hpp"


namespace lidar_localization{

    class ImageSubscriber{
        public:
            ImageSubscriber( ros::NodeHandle & nh, std::string topic_name,size_t buff_size, int image_type);

            ImageSubscriber()= default;
            void ParseData( std::deque<ImageData>& deque_image_data);
        
        private:
            void Nimage_msg_Callback(const sensor_msgs::ImageConstPtr& msg_image);
            void Cimagemsg_Callback(const sensor_msgs::CompressedImagePtr &msg_compressed_image);
        private:
            ros::NodeHandle nh_;
            ros::Subscriber subscriber_;
            /*         
                更换成了CvimagePtr 自带时间戳
            */
            std::deque<ImageData>  new_image_data;
            std::mutex buff_mutex_;
    };

}
#endif