#include "lidar_localization/subscriber/image_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
ImageSubscriber::ImageSubscriber(ros::NodeHandle & nh, std::string topic_name,size_t buff_size, int image_type) 
    :nh_(nh) {
    //等于0则是普通图像
    if(image_type == 0){
        subscriber_ = nh_.subscribe(topic_name,image_type,&ImageSubscriber::Nimage_msg_Callback,this);
    }
    else{
        subscriber_ = nh_.subscribe(topic_name,image_type,&ImageSubscriber::Cimagemsg_Callback,this);
    }

}

    void ImageSubscriber::Nimage_msg_Callback(const sensor_msgs::ImageConstPtr& msg_image){
        buff_mutex_.lock();
        ImageData image_data;
        image_data.image_ptr = cv_bridge::toCvCopy(msg_image, "bgr8");
    
        image_data.image_mat = image_data.image_ptr->image.clone();
       // ImageData::IMAGE_PTR image = cv_bridge::toCvCopy(msg_image, "bgr8");

       image_data.time = msg_image->header.stamp.toSec();
       // image->header.stamp = msg_image->header.stamp;

        new_image_data.push_back(image_data);
        buff_mutex_.unlock();

    }

    //压缩图像的保存
    void ImageSubscriber::Cimagemsg_Callback(const sensor_msgs::CompressedImagePtr &msg_compressed_image){
        buff_mutex_.lock();
        std::cout << "come in" << std::endl;
        ImageData image_data;
        image_data.image_ptr = cv_bridge::toCvCopy(msg_compressed_image, "bgr8");
        
         image_data.image_mat = image_data.image_ptr->image.clone();
       // ImageData::IMAGE_PTR image = cv_bridge::toCvCopy(msg_image, "bgr8");
        /*
   
        */
       image_data.time = msg_compressed_image->header.stamp.toSec();
       // image->header.stamp = msg_image->header.stamp;

        new_image_data.push_back(image_data);
        buff_mutex_.unlock();
    }

    void ImageSubscriber::ParseData( std::deque<ImageData>& deque_image_data){
        buff_mutex_.lock();
        if(new_image_data.size() > 0){
            deque_image_data.insert(deque_image_data.end(),new_image_data.begin(),new_image_data.end());
            new_image_data.clear();
        }

        buff_mutex_.unlock();

    }
}