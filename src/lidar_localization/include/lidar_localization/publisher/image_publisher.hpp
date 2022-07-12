#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMAGE_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_IMAGE_PUBLISHER_HPP_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include "lidar_localization/sensor_data/image_data.hpp"


namespace lidar_localization {
class ImagePublisher {
  public:
    ImagePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    ImagePublisher() = default;

    void Publish(ImageData::IMAGE_MAT& image_mat, double time);
    void Publish(ImageData::IMAGE_MAT& image_mat);

    bool HasSubscribers();
  
  private:
    void PublishData(ImageData::IMAGE_MAT& image_mat, ros::Time time);

  private:
     image_transport::ImageTransport nh_;
    image_transport::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif