#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMAGE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMAGE_DATA_HPP_

#include <cv_bridge/cv_bridge.h>

namespace lidar_localization
{
    class ImageData{
        public:
            using IMAGE_PTR = cv_bridge::CvImagePtr;
            using IMAGE_MAT = cv::Mat;
        public:
            IMAGE_PTR image_ptr;
            
            IMAGE_MAT image_mat;
            double time = 0; 
    };
} // namespace backwattman
#endif