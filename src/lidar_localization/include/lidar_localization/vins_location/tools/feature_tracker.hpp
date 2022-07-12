/*
* @descripter : 用于图像的特征提取
*/
#ifndef LIDAR_LOCALLIZATION_VINS_LOCALIZATION_TOOLS_FEATURE_TRAKER_HPP_
#define LIDAR_LOCALLIZATION_VINS_LOCALIZATION_TOOLS_FEATURE_TRAKER_HPP_
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/image_data.hpp"

#include <vector>
namespace lidar_localization
{
    const int MAX_CNT = 150;
    const int MIN_DIST = 30;
    
    class FeatureTraker{
        public:
            FeatureTraker() = default;

            void readImageData(const ImageData &image_msg);
            bool updateID(unsigned int i);

        private:
            
            
            void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
            void reduceVector(std::vector<int> &v, std::vector<uchar> status);
            void setMask();
            bool undistortedPoints();
            void rejectWithF();

            //归一化 去畸变
            bool liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d & P);
            bool distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d& d_u);

        public:
            cv::Mat mask;
            cv::Mat fisheye_mask;
            cv::Mat prev_img, cur_img;
            std::vector<cv::Point2f> n_pts;  // 好特征点的数量
            std::vector<cv::Point2f> prev_pts, cur_pts;
            std::vector<cv::Point2f> prev_un_pts, cur_un_pts;
            std::vector<cv::Point2f> pts_velocity;
            std::vector<int> feature_ids;
            std::vector<int> track_cnt;
            std::map<int, cv::Point2f> cur_un_pts_map;
            std::map<int, cv::Point2f> prev_un_pts_map;
        private:

            bool PUB_THIS_FRAME = true;

            
            
            double cur_time;
            double prev_time;


            int feature_num_id = 0;
    };
} // namespace 



#endif
