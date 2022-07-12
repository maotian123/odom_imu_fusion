#include "lidar_localization/vins_location/front_end/vins_location_flow.hpp"


#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"



namespace lidar_localization{


    VinsLocationFlow::VinsLocationFlow(ros::NodeHandle& nh){
        
        //subscriber
        image_sub_sync_ptr_ = std::make_shared<ImageSubscriber>(nh, "/synced_image", 100000, 0 );
        imu_sub_sync_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000 );
        imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 100000);
        //tools
        estimator_ptr_ = std::make_shared<Estimator>();
        feature_tracker_ptr_ = std::make_shared<FeatureTraker>();
        
    }
    void VinsLocationFlow::HandleImageData(){
                
                while (true)
                {
                    //printf("image_data_sync_buff_ size : %d\n", image_data_sync_buff_.size());
                    usleep(5000);    
                    const int num_image = image_data_sync_buff_.size();

                    // std::cout << " num_image : " << num_image << std::endl;
                    if(num_image <= 0){
                        continue;
                    }
                    
                    //printf("image_data_sync_buff_ size : %d\n", image_data_sync_buff_.size());
                    current_image_data = image_data_sync_buff_.front();
                    current_imu_data = imu_data_sync_buff_.front();
                    image_data_sync_buff_.pop_front();
                    imu_data_sync_buff_.pop_front();

                    static double last_img_time  = current_image_data.time;
                    double current_img_time = current_image_data.time;
                    
                    
                    double dt  = current_img_time - last_img_time;

                //    std::cout << " dt : " << dt << std::endl;
                    // if(dt > 0){
                    //     cv::imshow("test" , current_image_data.image_mat);
                    //     cv::waitKey(1);current_image_data.image_mat;
                    // }
                    if(dt <= 0)
                        continue;
                

                    feature_tracker_ptr_->readImageData(current_image_data);

                    //更新当前特征点的序列号
                    for(unsigned int i = 0;;i++){
                        bool completed = false;
                        completed |= feature_tracker_ptr_->updateID(i); 

                        if (!completed)
                            break;
                    }
                    
                    std::shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
                    feature_points->img_time = current_img_time;
                    //为了能够比较好的对齐 现在觉得date_pre_这个node写的好傻逼，但整体框架搭建成这样了，暂时不修改了
                    //目前本人也还没有找到一个比较好的对齐方案
                    feature_points->imu_time = current_imu_data.time; 
                    auto &current_undistored_pts = feature_tracker_ptr_->cur_un_pts;
                    auto &current_distored_uv = feature_tracker_ptr_->cur_pts;
                    auto &feature_ids = feature_tracker_ptr_->feature_ids;
                    auto &pts_velocity = feature_tracker_ptr_->pts_velocity;

                    //当前帧的所有特征点、像素坐标 、去畸变后的归一化坐标 、以及特征点速度
                    for(unsigned int i = 0; i < feature_ids.size(); ++i){
                        if(feature_tracker_ptr_->track_cnt[i] > 1){

                            int feature_id = feature_ids[i];
                            double x = current_undistored_pts[i].x;
                            double y = current_undistored_pts[i].y;
                            feature_points->id_of_points.push_back(feature_id);
                            feature_points->points.push_back(Eigen::Vector3d(x,y,1));
                            feature_points->u_of_points.push_back(current_distored_uv[i].x);
                            feature_points->v_of_points.push_back(current_distored_uv[i].y);
                            feature_points->velocity_x_of_points.push_back(pts_velocity[i].x);
                            feature_points->velocity_y_of_points.push_back(pts_velocity[i].y);   

                        }
                    }


                    //第一帧图片没有速度 故舍
                    static bool first_img = false;
                    if(!first_img){
                        first_img = true;
                    }
                    else{
                        feature_buf.lock();
                        //feature_buff_ 每帧的特征点集合
                        img_features_buff_.push_back(feature_points);
                        
                        feature_buf.unlock();
                    }
                    last_img_time = current_img_time;
                    // cv::imshow("thread1imgshow", img);
                    //  cv::waitKey(100);
                    
                    
                }
                

     }

    //TODO 
    bool  VinsLocationFlow::Run(){
        if(!ReadData())
            return false;
        
        while(HasData()){

              

            ProcessBackend();
            

            //VinsInitial();
        }
    }

    bool VinsLocationFlow::ReadData(){

        image_sub_sync_ptr_->ParseData(image_data_sync_buff_);
        imu_sub_sync_ptr_->ParseData(imu_data_sync_buff_);
        imu_sub_ptr_->ParseData(imu_data_buff_);

        if(image_data_sync_buff_.size() == 0 || image_data_sync_buff_.size() == 0)
            return false;

        return true;
    }


    bool VinsLocationFlow::HasData(){
        if(image_data_sync_buff_.size() == 0)
            return false;

        if(imu_data_sync_buff_.size() == 0)
            return false;

        if(imu_data_buff_.size() == 0)
            return false;


        return true;
    }

    bool VinsLocationFlow::ProcessBackend(){
            vins_measurement measurements;
            static double current_time = -1;
            measurements = VinsMeasurementGet();

            if(measurements.size() == 0)
                return false;

            
            
             estimator_buf.lock();

            for(auto &measurement:measurements){
                auto img_msg = measurement.second; // std::shared_ptr<IMG_MSG>
                double ax = 0, ay = 0, az = 0, wx = 0, wy = 0, wz = 0;
                for(auto &imu_msg:measurement.first){

                    double t = imu_msg.time;
                    double img_t = img_msg->img_time;
                    
                    // 插值
                    if(t <= img_t){
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        // printf("dt : %f\n" , dt);
                        assert(dt >= 0);
                        current_time = t;
                        ax = imu_msg.linear_acceleration.x;
                        ay = imu_msg.linear_acceleration.y;
                        az = imu_msg.linear_acceleration.z;
                        wx = imu_msg.angular_velocity.x;
                        wy = imu_msg.angular_velocity.y;
                        wz = imu_msg.angular_velocity.z;
                        //预积分因子
                        estimator_ptr_->processIMU(dt, Eigen::Vector3d(ax,ay,az), Eigen::Vector3d(wx,wy,wz));
                        
                        // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, ax, ay, az, wx, wy, wz);
                    }
                    else{
                        //插值
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        assert(dt_1 >= 0);
                        assert(dt_2 >= 0);
                        assert(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        ax = w1 * ax + w2 * imu_msg.linear_acceleration.x;
                        ay = w1 * ay + w2 * imu_msg.linear_acceleration.y;
                        az = w1 * ax + w2 * imu_msg.linear_acceleration.z;
                        wx = w1 * wx + w2 * imu_msg.angular_velocity.x;
                        wy = w1 * wy + w2 * imu_msg.angular_velocity.y;
                        wz = w1 * wz + w2 * imu_msg.angular_velocity.z;
                        estimator_ptr_->processIMU(dt_1, Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(wx, wy, wz));
                        // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, ax, ay, az, wx, wy, wz);
                    }

                }
                             
                 std::map<int , Eigen::Matrix<double, 7, 1>>  feature_;
                for(unsigned int i = 0; i < img_msg->points.size(); ++i) {
                    //feature 特定的id号
                    int feature_id =img_msg->id_of_points[i];
                    // std::cout << "feature_id : " << feature_id <<std::endl;
                    //经过了去畸变的归一化坐标
                    double x = img_msg->points[i].x();
                    double y = img_msg->points[i].y();
                    double z = img_msg->points[i].z();
                    double p_u = img_msg->u_of_points[i];
                    double p_v = img_msg->v_of_points[i];
                    double velocity_x = img_msg->velocity_x_of_points[i];
                    double velocity_y = img_msg->velocity_y_of_points[i];

                     assert(z == 1);

                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x,y,z,p_u,p_v,velocity_x,velocity_y;
                    feature_[feature_id] = xyz_uv_velocity;
                    
            
                }           

                // std::cout << "feature_ size : " << feature_.size() << std::endl;
                estimator_ptr_->processImage(feature_,img_msg->img_time);

            }



            estimator_buf.unlock();
            return true;
            
    }

    vins_measurement VinsLocationFlow::VinsMeasurementGet(){
        
        vins_measurement measurements;

        // printf("145\n");
        // printf("img_features_buff_ size  : %d\n", img_features_buff_.size());

        const int num_image = img_features_buff_.size();
        
        if(num_image == 0)
            return measurements;


        

        while (true)
        {
            //循环停止是将这两个数组榨干
            if(imu_data_buff_.empty() || img_features_buff_.empty()){
                return measurements;
            }
            std::vector<IMUData> IMU_MSGs;

             double imu_data_front_time  =  imu_data_buff_.front().time ;
             double img_data_front_time  =  img_features_buff_.front()->imu_time;

            
            while(true){
                //date_pre已经做过时间对齐，故直接将小于这个对齐imu之前的收集起来
                const double diff_image_time = img_data_front_time - imu_data_front_time;
                if(diff_image_time >= 0 && diff_image_time < 0.5){
                    IMU_MSGs.emplace_back(imu_data_buff_.front());
                    imu_data_buff_.pop_front();
                }
                else if(diff_image_time >= 0.5){
                    imu_data_buff_.pop_front();
                }
                else{
                    break;
                }
                
                imu_data_front_time  =  imu_data_buff_.front().time;
                
            }

            // printf("two img between %d imu msgs\n",IMU_MSGs.size());


            measurements.emplace_back(IMU_MSGs, img_features_buff_.front());

            img_features_buff_.pop_front();

            //  print_measurement(measurements);

             return measurements;
            
        }
        

        
        // printf("num_image : %d\n",num_image);
        // if(num_image > 0){
        //     double aaa = current_image_data.time -  img_features_buff_.back()->time;
        //     printf("aaa : %f\n", aaa);
        //     printf("start get measurement\n");
        // }
        
        // const double imu_sync_time = current_imu_data.time;
        
        
        
        // //提取小于与image对齐的了个imu_sync的imu数据
        // while(true){
            
        //     // if(imu_data_buff_.empty() || )
        //     const double imu_time = imu_data_buff_.front().time;
            // printf("image_data_sync_buff_ size : %d\n", image_data_sync_buff_.size());
        //     if(imu_time <= imu_sync_time){
        //         IMU_MSG.push_back(imu_data_buff_.front());

        //         imu_data_buff_.pop_front();
        //     }
        //     else{
        //         break;
        //     }
        // }
        

        // //第一次相机有曝光时间 所以imu积累较多，故舍弃第一次的mesurement
        // static bool first_time = false;

        // if(!first_time){
        //     first_time = true;
        //     return false;
        // }

        //measurements.push_back(std::pair<std::vector<IMUData>,ImageData>(IMU_MSG, current_image_data));        

        
    }

    bool VinsLocationFlow::VinsInitial(){
            // for(auto &measurement:measurements){
            //     ImageData img_msg = measurement.second;
                
            //     double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            //     std::vector<IMUData> imu_msgs = measurement.first;

            //     //用于得到预积分                
            //     for(int i = 0; i< imu_msgs.size(); ++i){
            //         const double current_time = imu_msgs[i].time;
            //         static double last_time = current_time;
            //         const double img_t = img_msg.time;


            //         double dt = current_time - last_time;
            //         //assert(dt >= 0);
            //         dx = imu_msgs[i].linear_acceleration.x;
            //         dy = imu_msgs[i].linear_acceleration.y;
            //         dz = imu_msgs[i].linear_acceleration.z;
                    
            //         rx = imu_msgs[i].angular_velocity.x;
            //         ry = imu_msgs[i].angular_velocity.y;
            //         rz = imu_msgs[i].angular_velocity.z;

            //         estimator_ptr_->processIMU(dt, Eigen::Vector3d(dx,dy,dz),Eigen::Vector3d(rx,ry,rz));
                    
                    
            //         last_time = current_time;
            //     }

            //     //std::map<int, std::vector<std::pair<int, Eigen::MatrixM<double,7 ,1>>>
            // }
    }

    void VinsLocationFlow::print_measurement(const vins_measurement&  measurements){
        for(auto &measurement:measurements){
            
            double img_tt = measurement.second->img_time;
            double fisrt_imu_time = measurement.first.front().time;
            for(auto &imu:measurement.first){
                
                // double dt = img_tt - imu.time;
                // printf("imgtt imu diff time : %f\n", dt);
                double dt_1 =  imu.time - fisrt_imu_time;
              
                   
                    printf(" imu diff time : %f\n", dt_1);

                    
                
                

            }
            printf("---------------------------------------------------------------\n");    
        }
        
        
    }
}