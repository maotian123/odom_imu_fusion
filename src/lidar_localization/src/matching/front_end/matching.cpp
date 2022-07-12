/*
 * @Description: LIO localization frontend, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include "lidar_localization/matching/front_end/matching.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"

#include <random>
namespace lidar_localization {

Matching::Matching()
    : global_map_ptr_(new CloudData::CLOUD()),
      local_map_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD()) ,
      angular_vel_bias_{3.5640318696367613e-05,3.5640318696367613e-05,3.5640318696367613e-05},
      G_{0, 0,9.79157},
     linear_acc_bias_( 6.4356659353532566e-05, 6.4356659353532566e-05,6.4356659353532566e-05)
{
    InitWithConfig();

    ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitWithConfig() {
    //
    // load lio localization frontend config file:
    // 
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // prompt:
    LOG(INFO) << std::endl << "----------------- Init LIO Localization, Frontend -------------------" << std::endl;

    // a. init point cloud map & measurement processors:
    InitPointCloudProcessors(config_node);
    // b. load global map:
    InitGlobalMap(config_node);
    // c. init lidar frontend for relative pose estimation:
    InitRegistration(config_node, registration_ptr_);
    // d. init map matcher:
    InitScanContextManager(config_node);

    return true;
}

bool Matching::InitFilter(
    const YAML::Node &config_node, std::string filter_user, 
    std::shared_ptr<CloudFilterInterface>& filter_ptr
) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

    // prompt:
    LOG(INFO) << "\t\tFilter Method for " << filter_user << ": " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitLocalMapSegmenter(const YAML::Node& config_node) {
    local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

bool Matching::InitPointCloudProcessors(const YAML::Node& config_node) {
    // prompt:
    LOG(INFO) << "\tInit Point Cloud Processors:" << std::endl;

    // a. global map filter:
    InitFilter(config_node, "global_map", global_map_filter_ptr_);

    // b.1. local map segmenter:
    InitLocalMapSegmenter(config_node);
    // b.2. local map filter:
    InitFilter(config_node, "local_map", local_map_filter_ptr_);

    // c. scan filter -- 
    InitFilter(config_node, "frame", frame_filter_ptr_);

    return true;
}

bool Matching::InitGlobalMap(const YAML::Node& config_node) {
    std::string map_path = config_node["map_path"].as<std::string>();

    // load map:
    pcl::io::loadPCDFile(map_path, *global_map_ptr_);
    // apply local map filter to global map for later scan-map matching:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);

    // prompt:
    LOG(INFO) << "\tLoad Global Map, size:" << global_map_ptr_->points.size();

    has_new_global_map_ = true;

    return true;
}

bool Matching::InitRegistration(
    const YAML::Node& config_node, 
    std::shared_ptr<RegistrationInterface>& registration_ptr
) {
    std::string registration_method = config_node["registration_method"].as<std::string>();

    // prompt:
    LOG(INFO) << "\tLidar Frontend Estimation Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Estimation method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitScanContextManager(const YAML::Node& config_node) {
    // get loop closure config:
    std::string map_matching_method = config_node["map_matching_method"].as<std::string>();

    // prompt:
    LOG(INFO) << "\tMap Matching Method: " << map_matching_method << std::endl;

    // create instance:
    scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node[map_matching_method]);

    // load pre-built index:
     scan_context_index_path = config_node["scan_context_path"].as<std::string>();
    record_scancontext = config_node["record_scancontext"].as<bool>();

    if(!record_scancontext){
        scan_context_manager_ptr_->Load(scan_context_index_path);
    }
    

    

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};

    // use ROI filtering for local map segmentation:
    local_map_segmenter_ptr_->SetOrigin(origin);
    local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
    LOG(INFO) << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    pose_ = init_pose.cast<double>();
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

// bool Matching::SetInitPoseByhand(const Eigen::Matrix4f& init_pose){
//     SetInitPose(init_pose);
//     has_inited_ = true;
    
//     return true;
// }
bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {
    static int gnss_cnt = 0;

    current_gnss_pose_ = gnss_pose;

    if (gnss_cnt == 0) {
        SetInitPose(gnss_pose);
    } else if (gnss_cnt > 3) {
        has_inited_ = true;
    }
    gnss_cnt++;

    return true;
}

/**
 * @brief  get init pose using scan context matching
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */
bool Matching::SetScanContextPose(const CloudData& init_scan) {
    // get init pose proposal using scan context match:
    Eigen::Matrix4f init_pose =  Eigen::Matrix4f::Identity();
    if (
        !scan_context_manager_ptr_->DetectLoopClosure(init_scan, init_pose)
    ) {
        return false;
    }

    // set init pose:
    SetInitPose(init_pose);
    has_inited_ = true;
    
    return true;
}

bool Matching::Update(
    const CloudData& cloud_data, 
    const LidarMeasurement& measurement,
    Eigen::Matrix4f& laser_pose, Eigen::Matrix4f &map_matching_pose
) {
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = Eigen::Matrix4f::Identity();
    static double last_laser_time = cloud_data.time;
    // remove invalid measurements:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    // downsample current scan:
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }
    

    //matching miss test
    if(miss_matching_signal){
        double trans_w_sigma= 0.5;                 // 噪声Sigma值
        double angle_w_sigma= 1.;                 // 噪声Sigma值

        std::default_random_engine angle_generator;
        std::normal_distribution<double> angle_noise(0, angle_w_sigma);

        std::default_random_engine trans_generator;
        std::normal_distribution<double> trans_noise(0, trans_w_sigma);
        
        double random_angle = angle_noise(angle_generator);
        double random_trans = trans_noise(trans_generator);
       predict_pose(0,3) =  predict_pose(0,3) + random_trans;
       predict_pose(1,3) =  predict_pose(1,3) + random_trans;
       predict_pose(2,3) =  predict_pose(2,3) + random_trans;

        Eigen::Vector3f ea0(random_angle,random_angle,random_angle); //
        Eigen::Matrix3f R;
        R = ::Eigen::AngleAxisf(ea0[0], ::Eigen::Vector3f::UnitZ())
                * ::Eigen::AngleAxisf(ea0[1], ::Eigen::Vector3f::UnitY())
                * ::Eigen::AngleAxisf(ea0[2], ::Eigen::Vector3f::UnitX());       
        
        predict_pose.block<3,3>(0,0) = R * predict_pose.block<3,3>(0,0);
        miss_matching_signal  = false;
    }
    // matching:
    bool use_imu_pre = false;
    // std::cout << " measurement[0].first.size() : " << measurement[0].first.size() << std::endl;
    if(measurement.size() > 0){
        
        UpdatePreposition(measurement,laser_pose);
        use_imu_pre = true;
    }
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());

    if(!use_imu_pre){
        registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, laser_pose);
    }
    else{
        registration_ptr_->ScanMatch(filtered_cloud_ptr, pose_.cast<float>(), result_cloud_ptr, laser_pose);
    }
    
    
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, laser_pose);
   
     double delte_time = cloud_data.time - last_laser_time;
    // std::cout << " delte_time : " << delte_time << std::endl;
    RefreshState(last_pose.cast<double>(),laser_pose.cast<double>(),  delte_time);

    //update delte time
    last_laser_time = cloud_data.time;
    // update predicted pose:
    step_pose = last_pose.inverse() * laser_pose;
    last_pose = laser_pose;
    //imu 预测之前的位姿

    predict_pose = laser_pose * step_pose;
    current_frame_.time = cloud_data.time;
    current_frame_.pose = laser_pose;

    if(record_scancontext){

        
        double a =fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3));

        // printf("a:, %f\n",a);
        if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2) {
            printf("record the scancontextg\n");
            
            
            scan_context_manager_ptr_->Update(
                cloud_data, current_frame_
            );
            
            last_key_frame_pose = current_frame_.pose;
        }
    }
    else{
        // init the map matching pose as laser pose:
        
        map_matching_pose = laser_pose;
        scan_context_manager_ptr_->DetectLoopClosure(cloud_data, map_matching_pose);
        
    }
    
    // shall the local map be updated:
    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (
            fabs(laser_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(laser_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(laser_pose(0,3), laser_pose(1,3), laser_pose(2,3));
        break;
    }

    
    return true;
}

void Matching::RefreshState(const Eigen::Matrix4d &last_pose_,const Eigen::Matrix4d& current_pose_,const double& delte_time){

    // std::cout << " ok " << std::endl;

    if(!delte_time == 0 ){
        vel_[0] = (current_pose_(0,3) - last_pose_(0,3)) / (delte_time);
        vel_[1] = (current_pose_(1,3) - last_pose_(1,3)) / (delte_time);
        vel_[2] = (current_pose_(2,3) - last_pose_(2,3)) / (delte_time);
        // std::cout << " come in  " << vel_ << std::endl;
    }
    
    // std::cout << " ok  376 " << std::endl;
    pose_ = current_pose_;

    
}
bool Matching::UpdatePreposition(const LidarMeasurement& measurement, Eigen::Matrix4f &predict_pose){
        
        Eigen::Vector3d angular_delte;
        Eigen::Vector3d velocity_delte;
        Eigen::Matrix3d R_curr;
        Eigen::Matrix3d R_prev;
        const size_t current_idx = 1, prev_idx = 0;
        imu_data_buff_ = measurement[0].first;
        while(imu_data_buff_.size() != 1){
            double delta_time = imu_data_buff_.at(current_idx).time - imu_data_buff_.at(prev_idx).time;
            GetAngularDelta(current_idx, prev_idx, angular_delte);
            UpdateOrientation(angular_delte, R_curr, R_prev);
            GetVelocityDelta(current_idx, prev_idx,R_curr, R_prev,  delta_time,
                                                velocity_delte);
            UpdatePosition(delta_time, velocity_delte);

            imu_data_buff_.pop_front();
        }

}

bool Matching::GetAngularDelta( const size_t index_curr, const size_t index_prev, Eigen::Vector3d &angular_delta)
{
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }
    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;
    Eigen::Vector3d temp_imu_data_curr{imu_data_curr.angular_velocity.x,imu_data_curr.angular_velocity.y,imu_data_curr.angular_velocity.z};
    Eigen::Vector3d temp_imu_data_prev{imu_data_prev.angular_velocity.x,imu_data_prev.angular_velocity.y,imu_data_prev.angular_velocity.z};

    Eigen::Vector3d angular_vel_curr = GetUnbiasedAngularVel(temp_imu_data_curr);
    Eigen::Vector3d angular_vel_prev = GetUnbiasedAngularVel(temp_imu_data_prev);
    
    angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);

}

inline Eigen::Vector3d Matching::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel) {
    return angular_vel - angular_vel_bias_;
}
void Matching::UpdateOrientation(
    const Eigen::Vector3d &angular_delta,
    Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
) {

    // magnitude:
    double angular_delta_mag = angular_delta.norm();
    // direction:
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    // build delta q:
    double angular_delta_cos = cos(angular_delta_mag/2.0);
    double angular_delta_sin = sin(angular_delta_mag/2.0);
    Eigen::Quaterniond dq(
        angular_delta_cos, 
        angular_delta_sin*angular_delta_dir.x(), 
        angular_delta_sin*angular_delta_dir.y(), 
        angular_delta_sin*angular_delta_dir.z()
    );
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    
    // update:
    q = q*dq;
    
    // write back:
    R_prev = pose_.block<3, 3>(0, 0);
    pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    R_curr = pose_.block<3, 3>(0, 0);
}

inline Eigen::Vector3d Matching::GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &R
) {
    
    return R*(linear_acc - linear_acc_bias_) - G_;
}

bool Matching::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
    double &delta_t, Eigen::Vector3d &velocity_delta
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    delta_t = imu_data_curr.time - imu_data_prev.time;
    Eigen::Vector3d temp_imu_data_curr{imu_data_curr.linear_acceleration.x,imu_data_curr.linear_acceleration.y,imu_data_curr.linear_acceleration.z};
    Eigen::Vector3d temp_imu_data_prev{imu_data_prev.linear_acceleration.x,imu_data_prev.linear_acceleration.y,imu_data_prev.linear_acceleration.z};
    
    Eigen::Vector3d linear_acc_curr = GetUnbiasedLinearAcc(temp_imu_data_curr,R_curr);
    Eigen::Vector3d linear_acc_prev = GetUnbiasedLinearAcc(temp_imu_data_prev, R_prev);
    
    velocity_delta = 0.5*delta_t*(linear_acc_curr + linear_acc_prev);

    return true;
}

void Matching::UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    pose_.block<3, 1>(0, 3) += delta_t*vel_ + 0.5*delta_t*velocity_delta;
    
    vel_ += velocity_delta;
}

bool Matching::HasInited(void) {
    return has_inited_;
}

bool Matching::HasNewGlobalMap(void) {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap(void) {
    return has_new_local_map_;
}

bool Matching::Save(){
    scan_context_manager_ptr_->Save(scan_context_index_path);
}

bool Matching::MissMatch(){
    miss_matching_signal = true;
}

Eigen::Matrix4f Matching::GetInitPose(void) {
    return init_pose_;
}

CloudData::CLOUD_PTR& Matching::GetGlobalMap(void) {
    has_new_global_map_ = false;
    return global_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap(void) {
    return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan(void) {
    return current_scan_ptr_;
}

} // namespace lidar_localization