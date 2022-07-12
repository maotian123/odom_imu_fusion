#include "lidar_localization/vins_location/tools/estimator.hpp"


namespace lidar_localization{
    Estimator::Estimator(){
         for (size_t i = 0; i < WINDOW_SIZE + 1; i++)
        {
            pre_integrations[i] = nullptr;
        }
        for(auto &it: all_image_frame)
        {
            it.second.pre_intergration = nullptr;
        }
        temp_intergration = nullptr;
        
        clearState();
        InitWithYamlConfig();

    }
    bool Estimator::clearState(){
            for (int i = 0; i < WINDOW_SIZE + 1; i++)
            {
                Rs[i].setIdentity();
                Ps[i].setZero();
                Vs[i].setZero();
                Bas[i].setZero();
                Bgs[i].setZero();
                dt_buf[i].clear();
                linear_acceleration_buf[i].clear();
                angular_velocity_buf[i].clear();

                if (pre_integrations[i] != nullptr)
                    delete pre_integrations[i];

                pre_integrations[i] = nullptr;
            }

            tic = Eigen::Vector3d::Zero();
            ric = Eigen::Matrix3d::Identity();
            
            for (auto &it : all_image_frame)
            {
                if (it.second.pre_intergration != nullptr)
                {
                    delete it.second.pre_intergration;
                    it.second.pre_intergration = nullptr;
                }
            }

            solver_flag = INITIAL;
            current_frame_size = 0;
            all_image_frame.clear();
            if (temp_intergration != nullptr)
                delete temp_intergration;
    
            temp_intergration = nullptr;
            feature_list.clear();
        }

    bool Estimator::InitWithYamlConfig(){

        std::string config_file_path = WORK_SPACE_PATH + "/config/vins_location/vins.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        std::cout << "-----------------Init Vins Localization, frontend-------------------" << std::endl;

        ACC_N = config_node["acc_n"].as<double>();
        ACC_W = config_node["acc_w"].as<double>();
        GYR_N = config_node["gyr_n"].as<double>();
        GYR_W = config_node["gyr_w"].as<double>();

    }


    bool Estimator::processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity){
        
        if (!first_imu)
        {
            first_imu = true;
            acc_0 = linear_acceleration;
            gyr_0 = angular_velocity;
        }

        //初始化
        if(!pre_integrations[current_frame_size]){

            pre_integrations[current_frame_size] = new PreIntergration{acc_0, gyr_0, Bas[current_frame_size], Bgs[current_frame_size],
                                                                                                                                         ACC_N, ACC_W, GYR_N, GYR_W};
            
        }

        if(current_frame_size != 0){
            
            pre_integrations[current_frame_size]->push_back(dt, linear_acceleration, angular_velocity);
            
            // printf("pre_integrations dt 94 : %f\n",pre_integrations[current_frame_size]->dt);
            // printf("pre_integrations sum_dt 94 : %f\n",pre_integrations[current_frame_size]->sum_dt);
            // printf("pre_integrations delta v 94 : %f\n",pre_integrations[current_frame_size]->delta_v(0));
            temp_intergration->push_back(dt, linear_acceleration, angular_velocity);
            
            dt_buf[current_frame_size].push_back(dt);

            linear_acceleration_buf[current_frame_size].push_back(linear_acceleration);

            angular_velocity_buf[current_frame_size].push_back(angular_velocity);
            
            int j = current_frame_size;
            //中值积分
            Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) ;
            
            Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j] ;
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]);
            Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
            
        }
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    bool Estimator::processImage(const std::map<int , Eigen::Matrix<double, 7, 1>>  &feature_, double time){
        static double initial_timestamp = time;

        bool diff_view_angle_ok =  addFeatureCheckParallax(feature_);
        //参考https://www.cnblogs.com/buxiaoyi/p/8660854.html 4.4.2
        // 视角比较小则边缘化第一帧 比较大则边缘化倒数第二帧
        if(diff_view_angle_ok)
            marginalization_flag = 0;
        else
            marginalization_flag = 1;

        // printf("---------------------------------------------------------------------------------------------  : %d\n",current_frame_size);
        Images_time[current_frame_size] = time;
        ImageFrame imageframe(feature_, time);

        imageframe.pre_intergration = temp_intergration;
        // printf("time : %f\n", time);
        all_image_frame.insert(std::make_pair(time, imageframe));
        temp_intergration = new PreIntergration{acc_0, gyr_0, Bas[current_frame_size], Bgs[current_frame_size],ACC_N,ACC_W,GYR_N,GYR_W};
        //线性初始化
        // printf("136 come in \n");
        if(solver_flag == INITIAL){
            if(current_frame_size == WINDOW_SIZE){
                bool result = false;
                
                if(time - initial_timestamp> 0.033){
                    result = initialStructure();
                    //clearState();
                    initial_timestamp = time;
                }
                //如果result 为false说明初始化失败 进行边缘化
                if(result){
                    printf("initial fauiler\n");
                }
                else{
                        slideWindow();
                }
            }
            else{
                // printf("frame count  : %d\n",current_frame_size);
                current_frame_size++;
            }

        }



        // addFeatureCheckParallax();
    }

    bool Estimator::addFeatureCheckParallax(const std::map<int , Eigen::Matrix<double, 7, 1>>  &features_info){
         // 用于记录所有特征点的视差总和
        double parallax_sum = 0;
        // 记录满足某些条件的特征点个数
        int parallax_num = 0;
        // 被跟踪点的个数
         int last_track_num = 0;
        for(auto &feature_info:features_info){

            int pts_id = feature_info.first;
            //内部集成了每个特征点的开始的帧 以及 在那些帧上检测（都是连续的）、归一化坐标等信息
            PerFrameFeaturesInfo feature_per_frame(pts_id, current_frame_size);
            Pts_Info_frame pts_info_frame;
            pts_info_frame.pts_info = feature_info.second;
  
            
            //在list中差找到了这个特征点信息
            auto it = find_if(feature_list.begin(), feature_list.end(), [pts_id](const PerFrameFeaturesInfo &it){
                return it.feature_id == pts_id;
            });
            
            //没有这个特征点的信息记录
            if(it == feature_list.end()){

                feature_list.push_back(feature_per_frame);
                feature_list.back().per_features_info.push_back(pts_info_frame);
                feature_list.back().per_features_info.back().frame = 1;
                // feature_list.push_back(std::make_pair<int, std::vector<int, Eigen::Matrix<double, 7, 1>>)(current_frame_size, pts_Info);
            }
            else{
                Pts_Info_frame temp_pts_info_frame;
                temp_pts_info_frame = pts_info_frame;
                temp_pts_info_frame.frame = it->per_features_info.size() + it->start_frame;
                it->per_features_info.push_back(temp_pts_info_frame);
                // it->per_features_info.push_back();
                last_track_num++;
            }
           
        }
        // std::cout << " current_frame_size : " << current_frame_size << " last_track_num : " << last_track_num << std::endl;
        if(current_frame_size < 2 || last_track_num < 20){
            return true;
        }

        for(auto& it_per_id:feature_list){
            if(it_per_id.start_frame <= current_frame_size-2 &
                it_per_id.per_features_info.back().frame >=  current_frame_size-1){
                    parallax_sum += compensatedParallax2(it_per_id);
                    parallax_num++;
                    // printf("feature start  frame : %d\n",it_per_id.start_frame);
                    // printf("feature end  frame : %d\n",it_per_id.per_features_info.back().frame);
                }
        }

        
        // std::cout << " parallax_sum : " << parallax_sum << std::endl;
        
        
        if(parallax_num == 0){
            return true;
        }
        else{
            bool result = parallax_sum/parallax_num  >= MIN_PARALLAX;
            double a= parallax_sum/parallax_num;
            // std::cout << " a " << a << std::endl;
            // std::cout << "MIN_PARALLAX  " << MIN_PARALLAX<< std::endl;
            // std::cout << " parallax_sum/parallax_num  >= MIN_PARALLAX " <<result << std::endl;
            return parallax_sum/parallax_num  >= MIN_PARALLAX;
        }
        // printf("--------------------------------------------------------");

    }

    //计算视角差
    double Estimator::compensatedParallax2(const PerFrameFeaturesInfo& it_per_id){
        //倒数第三张跟踪成功与倒数第二张跟踪成功的照片
        const Pts_Info_frame &frame_i = it_per_id.per_features_info[current_frame_size - 2 - it_per_id.start_frame];
        const Pts_Info_frame &frame_j = it_per_id.per_features_info[current_frame_size - 1 - it_per_id.start_frame];

        double ans = 0;

        Eigen::Vector3d p_i = frame_i.pts_info.block<3,1>(0,0);
        Eigen::Vector3d p_j = frame_j.pts_info.block<3,1>(0,0);

        double x_i = p_i(0);
        double x_j = p_j(0);

        double y_i = p_i(1);
        double y_j = p_j(1);

        double d_x = x_i - x_j;
        double d_y = y_i - y_j;

        
        
        ans = std::max(ans, d_x * d_x + d_y * d_y);
        // std::cout << " ans : " << ans << std::endl;
        return ans;
    }


    bool Estimator::initialStructure(){
        int i = 0;
        std::map<double, ImageFrame>::iterator frame_it;
         Eigen::Vector3d sum_g;
          //遍历除了第一帧图像外的所有图像帧
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            
            double dt = frame_it->second.pre_intergration->sum_dt;
            // printf("frame_it->second.pre_intergration->delta_v  : %f\n", frame_it->second.pre_intergration->delta_v (0));
            //计算每一帧图像对应的加速度
            Eigen::Vector3d tmp_g = frame_it->second.pre_intergration->delta_v / dt;
            // printf("tmp_g  : %f\n", tmp_g(0));
            //图像的加速度之累加和
            sum_g += tmp_g;
        //     i++;
        //    printf("  sum_g  : %f\n",sum_g(0));
        }
        
        Eigen::Vector3d aver_g;
        //计算平均加速度。因为上边计算的是除了第一帧图像之外的其他所有图像帧对应的加速度之和，所以这里要减去1
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

        double var = 0;

        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_intergration->sum_dt;
            //计算获得每一帧的加速度
            Eigen::Vector3d tmp_g = frame_it->second.pre_intergration->delta_v / dt;
            //加速度减去平均加速度的差值的平方的累加
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // printf("  var  : %f\n",var);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }

        Eigen::Quaterniond Q[current_frame_size + 1];
        Eigen::Vector3d T[current_frame_size + 1];
        std::map<int, Eigen::Vector3d> sfm_tracked_points;
        std::vector<SFMFeature> sfm_f;
        for(auto &ite_per_id:feature_list){
            int imu_j = ite_per_id.start_frame -1;
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = ite_per_id.feature_id;
              for (auto &it_per_frame : ite_per_id.per_features_info)
                {
                    imu_j++;
                    Vector3d pts_j = it_per_frame.pts_info.block<3,1>(0,0);
                    //遍历每一个能观察到该feature的frame
                    tmp_feature.observation.push_back(std::make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
                }
            sfm_f.push_back(tmp_feature);
        }

        //遍历feature，获得里边每一个特征点对应的图像帧中点的x,y坐标存入tmp_feature中，
        //最后将tmp_feature存入到sfm_f中。后边就可以通过sfm_f获取到feature的id以及有哪些帧可以观测到该feature以及在图像帧中的坐标。
        //位姿求解
        Eigen::Matrix3d relative_R;
        Eigen::Vector3d relative_T;
        int l;
        //通过求取本质矩阵来求解出位姿
        /**
         * 这里的l表示滑动窗口中第l帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，
         * 会作为参考帧到下面的全局sfm使用，得到的Rt为当前帧到第l帧的坐标系变换Rt，存储在relative_R和relative_T当中
         * */
        //三角化求解地图点的深度
        GlobalSFM sfm;
        if (!relativePose(relative_R, relative_T, l))
        {
            cout << "Not enough features or parallax; Move device around" << endl;
            return false;
        }

        if (!sfm.construct(current_frame_size + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
        {
            cout << "global SFM failed!" << endl;
            marginalization_flag = 0;
            return false;
        }
 
        return false;
    }

    bool Estimator::slideWindow(){
        //MARGIN_OLD
        if(marginalization_flag == 0){
            // std::cout << " come in 328 " << std::endl;
        }
        else{
            //边缘化掉滑动窗口中次新的帧
            // std::cout << " come in 332 " << std::endl;
            if(current_frame_size == WINDOW_SIZE){
                 for (unsigned int i = 0; i < dt_buf[current_frame_size].size(); i++)
                {
                    //边缘化次新的帧 将最后一帧的所有imu数据给到倒数第二帧作积分
                    double tmp_dt = dt_buf[current_frame_size][i];
                    Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf[current_frame_size][i];
                    Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf[current_frame_size][i];

                    pre_integrations[current_frame_size - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[current_frame_size - 1].push_back(tmp_dt);
                    linear_acceleration_buf[current_frame_size - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[current_frame_size - 1].push_back(tmp_angular_velocity);
                }
                //用滑动窗口中最新的帧的状态替换掉次新的帧的状态
                Images_time[current_frame_size - 1] = Images_time[current_frame_size];
                Ps[current_frame_size - 1] = Ps[current_frame_size];
                Vs[current_frame_size - 1] = Vs[current_frame_size];
                Rs[current_frame_size - 1] = Rs[current_frame_size];
                Bas[current_frame_size - 1] = Bas[current_frame_size];
                Bgs[current_frame_size - 1] = Bgs[current_frame_size];
                
                //删除最后一个预积分器
                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new PreIntergration{acc_0, gyr_0, Bas[current_frame_size], Bgs[current_frame_size],ACC_N,ACC_W,GYR_N,GYR_W};


                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();

                slideWindowNew();
            }
        }
    }


    bool Estimator::slideWindowNew(){
        for(auto it = feature_list.begin(), it_next = feature_list.begin(); it!=feature_list.end(); it=it_next){
            it_next++;
            
            if(it->start_frame == current_frame_size){
                it->start_frame--;
            }
            else{
                int j = WINDOW_SIZE -1 - it->start_frame;
                int end_frame = it->start_frame + it->per_features_info.size() - 1;
                if(end_frame < current_frame_size - 1)
                    continue;
                it->per_features_info.erase(it->per_features_info.begin() + j);
                if(it->per_features_info.size() == 0)
                    feature_list.erase(it);    
            }
        }
        
        return true;   
    }

    /**
     * 这里的第l帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，
     * 会作为参考帧到下面的全局sfm使用，得到的Rt为当前帧到第l帧的坐标系变换Rt
     * 该函数判断滑动窗口中第一个到窗口最后一帧对应特征点的平均视差大于30，且内点数目大于12的帧，此时可进行初始化，同时返回当前帧到第l帧的坐标系变换R和T
     * */
    bool Estimator::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l){
        for(int i = 0; i < WINDOW_SIZE; ++i){
            
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
             //寻找第i帧到窗口最后一帧的对应特征点，存放在corres中
            corres = getCorresponding(i, WINDOW_SIZE); 

            if(corres.size() > 20){
             //计算平均视差
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                 //第j个对应点在第i帧和最后一帧的(x,y)
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                //计算视差
                double parallax = (pts_0 - pts_1).norm();
                //计算视差的总和
                sum_parallax = sum_parallax + parallax;
            }
            //计算平视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            std::cout << " i  : "  << i << " average_parallax goal : " << average_parallax * 702 << std::endl;
            //判断是否满足初始化条件：视差>30和内点数满足要求(大于12)
            //solveRelativeRT()通过基础矩阵计算当前帧与第l帧之间的R和T,并判断内点数目是否足够
            //同时返回窗口最后一帧（当前帧）到第l帧（参考帧）的relative_R，relative_T
            if (average_parallax * 702 > 30 && solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                //ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
        }
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> Estimator::getCorresponding(int frame_count_l, int frame_count_r)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        for (auto &it : feature_list)
        {
            // std::cout << " it.start_frame : "  << it.start_frame << std::endl;
            int end_frame = it.start_frame + it.per_features_info.size() - 1;
            // std::cout << " end_frame : "  << end_frame << std::endl;
            if (it.start_frame <= frame_count_l && end_frame >= frame_count_r)
            {
                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
                int idx_l = frame_count_l - it.start_frame;
                int idx_r = frame_count_r - it.start_frame;

                a = it.per_features_info[idx_l].pts_info.block<3,1>(0,0);

                b = it.per_features_info[idx_r].pts_info.block<3,1>(0,0);
                
                corres.push_back(std::make_pair(a, b));
            }
        }

         return corres;
    }

        /*
    这里计算出滑动窗口中第一个满足条件的帧和最新帧之间的旋转和平移之后，还要判断内点数是否大于12，大于12才认为计算出的旋转和平移量是有效的。
    */
    bool Estimator::solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation)
    {
        if (corres.size() >= 15)
        {
            vector<cv::Point2f> ll, rr;
            for (int i = 0; i < int(corres.size()); i++)
            {
                ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
                rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
            }
            cv::Mat mask;
            cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
            cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            cv::Mat rot, trans;
            int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
            
            Eigen::Matrix3d R;
            Eigen::Vector3d T;
            for (int i = 0; i < 3; i++)
            {   
                T(i) = trans.at<double>(i, 0);
                for (int j = 0; j < 3; j++)
                    R(i, j) = rot.at<double>(i, j);
            }
            // std::cout << "R" << R << std::endl;
            // std::cout << "T" << T << std::endl;
            Rotation = R.transpose();
            Translation = -R.transpose() * T;
            if(inlier_cnt > 12)
                return true;
            else
                return false;
        }
        return false;
    }

}
