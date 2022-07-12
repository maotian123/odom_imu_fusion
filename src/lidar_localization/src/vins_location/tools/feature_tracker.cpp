#include "lidar_localization/vins_location/tools/feature_tracker.hpp"


namespace lidar_localization
{
    void FeatureTraker::readImageData(const ImageData &image_msg){
        
        // for(auto &pt_v : pts_velocity){
        //     printf("pt_x : %f, pt_y : %f \n", pt_v.x,pt_v.y);
        // }
    
        //均值化
        cv::Mat current_grey_img;
        cur_time  = image_msg.time;

        cvtColor(image_msg.image_mat,current_grey_img,CV_BGR2GRAY);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(current_grey_img, current_grey_img);

  
        cur_pts.clear();
        if(prev_pts.size() > 0){
            std::vector<uchar> status;
            std::vector<float> err;

            cv::calcOpticalFlowPyrLK(prev_img, current_grey_img, prev_pts,cur_pts , status, err, cv::Size(21, 21), 3);

            // for(int i = 0; i < cur_pts.size(); ++i){
            //    if(status[i] == 0){
            //          std::cout << "status  0  in " << i << " : " << int(status[i]) << std::endl;    
            //    }
               
            // }
            reduceVector(prev_pts, status);
            //这里的cur_un_pts 是上一次纠正的，因为当前纠正的函数在下面
            reduceVector(cur_un_pts,status);
            reduceVector(cur_pts, status);
            reduceVector(track_cnt, status);
            reduceVector(feature_ids, status);

        }
       
        
        for (auto &n : track_cnt)
            n++;
        //通过本质矩阵进一步筛选trackerData
        rejectWithF();
        //TODO 特征点
        /**
         * 提取新的角点，MAX_CNT - forw_pts.size()为提取的最大个数
         * 新提取的角点坐标保存在n_pts中
         * MIN_DIST=30，该参数保证2个相邻角点之间的最小距离
         * 
         * 第一个参数是输入图像（8位或32位单通道图）。
         * 第二个参数是检测到的所有角点，类型为vector或数组，由实际给定的参数类型而定。如果是vector，那么它应该是一个包含cv::Point2f的vector对象；如果类型是cv::Mat,那么它的每一行对应一个角点，点的x、y位置分别是两列。
         * 第三个参数用于限定检测到的点数的最大值。
         * 第四个参数表示检测到的角点的质量水平（通常是0.10到0.01之间的数值，不能大于1.0）。
         * 第五个参数用于区分相邻两个角点的最小距离（小于这个距离得点将进行合并）。
         * 第六个参数是mask，如果指定，它的维度必须和输入图像一致，且在mask值为0处不进行角点检测。
         * 第七个参数是blockSize，表示在计算角点时参与运算的区域大小，常用值为3，但是如果图像的分辨率较高则可以考虑使用较大一点的值。
         * 第八个参数用于指定角点检测的方法，如果是true则使用Harris角点检测，false则使用Shi Tomasi算法。
         * 第九个参数是在使用Harris算法时使用，最好使用默认值0.04。
         * */
        setMask();
        //在这张图片中找到 MAX_CNT - forw_pts.size() 个 特征点 存放于 n_pts中
        const int n_max_cnt =  MAX_CNT - static_cast<int>(cur_pts.size());
        
        if(n_max_cnt > 0){
            cv::goodFeaturesToTrack(current_grey_img, n_pts, n_max_cnt, 0.01, MIN_DIST,mask);
        }
         else{
            n_pts.clear(); //这句话很重要
        }
        //add points
        for(auto &p:n_pts){
            // std::cout << " come in " << std::endl;
            cur_pts.push_back(p);
            feature_ids.push_back(-1);
            track_cnt.push_back(1);
        }

        prev_pts = cur_pts;
        prev_img = current_grey_img;
        prev_un_pts = cur_un_pts;
        undistortedPoints();
        prev_time = cur_time;
        // cv::Mat UndistortImage;
        // cv::undistort(image_msg.image_mat, UndistortImage, K, D, K);        
        // cv::imshow("UndistortImage", UndistortImage);
        // cv::waitKey(1);
        
        // cv::Mat show_img;
        // cv::cvtColor(current_grey_img, show_img, CV_GRAY2RGB);
        //  for (unsigned int j = 0; j < prev_pts.size(); j++)
        // {
        //     double len = std::min(1.0, 1.0*track_cnt[j]/10 );
        //     cv::circle(show_img, prev_pts[j] , 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        // }
        
        // cv::imshow("IMAGE", show_img);
        // cv::waitKey(1);

        return ;
    }

    void FeatureTraker::rejectWithF(){
        if(cur_pts.size() >=8){
            std::vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_cur_pts(cur_pts.size());

            for(unsigned int i = 0; i < prev_pts.size(); ++i){
                Eigen::Vector3d temp_p;
                liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y),temp_p);
                temp_p.x() = 702 * temp_p.x() + 480;
                temp_p.y() = 702 * temp_p.x() + 270;
                un_prev_pts[i] = cv::Point2f(temp_p.x(), temp_p.y());


                liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),temp_p);
                temp_p.x() = 702 * temp_p.x() + 480;
                temp_p.y() = 702 * temp_p.x() + 270;
                un_cur_pts[i] = cv::Point2f(temp_p.x(), temp_p.y());
            }

            std::vector<uchar> status;
            cv::findFundamentalMat(un_prev_pts, un_cur_pts, cv::FM_RANSAC, 1.0, 0.99, status);
            reduceVector(prev_pts, status);
            //这里的cur_un_pts 是上一帧图像的特征点 纠正的，因为Undistored函数在下面
            reduceVector(cur_un_pts,status);
            reduceVector(cur_pts, status);
            reduceVector(track_cnt, status);
            reduceVector(feature_ids, status);
            
        }
    }

    void FeatureTraker::reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status){
         int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }


    void FeatureTraker::reduceVector(std::vector<int> &v, std::vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }



    void FeatureTraker::setMask(){
        mask = cv::Mat(540, 960, CV_8UC1, cv::Scalar(255));

        // prefer to keep features that are tracked for long time
        std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

        for (unsigned int i = 0; i < cur_pts.size(); i++)
            cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(cur_pts[i], feature_ids[i])));


        sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

        // std::cout << " before cur_pts size " << cur_pts.size() << std::endl;
        // std::cout << " cnt_pts_id size " << cnt_pts_id.size() << std::endl;
         cur_pts.clear();
         feature_ids.clear();
         track_cnt.clear();
        
         for (auto &it : cnt_pts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                cur_pts.push_back(it.second.first);
                feature_ids.push_back(it.second.second);
                track_cnt.push_back(it.first);

                cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
                // cv::imshow("mask",mask);
                // cv::waitKey(1);
            }

        
        }
        // std::cout << " after cur_pts size " << cur_pts.size() << std::endl;
    }


    bool FeatureTraker::updateID(unsigned int i){
        // std::cout << "feature_ids  size :  " << feature_ids.size() << std::endl;
        
        if(i < feature_ids.size()){
            // std::cout << " wtf " << std::endl;
            if(feature_ids[i] == -1){
                feature_ids[i] = feature_num_id++;
                
                return true;
            }
        }else 
            return false;

        return true;
    }




    bool FeatureTraker::liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d & P){

        // static const cv::Mat K =  (cv::Mat_<double>( 3,3 ) << 701.8400268554688, 0.0, 486.79998779296875, 0.0, 702.0449829101562, 265.5050048828125, 0.0, 0.0, 1.0);
        // static const cv::Mat D =  (cv::Mat_<double>( 5,1 ) << -0.17445899546146393, 0.027615800499916077, 5.916510337211633e-11, 0.00015463300223927945, -2.4430599296465516e-05);
        static const double fx = 701.8400268554688;
        static const double fy = 486.79998779296875;
        static const double cx = 702.0449829101562;
        static const double cy = 265.5050048828125;
        
    
        // Lift points to normalised plane
        //X = (u- cx)/fx Y = (v - cy)/fy  points -> (col , row)
        const double u = p[0];
        const double v = p[1];
        const double X_distorted  = (p[0] - fx) / cx;
        const double Y_distorted  = (p[1] - fy) / cy;
        
        double X_undistorted, Y_undistorted;
        
        Eigen::Vector2d d_u;
        distortion(Eigen::Vector2d(X_distorted, Y_distorted),d_u);

        X_undistorted = X_distorted - d_u[0];
        Y_undistorted = Y_distorted - d_u[1];

        for(int i = 0; i < 8; ++i){  //循环去畸变
            distortion(Eigen::Vector2d(X_distorted, Y_distorted),d_u);

            X_undistorted = X_distorted - d_u[0];
            Y_undistorted = Y_distorted - d_u[1];
        }

        P << X_undistorted, Y_undistorted, 1;

        //  printf("X : %f, Y : %f\n",X ,Y );

    }

    bool FeatureTraker::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d& d_u){
        static const double k1 = -0.17445899546146393;
        static const double k2 = 0.027615800499916077;
        static const double k3 = 5.916510337211633e-11;
        static const double k4 = 0.00015463300223927945;


        double X_2, Y_2, XY, rho2, rad_dist;


        X_2 = p_u[0] * p_u[0];
        Y_2 = p_u[1] * p_u[1];
        XY = p_u[0] * p_u[1];
        rho2 = X_2 + Y_2;
        rad_dist = k1 * rho2 + k2*rho2 * rho2;
        d_u << p_u(0) * rad_dist + 2 * k3*XY + k4*(rho2 + 2 * X_2),
                        p_u(1) * rad_dist +  2 *k4*XY + k3*(rho2 + 2 * Y_2);
    }

    bool FeatureTraker::undistortedPoints(){
        cur_un_pts.clear();
        cur_un_pts_map.clear();

        /*测试去畸变后的图像
        D <<  -0.17445899546146393, 0.027615800499916077, 5.916510337211633e-11, 0.00015463300223927945, -2.4430599296465516e-05
        K <<  701.8400268554688, 0.0, 486.79998779296875, 0.0, 702.0449829101562, 265.5050048828125, 0.0, 0.0, 1.0
        ROW： 540
        COL ：960
        */

        //prev_pts 为经过了光流追踪以及特征提取后的所有特征
        for(unsigned int i = 0; i < prev_pts.size(); ++i){
            Eigen::Vector2d a(prev_pts[i].x, prev_pts[i].y);
            Eigen::Vector3d b;
            liftProjective(a,b);
            cur_un_pts.push_back(cv::Point2f(b.x(), b.y()));

            cur_un_pts_map.insert(std::make_pair(feature_ids[i], cv::Point2f(b.x() , b.y())));            
        }

        //计算points的速度
        if(!prev_un_pts_map.empty()){
            double dt = cur_time - prev_time;
            pts_velocity.clear();
            for (unsigned int i = 0; i < cur_un_pts.size(); i++)
            {
                if (feature_ids[i] != -1)
                {
                    std::map<int, cv::Point2f>::iterator it;
                    it = prev_un_pts_map.find(feature_ids[i]);
                    if (it != prev_un_pts_map.end())
                    {
                        double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                        double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                        // printf("aaaa\n");
                        pts_velocity.push_back(cv::Point2f(v_x, v_y));
                    }
                    else
                        pts_velocity.push_back(cv::Point2f(0, 0));
                }
                else
                {
                    pts_velocity.push_back(cv::Point2f(0, 0));
                }
            }
        }
        else{
            for (unsigned int i = 0; i < prev_pts.size(); i++)
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }

        //完成递归
        prev_un_pts_map = cur_un_pts_map;
    }

} // namespace lid
