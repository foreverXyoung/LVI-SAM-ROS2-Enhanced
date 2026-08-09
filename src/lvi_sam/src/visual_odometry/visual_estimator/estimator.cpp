#include "estimator.h"

Estimator::Estimator(): Node("estimator"), f_manager{Rs}
{
    failureCount = -1;
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    ++failureCount;

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

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, 
                             const vector<float> &lidar_initialization_info,
                             const std_msgs::msg::Header &header)
{
    // Add new image features
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    // Marginalize old imgs if lidar odometry available for initialization
    if (solver_flag == INITIAL && lidar_initialization_info[0] >= 0)
        marginalization_flag = MARGIN_OLD;

    Headers[frame_count] = header;

    ImageFrame imageframe(image, lidar_initialization_info, rclcpp::Time(header.stamp).seconds());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(rclcpp::Time(header.stamp).seconds(), imageframe));
    
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // Calibrate rotational extrinsics
    if(ESTIMATE_EXTRINSIC == 2)
    {
        // RCLCPP_WARN(rclcpp::get_logger("calibration"), "calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                // RCLCPP_WARN(rclcpp::get_logger("calibration"), "initial extrinsic rotation calib success");
                // RCLCPP_WARN_STREAM(rclcpp::get_logger("calibration"), "initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (rclcpp::Time(header.stamp).seconds() - initial_timestamp) > 0.1)
            {
               result = initialStructure();
               initial_timestamp = rclcpp::Time(header.stamp).seconds();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                RCLCPP_INFO(this->get_logger(), "Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else
    {
        solveOdometry();

        if (failureDetection())
        {
            RCLCPP_ERROR(this->get_logger(), "VINS failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            RCLCPP_ERROR(this->get_logger(), "VINS system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();

        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    // Lidar initialization
    {
        bool lidar_info_available = true;

        // clear key frame in the container        
        for (map<double, ImageFrame>::iterator frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
            frame_it->second.is_key_frame = false;

        // check if lidar info in the window is valid
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            if (all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].reset_id < 0 || 
                all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].reset_id != all_image_frame[rclcpp::Time(Headers[0].stamp).seconds()].reset_id)
            {
                // lidar odometry not available (id=-1) or lidar odometry relocated due to pose correction
                lidar_info_available = false;
                RCLCPP_INFO(this->get_logger(), "Lidar initialization info not enough.");
                break;
            }
        }

        if (lidar_info_available == true)
        {
            // Update state
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Ps[i] = all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].T;
                Rs[i] = all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].R;
                Vs[i] = all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].V;
                Bas[i] = all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].Ba;
                Bgs[i] = all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].Bg;

                pre_integrations[i]->repropagate(Bas[i], Bgs[i]);

                all_image_frame[rclcpp::Time(Headers[i].stamp).seconds()].is_key_frame = true;
            }

            // update gravity
            g = Eigen::Vector3d(0, 0, all_image_frame[rclcpp::Time(Headers[0].stamp).seconds()].gravity);

            // reset all features
            VectorXd dep = f_manager.getDepthVector();
            for (int i = 0; i < dep.size(); i++)
                dep[i] = -1;
            f_manager.clearDepth(dep);

            // triangulate all features
            Vector3d TIC_TMP[NUM_OF_CAM];
            for(int i = 0; i < NUM_OF_CAM; i++)
                TIC_TMP[i].setZero();
            ric[0] = RIC[0];
            f_manager.setRic(ric);
            f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

            return true;
        }
    }

    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //RCLCPP_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            RCLCPP_INFO(this->get_logger(), "Trying to initialize VINS, IMU excitation not enough!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        RCLCPP_INFO(this->get_logger(), "Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        RCLCPP_DEBUG(this->get_logger(), "global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == rclcpp::Time(Headers[i].stamp).seconds())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > rclcpp::Time(Headers[i].stamp).seconds())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            RCLCPP_DEBUG(this->get_logger(), "Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            RCLCPP_DEBUG(this->get_logger(), "solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
   
    if (visualInitialAlign())
        return true;
    else
    {
        RCLCPP_INFO(this->get_logger(), "misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        RCLCPP_INFO(this->get_logger(), "solve gravity failed, try again!");
        return false;
    }

    // changeã^·¶‰žËkºwµçUÉ}™É…µ•lÁt¹ÕØ¹ä ¤°¥Ñ}Á•É}™É…µ”¹ÕØ¹ä ¤¤ì4(€€€€€€€€€€€€€€€ÁÉ½‰±•´¹‘‘I•Í¥‘Õ…±	±½¬¡™}Ñ°±½ÍÍ}™Õ¹Ñ¥½¸°Á…É…}A½Í•m¥µÕ}¥t°Á…É…}A½Í•m¥µÕ}©t°Á…É…}á}A½Í•lÁt°Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•át°Á…É…}Q‘lÁt¤ì4(€€€€€€€€€€€€€€€€4(€€€€€€€€€€€€€€€€¼¼‘•ÁÑ ¥Ì½‰Ñ…¥¹•™É½´±¥‘…È°Í­¥À½ÁÑ¥µ¥é¥¹œ¥Ð4(€€€€€€€€€€€€€€€¥˜€¡¥Ñ}Á•É}¥¹±¥‘…É}‘•ÁÑ¡}™±…œ€ôôÑÉÕ”¤4(€€€€€€€€€€€€€€€€€€€ÁÉ½‰±•´¹M•ÑA…É…µ•Ñ•É	±½­½¹ÍÑ…¹Ð¡Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•át¤ì4(€€€€€€€€€€€ô4(€€€€€€€€€€€•±Í”4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€AÉ½©•Ñ¥½¹…Ñ½È€©˜€ô¹•ÜAÉ½©•Ñ¥½¹…Ñ½È¡ÁÑÍ}¤°ÁÑÍ}¨¤ì4(€€€€€€€€€€€€€€€ÁÉ½‰±•´¹‘‘I•Í¥‘Õ…±	±½¬¡˜°±½ÍÍ}™Õ¹Ñ¥½¸°Á…É…}A½Í•m¥µÕ}¥t°Á…É…}A½Í•m¥µÕ}©t°Á…É…}á}A½Í•lÁt°Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•át¤ì4(4(€€€€€€€€€€€€€€€€¼¼‘•ÁÑ ¥Ì½‰Ñ…¥¹•™É½´±¥‘…È°Í­¥À½ÁÑ¥µ¥é¥¹œ¥Ð4(€€€€€€€€€€€€€€€¥˜€¡¥Ñ}Á•É}¥¹±¥‘…É}‘•ÁÑ¡}™±…œ€ôôÑÉÕ”¤4(€€€€€€€€€€€€€€€€€€€ÁÉ½‰±•´¹M•ÑA…É…µ•Ñ•É	±½­½¹ÍÑ…¹Ð¡Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•át¤ì4(€€€€€€€€€€€ô4(€€€€€€€€€€€™}µ}¹Ð¬¬ì4(€€€€€€€ô4(€€€ô4(4(€€€•É•ÌèéM½±Ù•Èèé=ÁÑ¥½¹Ì½ÁÑ¥½¹Ìì4(€€€½ÁÑ¥½¹Ì¹±¥¹•…É}Í½±Ù•É}ÑåÁ”€ô•É•Ìèé9M}M!UHì4(€€€€¼½½ÁÑ¥½¹Ì¹¹Õµ}Ñ¡É•…‘Ì€ô€Èì4(€€€½ÁÑ¥½¹Ì¹ÑÉÕÍÑ}É•¥½¹}ÍÑÉ…Ñ•å}ÑåÁ”€ô•É•Ìèé=1ì4(€€€½ÁÑ¥½¹Ì¹µ…á}¹Õµ}¥Ñ•É…Ñ¥½¹Ì€ô9U5}%QIQ%=9Lì4(€€€€¼½½ÁÑ¥½¹Ì¹ÕÍ•}•áÁ±¥¥Ñ}Í¡ÕÉ}½µÁ±•µ•¹Ð€ôÑÉÕ”ì4(€€€€¼½½ÁÑ¥½¹Ì¹µ¥¹¥µ¥é•É}ÁÉ½É•ÍÍ}Ñ½}ÍÑ‘½ÕÐ€ôÑÉÕ”ì4(€€€€¼½½ÁÑ¥½¹Ì¹ÕÍ•}¹½¹µ½¹½Ñ½¹¥}ÍÑ•ÁÌ€ôÑÉÕ”ì4(4(€€€¥˜€¡µ…É¥¹…±¥é…Ñ¥½¹}™±…œ€ôô5I%9}=1¤4(€€€€€€€½ÁÑ¥½¹Ì¹µ…á}Í½±Ù•É}Ñ¥µ•}¥¹}Í•½¹‘Ì€ôM=1YI}Q%5€¨€Ð¸À€¼€Ô¸Àì4(€€€•±Í”4(€€€€€€€½ÁÑ¥½¹Ì¹µ…á}Í½±Ù•É}Ñ¥µ•}¥¹}Í•½¹‘Ì€ôM=1YI}Q%5ì4(4(€€€•É•ÌèéM½±Ù•ÈèéMÕµµ…ÉäÍÕµµ…Éäì4(€€€•É•ÌèéM½±Ù”¡½ÁÑ¥½¹Ì°€™ÁÉ½‰±•´°€™ÍÕµµ…Éä¤ì4(4(€€€‘½Õ‰±”ÉÙ•Ñ½È ¤ì4(4(€€€¥˜€¡µ…É¥¹…±¥é…Ñ¥½¹}™±…œ€ôô5I%9}=1¤4(€€€ì4(€€€€€€€5…É¥¹…±¥é…Ñ¥½¹%¹™¼€©µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼€ô¹•Ü5…É¥¹…±¥é…Ñ¥½¹%¹™¼ ¤ì4(€€€€€€€Ù•Ñ½ÈÉ‘½Õ‰±” ¤ì4(4(€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤4(€€€€€€€ì4(€€€€€€€€€€€Ù•Ñ½Èñ¥¹Ðø‘É½Á}Í•Ðì4(€€€€€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ðÍÑ…Ñ¥}…ÍÐñ¥¹Ðø¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì¹Í¥é” ¤¤ì¤¬¬¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ím¥t€ôôÁ…É…}A½Í•lÁtñð4(€€€€€€€€€€€€€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ím¥t€ôôÁ…É…}MÁ••‘	¥…ÍlÁt¤4(€€€€€€€€€€€€€€€€€€€‘É½Á}Í•Ð¹ÁÕÍ¡}‰…¬¡¤¤ì4(€€€€€€€€€€€ô4(€€€€€€€€€€€€¼¼½¹ÍÑÉÕÐ¹•Üµ…É¥¹±¥é…Ñ¥½¹}™…Ñ½È4(€€€€€€€€€€€5…É¥¹…±¥é…Ñ¥½¹…Ñ½È€©µ…É¥¹…±¥é…Ñ¥½¹}™…Ñ½È€ô¹•Ü5…É¥¹…±¥é…Ñ¥½¹…Ñ½È¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤ì4(€€€€€€€€€€€I•Í¥‘Õ…±	±½­%¹™¼€©É•Í¥‘Õ…±}‰±½­}¥¹™¼€ô¹•ÜI•Í¥‘Õ…±	±½­%¹™¼¡µ…É¥¹…±¥é…Ñ¥½¹}™…Ñ½È°9U10°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€‘É½Á}Í•Ð¤ì4(4(€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù…‘‘I•Í¥‘Õ…±	±½­%¹™¼¡É•Í¥‘Õ…±}‰±½­}¥¹™¼¤ì4(€€€€€€€ô4(4(€€€€€€€ì4(€€€€€€€€€€€¥˜€¡ÁÉ•}¥¹Ñ•É…Ñ¥½¹ÍlÅt´ùÍÕµ}‘Ð€ð€ÄÀ¸À¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€%5U…Ñ½È¨¥µÕ}™…Ñ½È€ô¹•Ü%5U…Ñ½È¡ÁÉ•}¥¹Ñ•É…Ñ¥½¹ÍlÅt¤ì4(€€€€€€€€€€€€€€€I•Í¥‘Õ…±	±½­%¹™¼€©É•Í¥‘Õ…±}‰±½­}¥¹™¼€ô¹•ÜI•Í¥‘Õ…±	±½­%¹™¼¡¥µÕ}™…Ñ½È°9U10°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ‘½Õ‰±”€¨ùíÁ…É…}A½Í•lÁt°Á…É…}MÁ••‘	¥…ÍlÁt°Á…É…}A½Í•lÅt°Á…É…}MÁ••‘	¥…ÍlÅuô°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ¥¹ÐùìÀ°€Åô¤ì4(€€€€€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù…‘‘I•Í¥‘Õ…±	±½­%¹™¼¡É•Í¥‘Õ…±}‰±½­}¥¹™¼¤ì4(€€€€€€€€€€€ô4(€€€€€€€ô4(4(€€€€€€€ì4(€€€€€€€€€€€¥¹Ð™•…ÑÕÉ•}¥¹‘•à€ô€´Äì4(€€€€€€€€€€€™½È€¡…ÕÑ¼€™¥Ñ}Á•É}¥€è™}µ…¹…•È¹™•…ÑÕÉ”¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€¥Ñ}Á•É}¥¹ÕÍ•‘}¹Õ´€ô¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ”¹Í¥é” ¤ì4(€€€€€€€€€€€€€€€¥˜€ „¡¥Ñ}Á•É}¥¹ÕÍ•‘}¹Õ´€øô€È€˜˜¥Ñ}Á•É}¥¹ÍÑ…ÉÑ}™É…µ”€ð]%9=]}M%i€´€È¤¤4(€€€€€€€€€€€€€€€€€€€½¹Ñ¥¹Õ”ì4(4(€€€€€€€€€€€€€€€€¬­™•…ÑÕÉ•}¥¹‘•àì4(4(€€€€€€€€€€€€€€€¥¹Ð¥µÕ}¤€ô¥Ñ}Á•É}¥¹ÍÑ…ÉÑ}™É…µ”°¥µÕ}¨€ô¥µÕ}¤€´€Äì4(€€€€€€€€€€€€€€€¥˜€¡¥µÕ}¤€„ô€À¤4(€€€€€€€€€€€€€€€€€€€½¹Ñ¥¹Õ”ì4(4(€€€€€€€€€€€€€€€Y•Ñ½ÈÍÁÑÍ}¤€ô¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ•lÁt¹Á½¥¹Ðì4(4(€€€€€€€€€€€€€€€™½È€¡…ÕÑ¼€™¥Ñ}Á•É}™É…µ”€è¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ”¤4(€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€¥µÕ}¨¬¬ì4(€€€€€€€€€€€€€€€€€€€¥˜€¡¥µÕ}¤€ôô¥µÕ}¨¤4(€€€€€€€€€€€€€€€€€€€€€€€½¹Ñ¥¹Õ”ì4(4(€€€€€€€€€€€€€€€€€€€Y•Ñ½ÈÍÁÑÍ}¨€ô¥Ñ}Á•É}™É…µ”¹Á½¥¹Ðì4(€€€€€€€€€€€€€€€€€€€¥˜€¡MQ%5Q}Q¤4(€€€€€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€€€€€AÉ½©•Ñ¥½¹Q‘…Ñ½È€©™}Ñ€ô¹•ÜAÉ½©•Ñ¥½¹Q‘…Ñ½È¡ÁÑÍ}¤°ÁÑÍ}¨°¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ•lÁt¹Ù•±½¥Ñä°¥Ñ}Á•É}™É…µ”¹Ù•±½¥Ñä°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ•lÁt¹ÕÉ}Ñ°¥Ñ}Á•É}™É…µ”¹ÕÉ}Ñ°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€¥Ñ}Á•É}¥¹™•…ÑÕÉ•}Á•É}™É…µ•lÁt¹ÕØ¹ä ¤°¥Ñ}Á•É}™É…µ”¹ÕØ¹ä ¤¤ì4(€€€€€€€€€€€€€€€€€€€€€€€I•Í¥‘Õ…±	±½­%¹™¼€©É•Í¥‘Õ…±}‰±½­}¥¹™¼€ô¹•ÜI•Í¥‘Õ…±	±½­%¹™¼¡™}Ñ°±½ÍÍ}™Õ¹Ñ¥½¸°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ‘½Õ‰±”€¨ùíÁ…É…}A½Í•m¥µÕ}¥t°Á…É…}A½Í•m¥µÕ}©t°Á…É…}á}A½Í•lÁt°Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•át°Á…É…}Q‘lÁuô°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ¥¹ÐùìÀ°€Íô¤ì4(€€€€€€€€€€€€€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù…‘‘I•Í¥‘Õ…±	±½­%¹™¼¡É•Í¥‘Õ…±}‰±½­}¥¹™¼¤ì4(€€€€€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€€€€€€€€€•±Í”4(€€€€€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€€€€€AÉ½©•Ñ¥½¹…Ñ½È€©˜€ô¹•ÜAÉ½©•Ñ¥½¹…Ñ½È¡ÁÑÍ}¤°ÁÑÍ}¨¤ì4(€€€€€€€€€€€€€€€€€€€€€€€I•Í¥‘Õ…±	±½­%¹™¼€©É•Í¥‘Õ…±}‰±½­}¥¹™¼€ô¹•ÜI•Í¥‘Õ…±	±½­%¹™¼¡˜°±½ÍÍ}™Õ¹Ñ¥½¸°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ‘½Õ‰±”€¨ùíÁ…É…}A½Í•m¥µÕ}¥t°Á…É…}A½Í•m¥µÕ}©t°Á…É…}á}A½Í•lÁt°Á…É…}•…ÑÕÉ•m™•…ÑÕÉ•}¥¹‘•áuô°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€Ù•Ñ½Èñ¥¹ÐùìÀ°€Íô¤ì4(€€€€€€€€€€€€€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù…‘‘I•Í¥‘Õ…±	±½­%¹™¼¡É•Í¥‘Õ…±}‰±½­}¥¹™¼¤ì4(€€€€€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€ô4(€€€€€€€ô4(4(€€€€€€€Q¥Q½ŒÑ}ÁÉ•}µ…É¥¸ì4(€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ùÁÉ•5…É¥¹…±¥é” ¤ì4(€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰ÁÉ”µ…É¥¹…±¥é…Ñ¥½¸€•˜µÌˆ°Ñ}ÁÉ•}µ…É¥¸¹Ñ½Œ ¤¤ì4(€€€€€€€€4(€€€€€€€Q¥Q½ŒÑ}µ…É¥¸ì4(€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ùµ…É¥¹…±¥é” ¤ì4(€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰µ…É¥¹…±¥é…Ñ¥½¸€•˜µÌˆ°Ñ}µ…É¥¸¹Ñ½Œ ¤¤ì4(4(€€€€€€€ÍÑèéÕ¹½É‘•É•‘}µ…Àñ±½¹œ°‘½Õ‰±”€¨ø…‘‘É}Í¡¥™Ðì4(€€€€€€€™½È€¡¥¹Ð¤€ô€Äì¤€ðô]%9=]}M%iì¤¬¬¤4(€€€€€€€ì4(€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}A½Í•m¥t¥t€ôÁ…É…}A½Í•m¤€´€Åtì4(€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}MÁ••‘	¥…Ím¥t¥t€ôÁ…É…}MÁ••‘	¥…Ím¤€´€Åtì4(€€€€€€€ô4(€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ð9U5}=}4ì¤¬¬¤4(€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}á}A½Í•m¥t¥t€ôÁ…É…}á}A½Í•m¥tì4(€€€€€€€¥˜€¡MQ%5Q}Q¤4(€€€€€€€ì4(€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}Q‘lÁt¥t€ôÁ…É…}Q‘lÁtì4(€€€€€€€ô4(€€€€€€€Ù•Ñ½Èñ‘½Õ‰±”€¨øÁ…É…µ•Ñ•É}‰±½­Ì€ôµ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù•ÑA…É…µ•Ñ•É	±½­Ì¡…‘‘É}Í¡¥™Ð¤ì4(4(€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤4(€€€€€€€€€€€‘•±•Ñ”±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼ì4(€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼€ôµ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼ì4(€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì€ôÁ…É…µ•Ñ•É}‰±½­Ìì4(€€€€€€€€4(€€€ô4(€€€•±Í”4(€€€ì4(€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼€˜˜4(€€€€€€€€€€€ÍÑèé½Õ¹Ð¡ÍÑèé‰•¥¸¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì¤°ÍÑèé•¹¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì¤°Á…É…}A½Í•m]%9=]}M%i€´€Åt¤¤4(€€€€€€€ì4(4(€€€€€€€€€€€5…É¥¹…±¥é…Ñ¥½¹%¹™¼€©µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼€ô¹•Ü5…É¥¹…±¥é…Ñ¥½¹%¹™¼ ¤ì4(€€€€€€€€€€€Ù•Ñ½ÈÉ‘½Õ‰±” ¤ì4(€€€€€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€Ù•Ñ½Èñ¥¹Ðø‘É½Á}Í•Ðì4(€€€€€€€€€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ðÍÑ…Ñ¥}…ÍÐñ¥¹Ðø¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì¹Í¥é” ¤¤ì¤¬¬¤4(€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€…ÍÍ•ÉÐ¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ím¥t€„ôÁ…É…}MÁ••‘	¥…Ím]%9=]}M%i€´€Åt¤ì4(€€€€€€€€€€€€€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ím¥t€ôôÁ…É…}A½Í•m]%9=]}M%i€´€Åt¤4(€€€€€€€€€€€€€€€€€€€€€€€‘É½Á}Í•Ð¹ÁÕÍ¡}‰…¬¡¤¤ì4(€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€€€€€€¼¼½¹ÍÑÉÕÐ¹•Üµ…É¥¹±¥é…Ñ¥½¹}™…Ñ½È4(€€€€€€€€€€€€€€€5…É¥¹…±¥é…Ñ¥½¹…Ñ½È€©µ…É¥¹…±¥é…Ñ¥½¹}™…Ñ½È€ô¹•Ü5…É¥¹…±¥é…Ñ¥½¹…Ñ½È¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤ì4(€€€€€€€€€€€€€€€I•Í¥‘Õ…±	±½­%¹™¼€©É•Í¥‘Õ…±}‰±½­}¥¹™¼€ô¹•ÜI•Í¥‘Õ…±	±½­%¹™¼¡µ…É¥¹…±¥é…Ñ¥½¹}™…Ñ½È°9U10°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì°4(€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€€‘É½Á}Í•Ð¤ì4(4(€€€€€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù…‘‘I•Í¥‘Õ…±	±½­%¹™¼¡É•Í¥‘Õ…±}‰±½­}¥¹™¼¤ì4(€€€€€€€€€€€ô4(4(€€€€€€€€€€€Q¥Q½ŒÑ}ÁÉ•}µ…É¥¸ì4(€€€€€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰‰•¥¸µ…É¥¹…±¥é…Ñ¥½¸ˆ¤ì4(€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ùÁÉ•5…É¥¹…±¥é” ¤ì4(€€€€€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰•¹ÁÉ”µ…É¥¹…±¥é…Ñ¥½¸°€•˜µÌˆ°Ñ}ÁÉ•}µ…É¥¸¹Ñ½Œ ¤¤ì4(4(€€€€€€€€€€€Q¥Q½ŒÑ}µ…É¥¸ì4(€€€€€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰‰•¥¸µ…É¥¹…±¥é…Ñ¥½¸ˆ¤ì4(€€€€€€€€€€€µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ùµ…É¥¹…±¥é” ¤ì4(€€€€€€€€€€€I1AA}	U¡Ñ¡¥Ì´ù•Ñ}±½•È ¤°€‰•¹µ…É¥¹…±¥é…Ñ¥½¸°€•˜µÌˆ°Ñ}µ…É¥¸¹Ñ½Œ ¤¤ì4(€€€€€€€€€€€€4(€€€€€€€€€€€ÍÑèéÕ¹½É‘•É•‘}µ…Àñ±½¹œ°‘½Õ‰±”€¨ø…‘‘É}Í¡¥™Ðì4(€€€€€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ðô]%9=]}M%iì¤¬¬¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€¥˜€¡¤€ôô]%9=]}M%i€´€Ä¤4(€€€€€€€€€€€€€€€€€€€½¹Ñ¥¹Õ”ì4(€€€€€€€€€€€€€€€•±Í”¥˜€¡¤€ôô]%9=]}M%i¤4(€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}A½Í•m¥t¥t€ôÁ…É…}A½Í•m¤€´€Åtì4(€€€€€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}MÁ••‘	¥…Ím¥t¥t€ôÁ…É…}MÁ••‘	¥…Ím¤€´€Åtì4(€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€€€€€•±Í”4(€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}A½Í•m¥t¥t€ôÁ…É…}A½Í•m¥tì4(€€€€€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}MÁ••‘	¥…Ím¥t¥t€ôÁ…É…}MÁ••‘	¥…Ím¥tì4(€€€€€€€€€€€€€€€ô4(€€€€€€€€€€€ô4(€€€€€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ð9U5}=}4ì¤¬¬¤4(€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}á}A½Í•m¥t¥t€ôÁ…É…}á}A½Í•m¥tì4(€€€€€€€€€€€¥˜€¡MQ%5Q}Q¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€…‘‘É}Í¡¥™ÑmÉ•¥¹Ñ•ÉÁÉ•Ñ}…ÍÐñ±½¹œø¡Á…É…}Q‘lÁt¥t€ôÁ…É…}Q‘lÁtì4(€€€€€€€€€€€ô4(€€€€€€€€€€€€4(€€€€€€€€€€€Ù•Ñ½Èñ‘½Õ‰±”€¨øÁ…É…µ•Ñ•É}‰±½­Ì€ôµ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼´ù•ÑA…É…µ•Ñ•É	±½­Ì¡…‘‘É}Í¡¥™Ð¤ì4(€€€€€€€€€€€¥˜€¡±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼¤4(€€€€€€€€€€€€€€€‘•±•Ñ”±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼ì4(€€€€€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼€ôµ…É¥¹…±¥é…Ñ¥½¹}¥¹™¼ì4(€€€€€€€€€€€±…ÍÑ}µ…É¥¹…±¥é…Ñ¥½¹}Á…É…µ•Ñ•É}‰±½­Ì€ôÁ…É…µ•Ñ•É}‰±½­Ìì4(€€€€€€€€€€€€4(€€€€€€€ô4(€€€ô4)ô4(4)Ù½¥ÍÑ¥µ…Ñ½ÈèéÍ±¥‘•]¥¹‘½Ü ¤4)ì4(€€€Q¥Q½ŒÑ}µ…É¥¸ì4(€€€¥˜€¡µ…É¥¹…±¥é…Ñ¥½¹}™±…œ€ôô5I%9}=1¤4(€€€ì4(€€€€€€€‘½Õ‰±”Ñ|À€ôÉ±ÁÀèéQ¥µ”¡!•…‘•ÉÍlÁt¹ÍÑ…µÀ¤¹Í•½¹‘Ì ¤ì4(€€€€€€€‰…­}HÀ€ôIÍlÁtì4(€€€€€€€‰…­}@À€ôAÍlÁtì4(€€€€€€€¥˜€¡™É…µ•}½Õ¹Ð€ôô]%9=]}M%i¤4(€€€€€€€ì4(€€€€€€€€€€€™½È€¡¥¹Ð¤€ô€Àì¤€ð]%9=]}M%iì¤¬¬¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€IÍm¥t¹ÍÝ…À¡IÍm¤€¬€Åt¤ì4(4(€€€€€€€€€€€€€€€ÍÑèéÍÝ…À¡ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím¥t°ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím¤€¬€Åt¤ì4(4(€€€€€€€€€€€€€€€‘Ñ}‰Õ™m¥t¹ÍÝ…À¡‘Ñ}‰Õ™m¤€¬€Åt¤ì4(€€€€€€€€€€€€€€€±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m¥t¹ÍÝ…À¡±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m¤€¬€Åt¤ì4(€€€€€€€€€€€€€€€…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m¥t¹ÍÝ…À¡…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m¤€¬€Åt¤ì4(4(€€€€€€€€€€€€€€€!•…‘•ÉÍm¥t€ô!•…‘•ÉÍm¤€¬€Åtì4(€€€€€€€€€€€€€€€AÍm¥t¹ÍÝ…À¡AÍm¤€¬€Åt¤ì4(€€€€€€€€€€€€€€€YÍm¥t¹ÍÝ…À¡YÍm¤€¬€Åt¤ì4(€€€€€€€€€€€€€€€	…Ím¥t¹ÍÝ…À¡	…Ím¤€¬€Åt¤ì4(€€€€€€€€€€€€€€€	Ím¥t¹ÍÝ…À¡	Ím¤€¬€Åt¤ì4(€€€€€€€€€€€ô4(€€€€€€€€€€€!•…‘•ÉÍm]%9=]}M%it€ô!•…‘•ÉÍm]%9=]}M%i€´€Åtì4(€€€€€€€€€€€AÍm]%9=]}M%it€ôAÍm]%9=]}M%i€´€Åtì4(€€€€€€€€€€€YÍm]%9=]}M%it€ôYÍm]%9=]}M%i€´€Åtì4(€€€€€€€€€€€IÍm]%9=]}M%it€ôIÍm]%9=]}M%i€´€Åtì4(€€€€€€€€€€€	…Ím]%9=]}M%it€ô	…Ím]%9=]}M%i€´€Åtì4(€€€€€€€€€€€	Ím]%9=]}M%it€ô	Ím]%9=]}M%i€´€Åtì4(4(€€€€€€€€€€€‘•±•Ñ”ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím]%9=]}M%itì4(€€€€€€€€€€€ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím]%9=]}M%it€ô¹•Ü%¹Ñ•É…Ñ¥½¹	…Í•í…|À°åÉ|À°	…Ím]%9=]}M%it°	Ím]%9=]}M%iuôì4(4(€€€€€€€€€€€‘Ñ}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(€€€€€€€€€€€±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(€€€€€€€€€€€…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(4(€€€€€€€€€€€¥˜€¡ÑÉÕ”ñðÍ½±Ù•É}™±…œ€ôô%9%Q%0¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€µ…Àñ‘½Õ‰±”°%µ…•É…µ”øèé¥Ñ•É…Ñ½È¥Ñ|Àì4(€€€€€€€€€€€€€€€¥Ñ|À€ô…±±}¥µ…•}™É…µ”¹™¥¹¡Ñ|À¤ì4(€€€€€€€€€€€€€€€‘•±•Ñ”¥Ñ|À´ùÍ•½¹¹ÁÉ•}¥¹Ñ•É…Ñ¥½¸ì4(€€€€€€€€€€€€€€€¥Ñ|À´ùÍ•½¹¹ÁÉ•}¥¹Ñ•É…Ñ¥½¸€ô¹Õ±±ÁÑÈì4(€4(€€€€€€€€€€€€€€€™½È€¡µ…Àñ‘½Õ‰±”°%µ…•É…µ”øèé¥Ñ•É…Ñ½È¥Ð€ô…±±}¥µ…•}™É…µ”¹‰•¥¸ ¤ì¥Ð€„ô¥Ñ|Àì€¬­¥Ð¤4(€€€€€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€€€€€¥˜€¡¥Ð´ùÍ•½¹¹ÁÉ•}¥¹Ñ•É…Ñ¥½¸¤4(€€€€€€€€€€€€€€€€€€€€€€€‘•±•Ñ”¥Ð´ùÍ•½¹¹ÁÉ•}¥¹Ñ•É…Ñ¥½¸ì4(€€€€€€€€€€€€€€€€€€€¥Ð´ùÍ•½¹¹ÁÉ•}¥¹Ñ•É…Ñ¥½¸€ô9U10ì4(€€€€€€€€€€€€€€€ô4(4(€€€€€€€€€€€€€€€…±±}¥µ…•}™É…µ”¹•É…Í”¡…±±}¥µ…•}™É…µ”¹‰•¥¸ ¤°¥Ñ|À¤ì4(€€€€€€€€€€€€€€€…±±}¥µ…•}™É…µ”¹•É…Í”¡Ñ|À¤ì4(4(€€€€€€€€€€€ô4(€€€€€€€€€€€Í±¥‘•]¥¹‘½Ý=± ¤ì4(€€€€€€€ô4(€€€ô4(€€€•±Í”4(€€€ì4(€€€€€€€¥˜€¡™É…µ•}½Õ¹Ð€ôô]%9=]}M%i¤4(€€€€€€€ì4(€€€€€€€€€€€™½È€¡Õ¹Í¥¹•¥¹Ð¤€ô€Àì¤€ð‘Ñ}‰Õ™m™É…µ•}½Õ¹Ñt¹Í¥é” ¤ì¤¬¬¤4(€€€€€€€€€€€ì4(€€€€€€€€€€€€€€€‘½Õ‰±”ÑµÁ}‘Ð€ô‘Ñ}‰Õ™m™É…µ•}½Õ¹Ñum¥tì4(€€€€€€€€€€€€€€€Y•Ñ½ÈÍÑµÁ}±¥¹•…É}…•±•É…Ñ¥½¸€ô±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m™É…µ•}½Õ¹Ñum¥tì4(€€€€€€€€€€€€€€€Y•Ñ½ÈÍÑµÁ}…¹Õ±…É}Ù•±½¥Ñä€ô…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m™É…µ•}½Õ¹Ñum¥tì4(4(€€€€€€€€€€€€€€€ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím™É…µ•}½Õ¹Ð€´€Åt´ùÁÕÍ¡}‰…¬¡ÑµÁ}‘Ð°ÑµÁ}±¥¹•…É}…•±•É…Ñ¥½¸°ÑµÁ}…¹Õ±…É}Ù•±½¥Ñä¤ì4(4(€€€€€€€€€€€€€€€‘Ñ}‰Õ™m™É…µ•}½Õ¹Ð€´€Åt¹ÁÕÍ¡}‰…¬¡ÑµÁ}‘Ð¤ì4(€€€€€€€€€€€€€€€±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m™É…µ•}½Õ¹Ð€´€Åt¹ÁÕÍ¡}‰…¬¡ÑµÁ}±¥¹•…É}…•±•É…Ñ¥½¸¤ì4(€€€€€€€€€€€€€€€…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m™É…µ•}½Õ¹Ð€´€Åt¹ÁÕÍ¡}‰…¬¡ÑµÁ}…¹Õ±…É}Ù•±½¥Ñä¤ì4(€€€€€€€€€€€ô4(4(€€€€€€€€€€€!•…‘•ÉÍm™É…µ•}½Õ¹Ð€´€Åt€ô!•…‘•ÉÍm™É…µ•}½Õ¹Ñtì4(€€€€€€€€€€€AÍm™É…µ•}½Õ¹Ð€´€Åt€ôAÍm™É…µ•}½Õ¹Ñtì4(€€€€€€€€€€€YÍm™É…µ•}½Õ¹Ð€´€Åt€ôYÍm™É…µ•}½Õ¹Ñtì4(€€€€€€€€€€€IÍm™É…µ•}½Õ¹Ð€´€Åt€ôIÍm™É…µ•}½Õ¹Ñtì4(€€€€€€€€€€€	…Ím™É…µ•}½Õ¹Ð€´€Åt€ô	…Ím™É…µ•}½Õ¹Ñtì4(€€€€€€€€€€€	Ím™É…µ•}½Õ¹Ð€´€Åt€ô	Ím™É…µ•}½Õ¹Ñtì4(4(€€€€€€€€€€€‘•±•Ñ”ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím]%9=]}M%itì4(€€€€€€€€€€€ÁÉ•}¥¹Ñ•É…Ñ¥½¹Ím]%9=]}M%it€ô¹•Ü%¹Ñ•É…Ñ¥½¹	…Í•í…|À°åÉ|À°	…Ím]%9=]}M%it°	Ím]%9=]}M%iuôì4(4(€€€€€€€€€€€‘Ñ}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(€€€€€€€€€€€±¥¹•…É}…•±•É…Ñ¥½¹}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(€€€€€€€€€€€…¹Õ±…É}Ù•±½¥Ñå}‰Õ™m]%9=]}M%it¹±•…È ¤ì4(4(€€€€€€€€€€€Í±¥‘•]¥¹‘½Ý9•Ü ¤ì4(€€€€€€€ô4(€€€ô4)ô4(4(¼¼É•…°µ…É¥¹…±¥é…Ñ¥½¸¥ÌÉ•µ½Ù•¥¸Í½±Ù•}•É•Ì ¤4)Ù½¥ÍÑ¥µ…Ñ½ÈèéÍ±¥‘•]¥¹‘½Ý9•Ü ¤4)ì4(€€€ÍÕµ}½™}™É½¹Ð¬¬ì4(€€€™}µ…¹…•È¹É•µ½Ù•É½¹Ð¡™É…µ•}½Õ¹Ð¤ì4)ô4(¼¼É•…°µ…É¥¹…±¥é…Ñ¥½¸¥ÌÉ•µ½Ù•¥¸Í½±Ù•}•É•Ì ¤4)Ù½¥ÍÑ¥µ…Ñ½ÈèéÍ±¥‘•]¥¹‘½Ý=± ¤4)ì4(€€€ÍÕµ}½™}‰…¬¬¬ì4(4(€€€‰½½°Í¡¥™Ñ}‘•ÁÑ €ôÍ½±Ù•É}™±…œ€ôô9=9}1%9H€üÑÉÕ”€è™…±Í”ì4(€€€¥˜€¡Í¡¥™Ñ}‘•ÁÑ ¤4(€€€ì4(€€€€€€€5…ÑÉ¥àÍHÀ°HÄì4(€€€€€€€Y•Ñ½ÈÍ@À°@Äì4(€€€€€€€HÀ€ô‰…­}HÀ€¨É¥lÁtì4(€€€€€€€HÄ€ôIÍlÁt€¨É¥lÁtì4(€€€€€€€@À€ô‰…­}@À€¬‰…­}HÀ€¨Ñ¥lÁtì4(€€€€€€€@Ä€ôAÍlÁt€¬IÍlÁt€¨Ñ¥lÁtì4(€€€€€€€™}µ…¹…•È¹É•µ½Ù•	…­M¡¥™Ñ•ÁÑ ¡HÀ°@À°HÄ°@Ä¤ì4(€€€ô4(€€€•±Í”4(€€€€€€€™}µ…¹…•È¹É•µ½Ù•	…¬ ¤ì4)ô(