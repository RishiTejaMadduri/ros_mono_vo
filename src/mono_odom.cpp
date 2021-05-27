#include "mono_odom.h"


monoodom::monoodom():
it_(nh_)
{

    fn_calib_ = "/media/rishi/bridge/data_odometry_color/sequences/00/calib.txt";
    fn_poses_ = "/media/rishi/bridge/data_odometry_color/poses/00.txt";
    fn_images_ =  "/media/rishi/bridge/data_odometry_color/sequences/00/image_2/";

    // camInfo_ = nh_.subscribe("/camera/rgb/camera_info", 1, &monoodom::getCalib, this);
    // imageSub_ = it_.subscribe("/camera/rgb/image_raw", 1, &monoodom::imageCallBack, this, image_transport::TransportHints("compressed"));
    // imagePub_ = it_.advertise("/mono_vo/output", 1);
    pub_odom = nh_.advertise<nav_msgs::Odometry>("odom", 1);

    transform.frame_id_ = "/world";
    transform.child_frame_id_ = "/camera";
    max_frames = INT_MAX;
    min_points = 2000;
    n_frame = 0;
    odom_output = cv::Mat::zeros(600, 600, CV_8UC3);

    cv::namedWindow("Feature Tracking", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);

    count = 0;
    flow = 0;
    prev_count = 0;

    getCalib();
    imageCallBack();
}

/*
    Function for adding zeros in a string for loading kitti files.
*/

std::string monoodom::AddZeroPadding(const int value, const unsigned precision) 
{
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

/* 
    Function for computing the scale at different frames - Since monocular we have scale problem
    @param nframe - ith frame. 
*/

double monoodom::compute_scale(int nframe)
{
  std::string line;
  int i=0;
  std::ifstream fin(fn_poses_);
  double scale_;
  double x=0,y=0,z=0;
  double prev_x, prev_y, prev_z;

  if(fin.is_open()) {
    while((std::getline(fin, line)) && (i<=nframe)) {
      prev_z = z;
      prev_y = y;
      prev_x = x;

      std::istringstream iss(line);
      for(int j=0; j<12; j++) {
        iss>>z;
        if(j==7) y=z;
        if(j==3) x=z;
      }
      i++;
    }
    fin.close();
  }
  else {
    std::cout << "[-] Unable to open file: " << fn_poses_ << std::endl;
    scale_ = 0;
    return scale_;
  }

  scale_ = std::sqrt( (x-prev_x)*(x-prev_x) +
                      (y-prev_y)*(y-prev_y) +
                      (z-prev_z)*(z-prev_z));
    return scale_;
}
/*
    Function that converts ROS Sensor messages to OpenCV images and converts them to grayscale. - NOT BEING USED
    @param Image1 - ROS Sensor message corresponding to first image .
    @param Img1 - CV Mat that stores the OpenCV image of Image1 converted to grayscale.
    @param Img2 - CV Mat that stores the OpenCV image of Image2 converted to grayscale - Is usually a dummy matrix/empty matrix if not the first iteration.
    @param Image1 - ROS Sensor message corresponding to second image - Is usually a nullpointer if not the first iteration/initialization. 
*/

// bool monoodom::convertImages(const sensor_msgs::ImageConstPtr& Image1, const sensor_msgs::ImageConstPtr& Image2 = nullptr)
// {
//     cv_bridge::CvImageConstPtr cvbImg1, cvbImg2;
//     cv_bridge::CvImageConstPtr ImgPtr1;
//     ImgPtr1 = cv_bridge::toCvCopy(Image1);
//     curr_image = ImgPtr1->image;
//     if(!Image2)
//     {
//         try
//         {
//             cvbImg1 = cv_bridge::toCvCopy(Image1);
//         }
//         catch(const std::exception& e)
//         {
//             ROS_ERROR("CV BRIDGE EXCEPTION: %s ", e.what());
//             return false;
//         }
//         cv::cvtColor(cvbImg1->image, Img1, cv::COLOR_BGR2GRAY);
//         return true;
//     }
//     else
//     {
//         try
//         {
//             cvbImg1 = cv_bridge::toCvCopy(Image1);
//             cvbImg2 = cv_bridge::toCvCopy(Image2);
//         }
//         catch(const std::exception& e)
//         {
//             ROS_ERROR("CV BRIDGE EXCEPTION: %s ", e.what());
//             return false;
//         }
//         cv::cvtColor(cvbImg1->image, Img1, cv::COLOR_BGR2GRAY);
//         cv::cvtColor(cvbImg2->image, Img2, cv::COLOR_BGR2GRAY);

//         return true;
//     }

//     return false;
// }

    /*
        A callback function for the Camera info subscriber - 
        focal, pp and K are initialized here. 
    */
    void monoodom::getCalib()
    // void monoodom::getCalib(const sensor_msgs::CameraInfoConstPtr& info)
    {
        // if(!calibInit)
        // {
        //     // R = cv::Mat(3, 3, CV_64FC1, (void *)info->R.elems).clone();
        //     K = cv::Mat(3, 3, CV_64FC1, (void *)info->K.elems).clone();
        //     D = cv::Mat::zeros(1, 5, CV_64FC1);
        //     focal = info->K[0];
        //     pp.x = info->K[2];
        //     pp.y = info->K[5]; 

        //     //std::cout<<"CALIBRATION DONE"<<std::endl;
        //     calibInit = true;
        // }

        if(!calibInit)
        {
             std::ifstream fin(fn_calib_);
            std::string line;

            if(fin.is_open()) 
            {
                std::getline(fin, line);
                std::istringstream iss(line);

                for(int j=0; j<13; j++) 
                {
                    std::string token;
                    iss >> token;
                    if(j==1) {
                        focal = std::stod(token);   // focal length
                    }
                    if(j==3) {
                        pp.x = std::stod(token); // principal point x
                    }
                    if(j==7) {
                        pp.y = std::stod(token); // printcipal point y
                    }
                }
                fin.close();
            }
            else {
                std::cout << "[-] Cannot read the calibration file: " << fn_calib_ << std::endl;
                focal = 0;
                pp = cv::Point2f(0,0);
            }
            calibInit = true;
        }

    }

    /*
        Feature detection function for detecting fast features.
        @param Img1 - A CV Mat corresponding to Image in which features must be detected.
        @param keyPoints - detected features are stored in the keypoints array. 
    */
    void monoodom::FeatureDetection(cv::Mat Img1, std::vector<cv::Point2f>& keyPoints)
    {

        std::vector<cv::KeyPoint> kptsBuckted;
        int fast_threshold = 20;
        bool nms = true;
        FAST(Img1, kptsBuckted, fast_threshold, nms);

        std::sort(kptsBuckted.begin(), kptsBuckted.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                                            return a.response > b.response;
                                                            });
        for(int i = 0; i<2000; i++)
        {
            keyPoints.push_back(kptsBuckted[i].pt);
        }

    }

    /*
        Feature Tracking function for tracking features into subsequent images. 
        @param Img1 - A CV Mat corresponding to Image in which features were initially detected.
        @param Img2 - A CV Mat corresponding to Image in which features will be tracked from Img1.
        @param keyPoints1 - A vector with stored keyPoints from Img1.
        @param keyPoints2 - A vector with stored keyPoints from Img2.
    */
    void monoodom::FeatureMatching(cv::Mat Img1, cv::Mat Img2, std::vector<cv::Point2f>& keyPoints1, std::vector<cv::Point2f>& keyPoints2)
    {
        std::vector<float>err;
        cv::Size winSize = cv::Size(21, 21);
        cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
 
        cv::calcOpticalFlowPyrLK(Img1, Img2, keyPoints1, keyPoints2, status, err, winSize, 3, termcrit, 0, 0.001);
        int indexCorrection = 0;

        for(int i = 0; i<status.size(); i++)
        {
            cv::Point2f kp = keyPoints2.at(i - indexCorrection);
            if((status.at(i) == 0) || (kp.x<0) || (kp.y<0))
            {
                if((kp.x<0 || (kp.y<0)))
                {
                    status.at(i) = 0;
                }
                keyPoints1.erase(keyPoints1.begin() + (i-indexCorrection));
                keyPoints2.erase(keyPoints2.begin() + (i-indexCorrection));
                indexCorrection++;
            }
        }
    }

    /*
        Function to verify the validty of a rotation matrix R*Rt = I
        @param R - A CV Mat.
    */
    bool monoodom::IsRotMat(cv::Mat &R) 
    {
        cv::Mat Rt;
        transpose(R, Rt);
        cv::Mat shouldBeIdentity = Rt * R;
        cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

        return norm(I, shouldBeIdentity) < 1e-6;
    }
    /*
        Function to convert rotation matrix to Euler Angles
        @param R - A CV Mat.
    */
    cv::Vec3f monoodom::RotMatToEuler(cv::Mat &R, bool flag) 
    {
        assert(IsRotMat(R));
        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        bool singular = sy < 1e-6;
        float x, y, z;

        if (!singular) {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }

        if(flag)
        {
            std::string DIR = "/home/rishi/catkin_ws/src/ros_mono_vo/src/data_r.txt";
            std::ofstream outdata;

            outdata.open(DIR, std::ios::app);
            outdata<<x<<" "<<y<<" "<<z<<std::endl;
            outdata.close();
        }

        return cv::Vec3f(x, y, z);
    }
    //############### DEAD CODE BEGIN ###############

    // void print_t(cv::Mat &R)
    // {
    //     std::string DIR = "/home/rishi/catkin_ws/src/ros_mono_vo/src/data_r.txt";
    //     std::ofstream outdata;

    //     outdata.open(DIR, std::ios::app);
    //     outdata<<T.at<double>(0)<<" "<<T.at<double>(1)<<" "<<T.at<double>(2)<<std::endl;
    //     outdata.close();
    // }
    
    // void monoodom::imageCallBack(const sensor_msgs::ImageConstPtr& Image1)
    // {
    //     if(!calibInit)
    //     {
    //         ROS_ERROR("NO CALIB INFO FOUND");
    //         return;
    //     }

    //      image_cache.push(Image1);

    //      if(!odomInit && n_frame<2)
    //      {
    //          if(image_cache.size()>10)
    //          {
    //                 const sensor_msgs::ImageConstPtr Image1 = image_cache.front();
    //                 image_cache.pop();

    //                 int size = image_cache.size();

    //                 while(size != 1)
    //                 {
    //                     image_cache.pop();
    //                     size--;
    //                 }

    //                 const sensor_msgs::ImageConstPtr Image2 = image_cache.front();
    //                 image_cache.pop();

    //                 if(!convertImages(Image1, Image2))
    //                 {
    //                     ROS_WARN("FAILED IMAGE CONVERSION");
    //                     return;
    //                 }


    //                 FeatureDetection(Img1, points1);
    //                 FeatureMatching(Img1, Img2, points1, points2);
    //                 E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    //                 cv::recoverPose(E, points2, points1, R, curr_T, focal, pp, mask);

    //                 prev_image = Img2.clone();
    //                 prev_points = points2;
    //                 prev_R = R.clone();
    //                 prev_T = curr_T.clone();


    //                 ROS_INFO(" INITIALIZED \n");

    //                 odomInit = true;
    //                 n_frame = 2;
    //          }
    //      }
         
    //      else
    //      {

    //          if(image_cache.size()>=1)
    //          {
    //             if(n_frame>max_frames)
    //                 return;
                
    //             // while(image_cache.size()>1)
    //             // {
    //             //     image_cache.pop();
    //             // }

    //             const sensor_msgs::ImageConstPtr Image1 = image_cache.front();
    //             image_cache.pop();
    //             convertImages(Image1);
        
    //             FeatureMatching(prev_image, Img1, prev_points, curr_points);


    //             E = cv::findEssentialMat(curr_points, prev_points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    //             cv::recoverPose(E, curr_points, prev_points, R, curr_T, focal, pp, mask);

    //             scale = 2.0;
    //             if((scale>0.1)&&(curr_T.at<double>(2) > curr_T.at<double>(0)) && (curr_T.at<double>(2) > curr_T.at<double>(1)))
    //             {
    //                 count++;
    //                 prev_T = prev_T + scale*(prev_R*curr_T);
    //                 prev_R = R * prev_R;
    //                 // std::cout<<"Prev_R at: "<<count<<prev_R<<std::endl;
    //                 // std::cout<<prev_T<<std::endl;
     
    //             }

    //             if(prev_points.size()<min_points)
    //             {
    //                 FeatureDetection(prev_image, prev_points);
    //                 FeatureMatching(prev_image, Img1, prev_points, curr_points);
    //             }

    //             for(int i = 0; i<curr_points.size(); i++)
    //             {
    //                 idx.push_back(id++);
    //             }

    //             for(int i = 0; i<curr_points.size(); i++)
    //             {
    //                 prev_points_map.insert(std::make_pair(idx[i], curr_points[i]));
    //             }

    //             image_traj = Img1.clone();
    //             n_frame++;
    //             prev_image = Img1;
    //             prev_points = curr_points;
    //             visualize(n_frame);
    //             while(!image_cache.empty())
    //                 image_cache.pop();

    //             flow++;
    //          }
    //      }
    // }
    //############### DEAD CODE END ###############

    /*
        The main Image call back function that accepts ROS ImageConstPtr message. 
        -During the initialization phase 
        - Store images messages in a image_cache
        - Ensure cache size is greater than 2. 
        - Conver image messages to OpenCV BGR8 Image
        - Perform feature detction and Feature tracking
        - Estimate Essential Matrix and Recover Pose.
        -Initialize the odometry pipeline.

        -After Initialization
        - Track features into each sub-sequent images. 
        - Estimate essential matrix
        - Recover pose
        - Visualize.
    */
    void monoodom::imageCallBack()
    {
        if(!calibInit)
        {
            ROS_ERROR("NO CALIB INFO FOUND");
            return;
        }

         while(n_frame<max_frames)
         {
             if(!odomInit && n_frame<2)
            {
                scale = 1.0;
                std::string fn1 = fn_images_ + AddZeroPadding(0, 6) + ".png";
                std::string fn2 = fn_images_ + AddZeroPadding(1, 6) + ".png";

                cv::Mat Img1 = cv::imread(fn1, cv::IMREAD_GRAYSCALE);
                cv::Mat Img2 = cv::imread(fn2, cv::IMREAD_GRAYSCALE);

                FeatureDetection(Img1, points1);
                FeatureMatching(Img1, Img2, points1, points2);
                E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                cv::recoverPose(E, points2, points1, R, curr_T, focal, pp, mask);

                prev_image = Img2;
                prev_points = points2;
                prev_R = R.clone();
                prev_T = curr_T.clone();


                ROS_INFO(" INITIALIZED \n");

                odomInit = true;
                n_frame = 2;
            }
            
            if(odomInit && n_frame>=2)
            {

                if(n_frame>max_frames)
                    return;
                
                std::string fn = fn_images_ + AddZeroPadding(n_frame, 6) + ".png";
                curr_image = cv::imread(fn);
                cv::cvtColor(curr_image, curr_image_, cv::COLOR_BGR2GRAY);
                FeatureMatching(prev_image, curr_image_, prev_points, curr_points);
                E = cv::findEssentialMat(curr_points, prev_points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                cv::recoverPose(E, curr_points, prev_points, R, curr_T, focal, pp, mask);

                scale = compute_scale(n_frame);
                if((scale>0.1)&&(curr_T.at<double>(2) > curr_T.at<double>(0)) && (curr_T.at<double>(2) > curr_T.at<double>(1)))
                {

                    prev_T = prev_T + scale*(prev_R*curr_T);
                    prev_R = R * prev_R;        
                }

                if(prev_points.size()<min_points)
                {
                    FeatureDetection(prev_image, prev_points);
                    FeatureMatching(prev_image, curr_image_, prev_points, curr_points);
                }

                for(int i = 0; i<curr_points.size(); i++)
                {
                    idx.push_back(id++);
                }

                for(int i = 0; i<curr_points.size(); i++)
                {
                    prev_points_map.insert(std::make_pair(idx[i], curr_points[i]));
                }

                image_traj = curr_image.clone();
                visualize(n_frame);
                prev_image = curr_image_.clone();
                prev_points = curr_points;
                n_frame++;
                std::cout<<fn<<std::endl;
            }
            // sleep(1);
         }
    }
    
    /*
        A visualization function that plots trajectory and also publishes the odometry on RVIZ.
        -Tracks previous and curr feature points and plots them on the image stream to visualize features being tracked
        -Converts rotation matrix to euler angles and prints the rotation and translation.
        -Uses TF broadcast to set a frame of reference for rviz and publishes odometry. 
    */
    void monoodom::visualize(int n_frame)
    {
        cv::Mat traj = cv::Mat::zeros(cv::Size(1241,376), CV_8UC3);
        

        int visual_limit = 5000;
        if(visual_limit>min_points)
        {
            visual_limit = min_points;
        }

        for(size_t i = 0; i<visual_limit; i++)
        {
            cv::circle(traj, prev_points[i], 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(traj, curr_points[i], 2, cv::Scalar(255, 0, 0), 2);
        }

        std::map<int, cv::Point2f>::iterator mit;

        for(size_t i = 0; i<visual_limit; i++)
        {
            int id = idx[i];
            mit = prev_points_map.find(id);

            if(mit != prev_points_map.end())
            {
                cv::arrowedLine(traj, curr_points[i], mit->second, cv::Scalar(255, 255, 255), 1, 16, 0, 0.1);
            }
        }
   
        // int x = int(prev_T.at<double>(0)) + 300;
        // int y = int(prev_T.at<double>(1)) + 150;
        // int z = int(prev_T.at<double>(2)) + 300;
        // // std::cout<<x<<" "<<y<<" "<<z<<std::endl;
        // cv::circle(odom_output, cv::Point(x,y), 0.25, cv::Scalar(255, 0, 0), 2);
        // cv::rectangle(odom_output, cv::Point(10,30), cv::Point(550, 50), cv::Scalar(0,0,0), CV_FILLED);
        // cv::imshow("Trajectory", odom_output);

        std::cout<<"Before Plotting function"<<std::endl;
        std::cout<<"image_traj "<<image_traj.rows<<" "<<image_traj.cols<<std::endl;
        cv::addWeighted(image_traj, 1.0, traj, 0.6, 0, image_traj);
        std::cout<<"Adding weights"<<std::endl;
        cv::imshow("Feature Tracking", image_traj);

        Rot = prev_R;
        trans = prev_T;
        euler = RotMatToEuler(Rot, true);
        cv::Vec3f p_R = RotMatToEuler(R, false);
        tf::Matrix3x3 _R(Rot.at<double>(0,0), Rot.at<double>(0,1), Rot.at<double>(0,2),
          Rot.at<double>(1,0), Rot.at<double>(1,1), Rot.at<double>(1,2),
          Rot.at<double>(2,0), Rot.at<double>(2,1), Rot.at<double>(2,2));
        _R.getRotation(quat);
        cv::Ptr<cv::Formatter> round = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
        round->set64fPrecision(3);
        round->set32fPrecision(3);
        std::cout<<"FRAME ID: "<<n_frame<<std::endl;
        std::cout<<"Rotation: "<<std::setprecision(3) << "[" << p_R.val[0] << ", " << p_R.val[1] << ", " << p_R.val[2]<< "]" << std::endl;
        std::cout << "Acc. Rotation(euler):  " <<std::setprecision(3) << "[" << euler.val[0]<< ", " << euler.val[1]<< ", " << euler.val[2]<< "]" << std::endl;
        std::cout << "Translate(x,y,z): " <<round->format(trans.t()) << std::endl;

        transform.stamp_ = ros::Time::now();
        transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
        transform.setOrigin(tf::Vector3(trans.at<double>(0), trans.at<double>(1), trans.at<double>(2)));

        tf_broadcaster.sendTransform(transform);

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "/world";
        odom.child_frame_id = "/camera";
        
        odom_quat.x = quat[0];
        odom_quat.y = quat[1];
        odom_quat.z = quat[2];
        odom_quat.w = quat[3];

        // Set the position and rotation.
        odom.pose.pose.position.x = trans.at<double>(0);
        odom.pose.pose.position.y = trans.at<double>(1);
        odom.pose.pose.position.z = trans.at<double>(2);
        odom.pose.pose.orientation = odom_quat;

        // publish to /odom.
        pub_odom.publish(odom);

        cv::waitKey(1);
    }