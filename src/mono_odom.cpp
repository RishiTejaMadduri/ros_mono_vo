#include "mono_odom.h"

/*
The main consturctor for initializing KITTI paths/ROS topics, n_frames, output windows
*/
monoodom::monoodom():
it_(nh_)
{
    camInfo_ = nh_.subscribe("/camera/rgb/camera_info", 1, &monoodom::getCalib, this);
    imageSub_ = it_.subscribe("/camera/rgb/image_raw", 1, &monoodom::imageCallBack, this);
    imagePub_ = it_.advertise("/mono_vo/output", 1);
    pub_odom = nh_.advertise<nav_msgs::Odometry>("odom", 1);

    transform.frame_id_ = "/world";
    transform.child_frame_id_ = "/camera";

    max_frames = INT_MAX;
    min_points = 50;
    n_frame = 0;
    odom_output = cv::Mat::zeros(600, 600, CV_8UC3);
}

/*
    Function that converts ROS Sensor messages to OpenCV images and converts them to grayscale. - NOT BEING USED
    @param Image1 - ROS Sensor message corresponding to first image .
    @param Img1 - CV Mat that stores the OpenCV image of Image1 converted to grayscale.
    @param Img2 - CV Mat that stores the OpenCV image of Image2 converted to grayscale - Is usually a dummy matrix/empty matrix if not the first iteration.
    @param Image1 - ROS Sensor message corresponding to second image - Is usually a nullpointer if not the first iteration/initialization. 
*/
// bool monoodom::convertImages(const sensor_msgs::ImageConstPtr& Image1, cv::Mat& Img1, cv::Mat& Img2, const sensor_msgs::ImageConstPtr& Image2 = nullptr)
// {
//     cv_bridge::CvImageConstPtr cvbImg1, cvbImg2;
//     if(!Image2)
//     {
//         try
//         {
//             cvbImg1 = cv_bridge::toCvShare(Image1);
//             c_image = cvbImg1->image;
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
//             cvbImg1 = cv_bridge::toCvShare(Image1);
//             cvbImg2 = cv_bridge::toCvShare(Image2);
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
    A callback function for the Camera info subscriber 
    focal, pp and K are initialized here and the Calibration init flag is marked True
    @param info - ROS topic info from CameraInfo topic
*/
void monoodom::getCalib(const sensor_msgs::CameraInfoConstPtr& info)
{
    if(!calibInit)
    {


        // R = cv::Mat(3, 3, CV_64FC1, (void *)info->R.elems).clone();
        K = cv::Mat(3, 3, CV_64FC1, (void *)info->K.elems).clone();
        D = cv::Mat::zeros(1, 5, CV_64FC1);
        focal = info->K[0];
        pp.x = info->K[2];
        pp.y = info->K[5]; 

        //std::cout<<"CALIBRATION DONE"<<std::endl;
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
    for(int i = 0; i<50; i++)
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
cv::Vec3f monoodom::RotMatToEuler(cv::Mat &R) 
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
    return cv::Vec3f(x, y, z);
}
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

    @param Image1 - Images from ROS Image topic
*/
void monoodom::imageCallBack(const sensor_msgs::ImageConstPtr& Image1)
{
    if(!calibInit)
    {
        ROS_ERROR("NO CALIB INFO FOUND");
        return;
    }
    //Maintaining an image cache to ensure a minimum of 10 image gap between the two images for initialization
    image_cache.push(Image1);
        if(!odomInit && n_frame<2)
        {
            scale = 1.0;
            if(image_cache.size()>10)
            {
                const sensor_msgs::ImageConstPtr image1 = image_cache.front();
                image_cache.pop();
                while(image_cache.size()>2)
                    image_cache.pop();

                const sensor_msgs::ImageConstPtr image2 = image_cache.front();
                image_cache.pop();

                cv::Mat Img1, Img2;
                cv_bridge::CvImagePtr cv_imgptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
                cv_bridge::CvImagePtr cv_imgptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::BGR8);

                cv::cvtColor(cv_imgptr1->image, Img1, cv::COLOR_BGR2GRAY);
                cv::cvtColor(cv_imgptr2->image, Img2, cv::COLOR_BGR2GRAY);
                
                // if(!convertImages(Image1, Img1, Img2, Image2))
                // {
                //     ROS_WARN("FAILED IMAGE CONVERSION");
                //     return;
                // }

                //Feature detection, Matching and Pose estimation
                FeatureDetection(Img1, points1);
                FeatureMatching(Img1, Img2, points1, points2);
                E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                cv::recoverPose(E, points2, points1, R, curr_T, focal, pp, mask);

                prev_image = Img2;
                prev_points = points2;

                prev_R = R.clone();
                prev_T = curr_T.clone();

                //Odometry initialization
                ROS_INFO(" INITIALIZED \n");

                odomInit = true;
                n_frame = 2;

                //Clearing the cache
                while(image_cache.size())
                    image_cache.pop();
            }
        }

        if(odomInit && n_frame>=2)
        {

            if(n_frame>max_frames)
                return;
            
            if(image_cache.size()>5)
            {
                //Maintaining a gap of 5 images - Arbitrarily chosen. 
                while(image_cache.size()>2)
                    image_cache.pop();

                //Image processing
                const sensor_msgs::ImageConstPtr c_Img1 = image_cache.front();

                cv_bridge::CvImagePtr curr_imgptr1 = cv_bridge::toCvCopy(c_Img1, sensor_msgs::image_encodings::BGR8);
                cv::Mat c_image = curr_imgptr1->image;
                // cv::Mat dummy;
                // convertImages(Image1, curr_image, dummy);
                cv::cvtColor(curr_imgptr1->image, curr_image, cv::COLOR_BGR2GRAY);

                //Feature matching and Pose recovery
                FeatureMatching(prev_image, curr_image, prev_points, curr_points);
                E = cv::findEssentialMat(curr_points, prev_points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                cv::recoverPose(E, curr_points, prev_points, R, curr_T, focal, pp, mask);

                scale = 0.5;

                //Update conditions
                if((scale>0.1)&&(curr_T.at<double>(2) > curr_T.at<double>(0)) && (curr_T.at<double>(2) > curr_T.at<double>(1)))
                {
                    prev_T += scale*(prev_R*curr_T);
                    prev_R = R * prev_R;     
                }

                if(prev_points.size()<min_points)
                {
                    FeatureDetection(prev_image, prev_points);
                    FeatureMatching(prev_image, curr_image, prev_points, curr_points);
                }

                for(int i = 0; i<curr_points.size(); i++)
                {
                    idx.push_back(id++);
                }

                for(int i = 0; i<curr_points.size(); i++)
                {
                    prev_points_map.insert(std::make_pair(idx[i], curr_points[i]));
                }
                image_traj = c_image.clone();

                visualize(n_frame);

                prev_image = curr_image.clone();
                prev_points = curr_points;

                n_frame++;

                while(image_cache.size())
                    image_cache.pop();

            }

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
    //CV Mat for displaying image of the most recent frame
    cv::Mat traj = cv::Mat::zeros(cv::Size(640,480), CV_8UC3);

    int visual_limit = 5000;
    if(visual_limit>min_points)
    {
        visual_limit = min_points;
    }

    //Plotting current and previous feature points in Red and blue colors
    for(size_t i = 0; i<visual_limit; i++)
    {
        cv::circle(traj, prev_points[i], 2, cv::Scalar(0, 0, 255), 2);
        cv::circle(traj, curr_points[i], 2, cv::Scalar(255, 0, 0), 2);
    }

    std::map<int, cv::Point2f>::iterator mit;

    //Tracking features through images using arrow lines
    for(size_t i = 0; i<visual_limit; i++)
    {
        int id = idx[i];
        mit = prev_points_map.find(id);

        if(mit != prev_points_map.end())
        {
            cv::arrowedLine(traj, curr_points[i], mit->second, cv::Scalar(255, 255, 255), 1, 16, 0, 0.1);
        }
    }

    int x = int(prev_T.at<double>(0)) + 300;
    // int y = int(prev_T.at<double>(1)) + 150;
    int z = int(prev_T.at<double>(2)) + 150;

    //Plotting translation points for estimating trajectory
    cv::circle(odom_output, cv::Point(x,z), 0.25, cv::Scalar(255, 0, 0), 2);
    cv::rectangle(odom_output, cv::Point(10,30), cv::Point(550, 50), cv::Scalar(0,0,0), CV_FILLED);
    cv::imshow("Trajectory", odom_output);

    //Displaying current frame with feature points and arrows
    cv::addWeighted(image_traj, 1.0, traj, 0.6, 0, image_traj);
    cv::imshow("Feature Tracking", image_traj);

    //Odom publishing in RVIZ
    Rot = prev_R;
    trans = prev_T;
    euler = RotMatToEuler(Rot);

    tf::Matrix3x3 _R(Rot.at<double>(0,0), Rot.at<double>(0,1), Rot.at<double>(0,2),
        Rot.at<double>(1,0), Rot.at<double>(1,1), Rot.at<double>(1,2),
        Rot.at<double>(2,0), Rot.at<double>(2,1), Rot.at<double>(2,2));
    _R.getRotation(quat);
    cv::Ptr<cv::Formatter> round = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    round->set64fPrecision(3);
    round->set32fPrecision(3);
    std::cout << "Rotation(euler):  " <<std::setprecision(3) << "[" << euler.val[0] << ", " << euler.val[1] << ", " << euler.val[2] << "]" << std::endl;
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