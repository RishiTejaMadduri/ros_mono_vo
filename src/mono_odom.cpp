#include "mono_odom.h"

bool score_comparator(const cv::KeyPoint& p1, const cv::KeyPoint& p2)
{
    return p1.response > p2.response;
}

monoodom::monoodom():
it_(nh_)
{
    camInfo_ = nh_.subscribe("/camera/rgb/camera_info", 1, &monoodom::getCalib, this);
    
    // if(calibInit)
    imageSub_ = it_.subscribe("/camera/rgb/image_raw", 2, &monoodom::imageCallBack, this, image_transport::TransportHints("compressed"));
    max_frames = INT_MAX;
}

bool monoodom::convertImages(const sensor_msgs::ImageConstPtr& Image1, const sensor_msgs::ImageConstPtr& Image2 = nullptr)
{
    cv_bridge::CvImageConstPtr cvbImg1, cvbImg2;

    try
    {
        cvbImg1 = cv_bridge::toCvShare(Image1);
        cvbImg2 = cv_bridge::toCvShare(Image2);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("CV BRIDGE EXCEPTION: %s ", e.what());
        return false;
    }
    cv::cvtColor(cvbImg1->image, Img1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cvbImg2->image, Img2, cv::COLOR_BGR2GRAY);

    return true;
}
    void monoodom::getCalib(const sensor_msgs::CameraInfoConstPtr& info)
    {
        if(!calibInit)
        {
            focal = info->P[0];
            pp.x = info->P[2];
            pp.y = info->P[5]; 

            R = cv::Mat(3, 3, CV_64FC1, (void *)info->R.elems).clone();
            P = cv::Mat(3, 3, CV_64FC1, (void *)info->P.elems).clone();
            K = cv::Mat(3, 3, CV_64FC1);
            
            for(int i = 0; i<3; i++)
            {
                for(int j = 0; j<3; j++)
                {
                    K.at<double>(i, j) = P.at<double>(i, j);
                }
            }
            std::cout<<"CALIBRATION DONE"<<std::endl;
            calibInit = true;
        }

    }

    void monoodom::kptsBucketing(std::vector<cv::Point2f>& srcKpts, std::vector<cv::KeyPoint>& dstKpts)
    {
        for(int i = 0; i<min_points; i++)
        {
            srcKpts.push_back(dstKpts[i].pt);
        }
    }

    void monoodom::FeatureDetection(cv::Mat& Img1, std::vector<cv::Point2f>& keyPoints)
    {

        std::cout<<"featureDetection"<<std::endl;

        std::vector<cv::KeyPoint> kptsBuckted;
        int fast_threshold = 20;
        bool nms = true;
        FAST(Img1, kptsBuckted, fast_threshold, nms);

        std::sort(kptsBuckted.begin(), kptsBuckted.end(), score_comparator);

        kptsBucketing(keyPoints, kptsBuckted);

    }

    void monoodom::FeatureMatching(cv::Mat& Img1, cv::Mat& Img2, std::vector<cv::Point2f>& keyPoints1, std::vector<cv::Point2f>& keyPoints2, std::vector<uchar>& status)
    {
        std::cout<<"Feature Matching"<<std::endl;
        std::vector<float>err;
        cv::Size winSize = cv::Size(21, 21);

        cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

        cv::calcOpticalFlowPyrLK(Img1, Img2, keyPoints1, keyPoints2, status, err, winSize, 3, termcrit, 0, 0.001);
        std::cout<<"calcOpticalFlow"<<std::endl;
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
        std::cout<<"End of For loop"<<std::endl;
    }


    void monoodom::imageCallBack(const sensor_msgs::ImageConstPtr& Image1)
    {
        std::cout<<"imageCallBack"<<std::endl;
        scale = 0.5;

        double flow = 0.0;

        if(!calibInit)
        {
            ROS_WARN("NO CALIB INFO FOUND");
            return;
        }

        // std::cout<<image_cache.size()<<std::endl;
         image_cache.push(Image1);
         
         if(!odomInit || n_frame<3)
         {
             if(image_cache.size()>2)
             {
                    
                    const sensor_msgs::ImageConstPtr Image1 = image_cache.front();
                    image_cache.pop();
                    const sensor_msgs::ImageConstPtr Image2 = image_cache.front();
                    image_cache.pop();

                    if(!convertImages(Image1, Image2))
                    {
                        ROS_WARN("FAILED IMAGE CONVERSION");
                        return;
                    }

                    std::vector<cv::Point2f> points1, points2;
                    FeatureDetection(Img1, points1);
                    FeatureMatching(Img1, Img2, points1, points2, status);

                    E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                    std::cout<<"FOUND E"<<std::endl;
                    cv::recoverPose(E, points2, points1, curr_R, curr_T, focal, pp, mask);

                    prev_image = Img2;
                    prev_points = points2;

                    prev_R = curr_R.clone();
                    prev_T = curr_T.clone();

                    ROS_INFO(" INITIALIZED \n");
                    odomInit = true;
                    return;
             }
         }

         else
         {
             if(n_frame>max_frames)
                return;
            
            const sensor_msgs::ImageConstPtr Image1 = image_cache.front();
            image_cache.pop();
            convertImages(Image1);

            FeatureMatching(prev_image, Img1, prev_points, curr_points, status);

            E = cv::findEssentialMat(curr_points, prev_points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
            cv::recoverPose(E, curr_points, prev_points, curr_R, curr_T, focal, pp, mask);

            if((scale>0.1) && (curr_T.at<double>(2)>curr_T.at<double>(0) && (curr_T.at<double>(2) > curr_T.at<double>(1) )) )
            {
                prev_T += scale*(prev_R*curr_T);
                prev_R = prev_R * curr_R;
            }

            if(prev_points.size()<min_points)
            {
                FeatureDetection(prev_image, prev_points);
                FeatureMatching(prev_image, curr_image, prev_points, curr_points, status);
            }

            for(int i = 0; i<curr_points.size(); i++)
            {
                idx.push_back(id++);
            }

            for(int i = 0; i<curr_points.size(); i++)
            {
                prev_points_map.insert(std::make_pair(idx[i], curr_points[i]));
            }

            image_taj = curr_image.clone();

            visualize(n_frame);

            prev_image = curr_image.clone();
            prev_points = curr_points;
         }
    }

    void monoodom::visualize(int n_frame)
    {
        std::cout<<"visualize"<<std::endl;
        cv::Mat traj = cv::Mat::zeros(cv::Size(600,600), CV_8UC3);
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

        for(int i = 0; i<visual_limit; i++)
        {
            int id = idx[i];
            mit = prev_points_map.find(id);

            if(mit != prev_points_map.end())
            {
                cv::arrowedLine(traj, curr_points[i], mit->second, cv::Scalar(255, 255, 255), 1, 16, 0, 0.1);
            }
        }
        // cv_ptr = image_taj;
        cv::addWeighted(image_taj, 1.0, image_taj, 0.6, 0, image_taj);
        // imagePub_.publish(cv_ptr->toImageMsg());
        cv::imshow("Result Image", image_taj);
        cv::namedWindow("Result, Image", cv::WINDOW_AUTOSIZE);
        
        cv::waitKey(1);
    }