#ifndef __MONOODOM_H__
#define __MONOODOM_H__

#include<iostream>
#include<queue>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include<fstream>
#include <Eigen/Geometry>
class monoodom
{
    public:
        monoodom();
        bool convertImages(const sensor_msgs::ImageConstPtr& Image1, cv::Mat& Img1, cv::Mat& Img2, const sensor_msgs::ImageConstPtr& Image2);
        void getCalib(const sensor_msgs::CameraInfoConstPtr& info);
        void kptsBucketing(std::vector<cv::Point2f>& srcKpts, std::vector<cv::KeyPoint>& dstKpts);
        void FeatureMatching(cv::Mat Img1, cv::Mat Img2, std::vector<cv::Point2f>& keyPoints1, std::vector<cv::Point2f>& keyPoints2);
        void FeatureDetection(cv::Mat Img1, std::vector<cv::Point2f>& keyPoints);
        void imageCallBack(const sensor_msgs::ImageConstPtr& Image1);
        void visualize(int n_frame);
        // void posecallback(const nav_msgs::Odometry::ConstPtr &msg);
        double compute_scale(cv::Mat prev_T, cv::Mat curr_T);
        // bool score_comparator(const cv::KeyPoint& p1, const cv::KeyPoint& p2);
        cv::Vec3f RotMatToEuler(cv::Mat &R);
        bool IsRotMat(cv::Mat &R); 
        std::string AddZeroPadding(const int value, const unsigned presision);

        cv::Mat GetRotation()
        {
            return prev_R;
        }
        int getFrames()
        {
            return n_frame;
        }
        cv::Mat GetTranslation()
        {
            return prev_T;
        }
    

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber imageSub_;
        image_transport::Publisher imagePub_;
        ros::Publisher pub_odom;
        ros::Subscriber camInfo_;
        tf::TransformBroadcaster tf_broadcaster;
        tf::StampedTransform transform;
        // tf::Matrix3x3 _R;
        tf::Quaternion quat;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion odom_quat;
        // ros::Subscriber pose_sub;

        std::vector<cv::Point2f> points1, points2;

        bool odomInit;
        bool calibInit;
        
        std::queue<sensor_msgs::ImageConstPtr>image_cache;
        
        int max_features;
        int min_points;
        int max_frames;
        int n_frame;
        int count;
        int flow;
        int prev_count;
        unsigned int id;

        double scale, focal;
        cv::Point2f pp;
        cv::Mat image_traj;
        cv::Mat K, P, T;
        cv::Mat prev_image, curr_image;
        cv::Mat init_pose;
        cv::Mat prev_R, prev_T, curr_R;
        cv::Mat R, curr_T;
        cv::Mat Rot, trans;
        cv::Vec3f euler;
        cv::Mat E;
        cv::Mat D;
        cv::Mat mask;
        cv::Mat odom_output;
        // cv::Mat Img1, Img2;
        cv_bridge::CvImagePtr cv_ptr;
        std::map<int, cv::Point2f>prev_points_map;
        std::vector<cv::Point2f>prev_points;
        std::vector<cv::Point2f>curr_points;
        std::vector<uchar> status;
        std::vector<int>idx;
};

#endif
