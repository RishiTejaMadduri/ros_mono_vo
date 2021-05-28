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
        //Image processing funciton - Not being used
        bool convertImages(const sensor_msgs::ImageConstPtr& Image1, cv::Mat& Img1, cv::Mat& Img2, const sensor_msgs::ImageConstPtr& Image2);
        
        //Calibration callback
        void getCalib(const sensor_msgs::CameraInfoConstPtr& info);

        //Feature Matching function for fast features
        void FeatureMatching(cv::Mat Img1, cv::Mat Img2, std::vector<cv::Point2f>& keyPoints1, std::vector<cv::Point2f>& keyPoints2);

        //Detection of FAST features
        void FeatureDetection(cv::Mat Img1, std::vector<cv::Point2f>& keyPoints);

        //The main ImageCallBack function that runs the VO Pipeline
        void imageCallBack(const sensor_msgs::ImageConstPtr& Image1);

        //Visualizer 
        void visualize(int n_frame);

        //Convert Rotation matrix to Euler Angles
        cv::Vec3f RotMatToEuler(cv::Mat &R);
        bool IsRotMat(cv::Mat &R); 

    private:

        //ROS Variables and handles
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber imageSub_;
        image_transport::Publisher imagePub_;
        ros::Publisher pub_odom;
        ros::Subscriber camInfo_;
        tf::TransformBroadcaster tf_broadcaster;
        tf::StampedTransform transform;
        tf::Quaternion quat;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion odom_quat;

        //Variables for features and n_frames and flags
        bool odomInit;
        bool calibInit;    
        int max_features;
        int min_points;
        int max_frames;
        int n_frame;
        int count;
        int flow;
        int prev_count;
        double scale, focal;
        unsigned int id;

        //Data structures for storing points, images
        std::map<int, cv::Point2f>prev_points_map;
        std::vector<cv::Point2f>prev_points;
        std::vector<cv::Point2f>curr_points;
        std::vector<uchar> status;
        std::vector<int>idx;
        std::queue<sensor_msgs::ImageConstPtr>image_cache;
        std::vector<cv::Point2f> points1, points2;

        //CV variables
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
        
};

#endif
