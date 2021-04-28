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

class monoodom
{
    public:
        monoodom();
        bool convertImages(const sensor_msgs::ImageConstPtr& Img, const sensor_msgs::ImageConstPtr& Img2);
        void getCalib(const sensor_msgs::CameraInfoConstPtr& info);
        void kptsBucketing(std::vector<cv::Point2f>& srcKpts, std::vector<cv::KeyPoint>& dstKpts);
        void FeatureMatching(cv::Mat& Img1, cv::Mat& Img2, std::vector<cv::Point2f>& keyPoints1, std::vector<cv::Point2f>& keyPoints2, std::vector<uchar>& status);
        void FeatureDetection(cv::Mat& Img1, std::vector<cv::Point2f>& keyPoints);
        void imageCallBack(const sensor_msgs::ImageConstPtr& Img1);
        void visualize(int n_frame);
    

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber imageSub_;
        image_transport::Publisher imagePub_;
        ros::Subscriber camInfo_;
        

        bool odomInit;
        bool calibInit;
        
        std::queue<sensor_msgs::ImageConstPtr>image_cache;
        int max_features;
        int min_points;
        int max_frames;
        int n_frame;
        unsigned int id;

        double scale, focal;
        cv::Point2f pp;
        cv::Mat image_taj;
        cv::Mat K, P, R;
        cv::Mat prev_image;
        cv::Mat curr_image;
        cv::Mat prev_R, prev_T, curr_R, curr_T;
        cv::Mat E;
        cv::Mat mask;
        cv::Mat Img1, Img2;
        cv_bridge::CvImagePtr cv_ptr;

        std::map<int, cv::Point2f>prev_points_map;
        std::vector<cv::Point2f>prev_points;
        std::vector<cv::Point2f>curr_points;
        std::vector<uchar> status;
        std::vector<int>idx;
};

#endif
