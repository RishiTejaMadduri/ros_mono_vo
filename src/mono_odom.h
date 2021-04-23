#ifndef __MONOODOM_H__
#define __MONOODOM_H__

#include<iostream>
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
    void monoodom();
    bool convertImages(const sensor_msgs::ImageConstPtr& Img, const sensor_msgs::ImageConstPtr& Img2 = nullptr, cv::Mat& cvImg1, cv::Mat& cvImg2);
    void getCalib(const sensor_msgs::CameraInfoConstptr& info);
    void kptsBucketing(std::vector<cv::KeyPoint>& srcKpts, std::vector<cv::KeyPoint>& dstKpts, int width, int height);
    void SelectKeyPoints(cv::Mat& Img1, std::vector<cv::KeyPoint>& keyPoints);
    void FeatureMatching(cv::Mat& Img1, cv::Mat& Img2, std::vector<cv::KeyPoint>& keyPoints1, std::vector<cv::KeyPoint>& keyPoints2, std::vector<uchar>& status);
    void featureDetection(cv::Mat& Img1, std::vector<cv::KeyPoint>& keyPoints);
    void imageCallBack(const sensor_msgs::ImageConstPtr& Img1, const sensor_msgs::ImageConstPtr& Img2 = nullptr, int n_frame);
    void visualize(int n_frame);
    

    private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter imageSub_;
    image_transport::Publisher imagePub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camInfo_;
    

    bool odomInit;
    bool calibInit;
    
    int max_features;
    int min_points;
    int max_frames;
    int min_points;
    int idx;

    double scale, focal;
    cv::Point2f pp;
    cv::Mat image_taj;
    cv::Mat K, P, R;
    cv::Mat prev_image;
    cv::Mat curr_image;
    cv::Mat prev_R, prev_T, curr_R, curr_T;
    cv::Mat E;
    cv::Mat mask;
    
    cv_bridge::CvImagePtr cv_ptr;

    std::map<int, cv::KeyPoint>prev_points_map;
    std::vector<cv::KeyPoint>prev_points;
    std::vector<cv::KeyPoint>curr_points;
};

#endif
