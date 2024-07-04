#ifndef INCLUDE
#define INCLUDE

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <queue>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/CameraInfo.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// OPENCV ERROR 
#include "opencv2/xfeatures2d/nonfree.hpp"

int sequence = 0;
int cnt = 1;
double latest_time = 0;

// ROS topics
std::string COLOR_TOPIC, DEPTH_TOPIC, INFRA1_TOPIC, INFRA2_TOPIC;
std::string COLOR_INFO, INFRA1_INFO, INFRA2_INFO;

// GT pose path
std::string GT_PATH;

// SAVE_IMG -> 0; not save & 1; save (.jpg) 
int SAVE_IMG;

// To set time difference
double DIFF_THRESHOLD;

// Publish Synced Images
int PUBLISH_SYNCED_IMGS_CONFIG;
bool PUBLISH_SYNCED_IMGS;

// Sync GT time with ROSBAG img
int SYNC_GT_TIME_CONFIG;
bool SYNC_GT_TIME;

// Setting Closing time btw cur Img and GT Img
double SIM_THRESHOLD;

std::queue<sensor_msgs::ImageConstPtr> img_buf;
std::queue<sensor_msgs::ImageConstPtr> depth_buf;
std::queue<sensor_msgs::CompressedImageConstPtr> infra1_buf;
std::queue<sensor_msgs::CompressedImageConstPtr> infra2_buf;
std::queue<sensor_msgs::CameraInfoConstPtr> color_info_buf;
std::queue<sensor_msgs::CameraInfoConstPtr> infra1_info_buf;
std::queue<sensor_msgs::CameraInfoConstPtr> infra2_info_buf;

ros::Publisher pub_img;
ros::Publisher pub_depth;
ros::Publisher pub_infra1;
ros::Publisher pub_infra2;
ros::Publisher pub_infra1_info;
ros::Publisher pub_infra2_info;

std::mutex m_buf;

struct Pose
{
    int idx;
    double time;
    Eigen::Vector3d translation;
    Eigen::Quaterniond quaternion;
};

// Setting default precision for std::cout globally
struct CoutSettings {
    CoutSettings() {
        std::cout << std::fixed << std::setprecision(5);
    }
};
static CoutSettings coutSettings;

#endif