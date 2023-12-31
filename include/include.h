#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
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

// SAVE_IMG 0; png & 1; jpg 
#define SAVE_IMG 1
// Publish Synced Images
#define PUBLISH_SYNCED_IMGS true

int sequence = 0;
int cnt = 1;
double latest_time = 0;

std::queue<sensor_msgs::CompressedImageConstPtr> img_buf;
std::queue<sensor_msgs::ImageConstPtr> depth_buf;
std::queue<sensor_msgs::CompressedImageConstPtr> infra1_buf;
std::queue<sensor_msgs::CompressedImageConstPtr> infra2_buf;
std::queue<sensor_msgs::CameraInfoConstPtr> infra1_info_buf;
std::queue<sensor_msgs::CameraInfoConstPtr> infra2_info_buf;

ros::Publisher pub_img;
ros::Publisher pub_depth;
ros::Publisher pub_infra1;
ros::Publisher pub_infra2;
ros::Publisher pub_infra1_info;
ros::Publisher pub_infra2_info;

std::mutex m_buf;