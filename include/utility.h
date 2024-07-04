#include "include.h"

// Ref site: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/estimator/parameters.cpp
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // Get ROS Topics
    fsSettings["color_topic"] >> COLOR_TOPIC;
    fsSettings["depth_topic"] >> DEPTH_TOPIC;
    fsSettings["infra1_topic"] >> INFRA1_TOPIC;
    fsSettings["infra2_topic"] >> INFRA2_TOPIC;

    fsSettings["color_info_topic"] >> COLOR_INFO;
    fsSettings["infra1_info_topic"] >> INFRA1_INFO;
    fsSettings["infra2_info_topic"] >> INFRA2_INFO;

    // SAVE_IMG -> 0; not save & 1; save (.jpg) 
    SAVE_IMG = fsSettings["save_img"];

    // Publish Synced Images
    PUBLISH_SYNCED_IMGS_CONFIG = fsSettings["publish_synced_imgs"];
    if(PUBLISH_SYNCED_IMGS_CONFIG == 0)
        PUBLISH_SYNCED_IMGS = false;
    else if(PUBLISH_SYNCED_IMGS_CONFIG == 1)
        PUBLISH_SYNCED_IMGS = true;

    // Sync GT time with ROSBAG img
    SYNC_GT_TIME_CONFIG = fsSettings["sync_gt_time"];
    if(SYNC_GT_TIME_CONFIG == 0)
        SYNC_GT_TIME = false;
    else if(SYNC_GT_TIME_CONFIG == 1)
        SYNC_GT_TIME = true;

    // To set time difference
    DIFF_THRESHOLD = fsSettings["diff_threshold"];
    // Setting Closing time btw cur Img and GT Img
    SIM_THRESHOLD = fsSettings["sim_threshold"];

    // GT Absolute Path
    fsSettings["gt_path"] >> GT_PATH;

    fsSettings.release();
}

bool isSame(std::vector<Pose> gt_pose, double cur_time)
{
    auto it = std::find_if(gt_pose.begin(), gt_pose.end(), [cur_time](Pose pose){
        return pose.time == cur_time;
    });
    
    return it != gt_pose.end();
}

bool isClose(std::vector<Pose> gt_pose, double cur_time, double diff_time)
{
    return std::any_of(gt_pose.begin(), gt_pose.end(), [cur_time, diff_time](Pose pose){
        return std::fabs(pose.time - cur_time) <= diff_time;
    });
}

void pubSyncImgs(std_msgs::Header header, cv::Mat color, cv::Mat depth)
{
    pub_img.publish(cvMat2sensorMsg(color, header));
    pub_depth.publish(cvMatDepth2sensorMsg(depth, header));
}
