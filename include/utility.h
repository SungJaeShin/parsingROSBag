#include "include.h"

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
