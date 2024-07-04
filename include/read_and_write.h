#include "include.h"

std::vector<Pose> loadGTfile()
{
	std::ifstream gtFile(gt_path.c_str());
	if(!gtFile.is_open())
    {
        std::cerr << "Cannot open GT file !!" << std::endl;        
        return {};
    }

    int cur_cnt = 1;
    std::string line;
    std::vector<Pose> gt_pose;
    double t, x, y, z, qx, qy, qz, qw;

    while(std::getline(gtFile, line))
    {
        std::istringstream iss(line);
        if (!(iss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            std::cerr << "Description line: " << cur_cnt << std::endl;
            cur_cnt++;
            continue; // Skip this line and continue with the next one
        }

        Pose cur_pose;
        cur_pose.idx = cur_cnt;
        cur_pose.time = t;
        cur_pose.translation = Eigen::Vector3d(x, y, z);
        cur_pose.quaternion = Eigen::Quaterniond(qx, qy, qz, qw);
                
        gt_pose.push_back(cur_pose);
        cur_cnt++;
    }

    return gt_pose;
}

void saveSyncImgs(int count, cv::Mat color, cv::Mat depth, cv::Mat infra1)
{
    std::string save_path;
    if(count >= 0 && count < 10)
        save_path = "/workspace/dataset/colmap/input/0000" + std::to_string(count) + ".jpg";
    else if(count >= 10 && count < 100)
        save_path = "/workspace/dataset/colmap/input/000" + std::to_string(count) + ".jpg";
    else if(count >= 100 && count < 1000)
        save_path = "/workspace/dataset/colmap/input/00" + std::to_string(count) + ".jpg";
    else if(count >= 1000 && count < 10000)
        save_path = "/workspace/dataset/colmap/input/0" + std::to_string(count) + ".jpg";

    cv::imwrite(save_path, color);
}

void saveSyncImgGT(std::vector<Pose> gt_pose, double cur_time, cv::Mat color, double diff_time)
{
    // auto it = std::find_if(gt_pose.begin(), gt_pose.end(), [cur_time](Pose pose){
    //     return pose.time == cur_time;
    // });

    auto it =  std::find_if(gt_pose.begin(), gt_pose.end(), [cur_time, diff_time](Pose pose){
        return std::fabs(pose.time - cur_time) <= diff_time;
    });

    int count = it->idx;
    double time = it->time;
    std::string save_path = "/workspace/dataset/colmap/input/gt_" + std::to_string(count) + "_" + std::to_string(time) + ".jpg";
    cv::imwrite(save_path, color);
}

