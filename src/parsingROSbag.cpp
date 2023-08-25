#include "utility.h"

void img_callback(const sensor_msgs::CompressedImageConstPtr &image_msg)
{
	m_buf.lock();
	img_buf.push(image_msg);
	m_buf.unlock();
}

void depth_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
	m_buf.lock();
	depth_buf.push(image_msg);
	m_buf.unlock();
}

void infra1_callback(const sensor_msgs::CompressedImageConstPtr &image_msg)
{
	m_buf.lock();
	infra1_buf.push(image_msg);
	m_buf.unlock();
}

void infra2_callback(const sensor_msgs::CompressedImageConstPtr &image_msg)
{
	m_buf.lock();
	infra2_buf.push(image_msg);
	m_buf.unlock();
}

void infra1_info_callback(const sensor_msgs::CameraInfoConstPtr &imageinfo_msg)
{
	m_buf.lock();
	infra1_info_buf.push(imageinfo_msg);
	m_buf.unlock();
}

void infra2_info_callback(const sensor_msgs::CameraInfoConstPtr &imageinfo_msg)
{
	m_buf.lock();
	infra2_info_buf.push(imageinfo_msg);
	m_buf.unlock();
}

void sync_process()
{
    while(1)
	{
		sequence++;

		cv::Mat img, depth, infra1, infra2;
		std_msgs::Header header;

		m_buf.lock();
		if(!depth_buf.empty() && !infra1_buf.empty() && !infra2_buf.empty())
		{
			double depth_time  = depth_buf.front() -> header.stamp.toSec();
			double infra1_time = infra1_buf.front() -> header.stamp.toSec();
			double infra2_time = infra2_buf.front() -> header.stamp.toSec();

			if(depth_time < infra1_time - 0.003 && infra1_time < infra2_time - 0.003)
			{
				depth_buf.pop();
				infra1_buf.pop();
				printf("[Case1] throw depth_img and throw infra1_img\n");
			}
			else if(depth_time > infra1_time + 0.003 && infra2_time > depth_time + 0.003)
			{
				infra1_buf.pop();
				depth_buf.pop();
				printf("[Case2] throw infra1_img and throw depth_img\n");
			}
			else if(infra1_time < infra2_time - 0.003 && infra2_time < depth_time - 0.003)
			{
				infra1_buf.pop();
				infra2_buf.pop();
				printf("[Case3] throw infra1_img and throw infra2_img\n");
			}
			else if(infra1_time > infra2_time + 0.003 && depth_time > infra1_time + 0.003)
			{
				infra2_buf.pop();
				infra1_buf.pop();
				printf("[Case4] throw infra2_img and throw infra1_img\n");
			}
			else if(infra2_time < depth_time - 0.003 && depth_time < infra1_time - 0.003)
			{
				infra2_buf.pop();
				depth_buf.pop();
				printf("[Case5] throw infra2_img and throw depth_img\n");
			}
			else if(infra2_time > depth_time + 0.003 && infra1_time > infra2_time + 0.003)
			{
				depth_buf.pop();
				infra2_buf.pop();
				printf("[Case6] throw depth_img and throw infra2_img\n");
			}
			else
			{
				header = depth_buf.front()->header;
                header.frame_id = "world";
				depth = sensorMsgDepth2cvMat(depth_buf.front());
				depth_buf.pop();
				infra1 = sensorMsg2cvMat(infra1_buf.front());
				infra1_buf.pop();
				infra2 = sensorMsg2cvMat(infra2_buf.front());
				infra2_buf.pop();

                while((infra1_info_buf.front()->header.stamp.toSec()) < header.stamp.toSec())
                    infra1_info_buf.pop();
                while((infra2_info_buf.front()->header.stamp.toSec()) < header.stamp.toSec())
                    infra2_info_buf.pop();

                sensor_msgs::CameraInfo infra1_info = *infra1_info_buf.front();
                infra1_info.header = header;
                sensor_msgs::CameraInfo infra2_info = *infra2_info_buf.front();
                infra2_info.header = header;

                // Publish Synced Images
                if(PUBLISH_SYNCED_IMGS)
                    pubSyncImgs(header, depth, infra1, infra2, infra1_info, infra2_info);
                
                infra1_info_buf.pop();
                infra2_info_buf.pop();

                // Save Synced Images
                // if(SAVE_IMG == 0)


            }
		}
		m_buf.unlock();

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parsing_and_sync_ROSBag");
    ros::NodeHandle nh;

    pub_img = nh.advertise<sensor_msgs::Image>("sync_color", 1000);
    pub_depth = nh.advertise<sensor_msgs::Image>("sync_depth", 1000);
    pub_infra1 = nh.advertise<sensor_msgs::Image>("sync_infra1", 1000);
    pub_infra2 = nh.advertise<sensor_msgs::Image>("sync_infra2", 1000);
    pub_infra1_info = nh.advertise<sensor_msgs::CameraInfo>("sync_infra1_info", 1000);
    pub_infra2_info = nh.advertise<sensor_msgs::CameraInfo>("sync_infra2_info", 1000);

    ros::Subscriber sub_img =  nh.subscribe("/camera/camera/color/image_raw/compressed", 1000, img_callback);
    ros::Subscriber sub_depth =  nh.subscribe("/camera/camera/depth/image_rect_raw", 1000, depth_callback);
    ros::Subscriber sub_infra1 =  nh.subscribe("/camera/camera/infra1/image_rect_raw/compressed", 1000, infra1_callback);
    ros::Subscriber sub_infra2 =  nh.subscribe("/camera/camera/infra2/image_rect_raw/compressed", 1000, infra2_callback);

    ros::Subscriber sub_infra1_info =  nh.subscribe("/camera/camera/infra1/camera_info", 1000, infra1_info_callback);
    ros::Subscriber sub_infra2_info =  nh.subscribe("/camera/camera/infra2/camera_info", 1000, infra2_info_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}