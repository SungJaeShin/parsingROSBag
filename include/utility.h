#include "include.h"

// Reference Site: https://github.com/ros-perception/image_transport_plugins/blob/noetic-devel/compressed_image_transport/src/compressed_subscriber.cpp
cv::Mat sensorMsg2cvMat(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8);
    cv::Mat img = ptr->image.clone();
    cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

    return img;
}

cv::Mat sensorMsgDepth2cvMat(const sensor_msgs::ImageConstPtr &img_msg)
{

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "16UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono16";

        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }
    else   
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO16);


    cv::Mat img = ptr->image.clone();

    return img;
}

cv::Mat sensorMsg2cvMat(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
		ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGRA8);
    }
    else
		ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8);

    cv::Mat img = ptr->image.clone();
    cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

    return img;
}

sensor_msgs::Image cvMat2sensorMsg(cv::Mat image, std_msgs::Header header)
{
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img;

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
	img_bridge.toImageMsg(img);

	return img;
}

sensor_msgs::Image cvMatDepth2sensorMsg(cv::Mat image, std_msgs::Header header)
{
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img;

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, image);
	img_bridge.toImageMsg(img);

	return img;
}

void pubSyncImgs(std_msgs::Header header, cv::Mat color, cv::Mat depth, cv::Mat infra1)
{
    pub_img.publish(cvMat2sensorMsg(color, header));
    pub_depth.publish(cvMatDepth2sensorMsg(depth, header));
    pub_infra1.publish(cvMat2sensorMsg(infra1, header));
}

void saveSyncImgs(int count, cv::Mat color, cv::Mat depth, cv::Mat infra1)
{
    std::string save_path;
    if(count >= 0 && count < 10)
        save_path = "~/Dataset/kaist/images/0000" + std::to_string(count) + ".jpg";
    else if(count >= 10 && count < 100)
        save_path = "~/Dataset/kaist/images/000" + std::to_string(count) + ".jpg";
    else if(count >= 100 && count < 1000)
        save_path = "~/Dataset/kaist/images/00" + std::to_string(count) + ".jpg";
    else if(count >= 1000 && count < 10000)
        save_path = "~/Dataset/kaist/images/0" + std::to_string(count) + ".jpg";

    cv::imwrite(save_path, color);
}
