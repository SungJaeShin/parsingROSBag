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
