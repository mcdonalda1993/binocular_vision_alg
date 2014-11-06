#include <ros/ros.h>
#include <RHcam/RHcam.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

// Parameter Server
static const char RESIZEPARAM[] = "/RH/resize_imgs"; // 1: resize

// Messages
static const char CAM_SUB_LEFT[] = "/image_left";
static const char CAM_SUB_RIGHT[] = "/image_right";
static const char CAM_PUB_LEFT_RESIZE[] = "/RH/left_camera/image";
static const char CAM_PUB_RIGHT_RESIZE[] = "/RH/right_camera/image";

float scale_factor;
image_transport::Publisher image_pub_left, image_pub_right;

Mat resizeImg(Mat rgb)
{
	// Resize image
	Mat rgb_scaled;
	if(scale_factor >= 0.0 && scale_factor != 1.0)
		resize(rgb, rgb_scaled, Size(rgb.cols/scale_factor, rgb.rows/scale_factor),
		scale_factor, scale_factor, cv::INTER_LINEAR);
	else
		rgb_scaled = rgb;
	
	ROS_INFO_STREAM("Image size rows: " << rgb_scaled.rows << " Cols: " << rgb_scaled.cols);
	return rgb_scaled;
}

void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
{
	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
        return;
    }
    
    ROS_INFO("Left camera time: %f", msg->header.stamp.toSec());
    
    cv_bridge::CvImage cvi;
    cvi.header.stamp = msg->header.stamp;
    cvi.header.frame_id = "camera_left_resized";
    cvi.encoding = "rgb8";
    cvi.image = resizeImg(cv_ptr->image);
    
    //imwrite("resizedImg1.png", cvi.image);
    
    image_pub_left.publish(cvi.toImageMsg());
    
}

void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
        return;
    }
    
    ROS_INFO("Right camera time: %f", msg->header.stamp.toSec());
    
    cv_bridge::CvImage cvi;
    cvi.header.stamp = msg->header.stamp;
    cvi.header.frame_id = "camera_right_resized";
    cvi.encoding = "rgb8";
    cvi.image = resizeImg(cv_ptr->image);
    
    //imwrite("resizedImg2.png", cvi.image);
    
    image_pub_right.publish(cvi.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RHcam_test");
    ros::NodeHandle nh;
    
    double param_resize;
    if(nh.hasParam(RESIZEPARAM))
    {
    	nh.getParam(RESIZEPARAM, param_resize);
    }
    else
    {
    	ROS_ERROR("RHcam_node exception: \"%s\" parameter is not set in the server", RESIZEPARAM);
    }
    
    scale_factor = (float)param_resize;
    if(scale_factor <= 0)
    {
    	ROS_ERROR("RHcam_node exception: \"%s\" parameter should be greater than zero", RESIZEPARAM);
    }
    
    image_transport::ImageTransport it1(nh), it2(nh);
    
    image_pub_left = it1.advertise(CAM_PUB_LEFT_RESIZE, 1);
    image_pub_right = it2.advertise(CAM_PUB_RIGHT_RESIZE, 1);
    
    image_transport::Subscriber sub1 = it1.subscribe(CAM_SUB_LEFT, 1, imageCallbackLeft);
    image_transport::Subscriber sub2 = it2.subscribe(CAM_SUB_RIGHT, 1, imageCallbackRight);
    
    ros::spin();
}
