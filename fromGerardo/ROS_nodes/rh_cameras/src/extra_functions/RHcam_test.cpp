#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_LEFT[] = "RH camera left";
static const char WINDOW_RIGHT[] = "RH camera right";

// Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";


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
    
    cv::imshow(WINDOW_LEFT, cv_ptr->image);
    //cv::resizeWindow(WINDOW_LEFT, 640, 480);
    //cv::waitKey(3);
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
    
    cv::imshow(WINDOW_RIGHT, cv_ptr->image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RHcam_test");
    ros::NodeHandle nh;
    
    cv::namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
    //cv::resizeWindow(WINDOW_LEFT, 640, 480);
    cv::namedWindow(WINDOW_RIGHT, CV_WINDOW_NORMAL);
    //cv::resizeWindow(WINDOW_RIGHT, 640, 480);
    
    cv::startWindowThread();
    image_transport::ImageTransport it1(nh), it2(nh);
    image_transport::Subscriber sub1 = it1.subscribe(CAM_SUB_LEFT, 1, imageCallbackLeft);
    image_transport::Subscriber sub2 = it2.subscribe(CAM_SUB_RIGHT, 1, imageCallbackRight);
    ros::spin();
    cv::destroyWindow(WINDOW_LEFT);
    cv::destroyWindow(WINDOW_RIGHT);
}
