#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

namespace enc = sensor_msgs::image_encodings;

// Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";

int nrFramesL;
int nrFramesR;

void saveImages(std::string str1, const cv::Mat& im, int numF)
{
	std::stringstream ss;
	ss << numF;
	std::string outputDir = ros::package::getPath("rh_cameras") + "/";
	std::string out_image = outputDir + ss.str() + str1;
	ROS_INFO("Saving image to: %s", out_image.c_str());
	
	cv::imwrite(out_image, im);
	
	ROS_INFO("Images saved!");
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
    
    saveImages("_L.tif", cv_ptr->image, nrFramesL);
    nrFramesL++;
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
    
    saveImages("_R.tif", cv_ptr->image, nrFramesR);
    nrFramesR++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RHcam_saveImgs");
    ros::NodeHandle nh;
    
    nrFramesL = 0;
    nrFramesR = 0;
    
    image_transport::ImageTransport it1(nh), it2(nh);
    image_transport::Subscriber sub1 = it1.subscribe(CAM_SUB_LEFT, 1, imageCallbackLeft);
    image_transport::Subscriber sub2 = it2.subscribe(CAM_SUB_RIGHT, 1, imageCallbackRight);
    ros::spin();

}
