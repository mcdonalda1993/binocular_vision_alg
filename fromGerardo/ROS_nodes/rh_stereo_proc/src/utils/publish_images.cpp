#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <rh_vergence/VergenceStatus.h>
#include <fstream>

using namespace std;
using namespace cv;

const string IMAGE_TOPIC_L = "/RH/left_camera/image";
const string IMAGE_TOPIC_R = "/RH/right_camera/image";
const string INFO_TOPIC_L = "/RH/left_camera/camera_info";
const string INFO_TOPIC_R = "/RH/right_camera/camera_info";
const string RH_VERGE = "/RH/status/verge";
const string info_file_l = "calL.xml";
const string info_file_r = "calR.xml";

sensor_msgs::Image img_l;
sensor_msgs::Image img_r;
ros::Publisher img_pub_l, img_pub_r;
ros::Publisher info_pub_l, info_pub_r;
ros::Publisher status_pub;
sensor_msgs::CameraInfo info_l, info_r;
rh_vergence::VergenceStatus status;

// ============================================================================
sensor_msgs::Image loadImage(string path)
{
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    return ros_image;
}

// ============================================================================
sensor_msgs::CameraInfo loadCameraInfo(string url)
{
    sensor_msgs::CameraInfo outInfo;

    try
    {
        FileStorage fs(url.c_str(), FileStorage::READ);
        if(!fs.isOpened())
        {
            ROS_WARN_STREAM("An exception occurred. Using default values for /camera_info");
            fs.release();
            outInfo.K.empty();
            outInfo.D.empty();
            outInfo.P.empty();
            //outInfo.F.push_back(0);
        }
        else
        {

            Mat K, D, P;//, F
            fs["K"] >> K;
            fs["D"] >> D;
            //fs["F"] >> F;
            fs["P"] >> P;

            for(int i = 0; i < D.cols; i++)
            {
                outInfo.D.push_back(D.at<double>(i));
                if(i < P.rows)
                {
                    for(int j = 0 ; j < P.cols; j++)
                    {
                        if(j < K.cols)
                        {
                            outInfo.K[3*i+j] = K.at<double>(i,j);
                            //outInfo.F.push_back(F.at<double>(i,j));
                        }
                        outInfo.P[4*i+j] = P.at<double>(i,j);
                    }
                }
            }
        }

        ROS_WARN_STREAM_COND((int)outInfo.K.size() == 0, "Size cam_info K: " << outInfo.K.size());
        //ROS_WARN_STREAM_COND((int)outInfo.F.size() == 0, "Size cam_info F: " << outInfo.F.size());

    }
    catch (...)
    {
        ROS_ERROR("Failed to open file %s\n", url.c_str());
        ros::shutdown();
    }

    return outInfo;
}

// ============================================================================
int main(int argc, char** argv)
{
    if(argc == 3)
    {
        img_l = loadImage(argv[1]);
        img_r = loadImage(argv[2]);
        ROS_INFO("Images loaded.");
    }
    else
    {
        ROS_INFO("You have to choose l and r image as argument.");
        return 1;
    }

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    string calib_loc = ros::package::getPath("rh_cameras") + "/calibrations/";

    info_l = loadCameraInfo(calib_loc + getenv("CLOPEMA_PARTNER") + "/" + info_file_l);
    info_r = loadCameraInfo(calib_loc + getenv("CLOPEMA_PARTNER") + "/" + info_file_r);
    ROS_INFO("Camera info loaded.");

    img_pub_l = nh.advertise<sensor_msgs::Image>(IMAGE_TOPIC_L, 1);
    img_pub_r = nh.advertise<sensor_msgs::Image>(IMAGE_TOPIC_R, 1);
    info_pub_l = nh.advertise<sensor_msgs::CameraInfo>(INFO_TOPIC_L,1);
    info_pub_r = nh.advertise<sensor_msgs::CameraInfo>(INFO_TOPIC_R,1);
    status_pub = nh.advertise<rh_vergence::VergenceStatus>(RH_VERGE, 1);

    ros::Rate loop_rate(5);
    while (nh.ok())
    {
        if(img_pub_l.getNumSubscribers() > 0  && img_pub_r.getNumSubscribers() > 0
                && info_pub_l.getNumSubscribers() > 0 && info_pub_r.getNumSubscribers() > 0
                && status_pub.getNumSubscribers() > 0)
        {

            ros::Time time = ros::Time::now();
            img_l.header.stamp = time;
            img_r.header.stamp = time;
            info_l.header.stamp = time;
            info_r.header.stamp = time;
            status.header.stamp = time;
            status.data = true;

            ROS_INFO("All messages got time stamp.");

            img_pub_l.publish(img_l);
            img_pub_r.publish(img_r);
            info_pub_l.publish(info_l);
            info_pub_r.publish(info_r);
            status_pub.publish(status);

            loop_rate.sleep();

            ros::spinOnce();
            ROS_INFO("All messages published!.");
            break;
        }
        loop_rate.sleep();
    }
}
