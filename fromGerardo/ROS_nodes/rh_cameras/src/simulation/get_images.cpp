//
//  get_images.cpp
//  Helper function for the rh_calibration node
//
//  Created by Gerardo Aragon on November, 2012.
//  Copyright (c) 2012 Gerardo Aragon. All rights reserved.

#include <RHcam/RHcam.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <RHcam/settings.h>
#include <rh_cameras/CamerasSync.h>
#include <sensor_msgs/CameraInfo.h>
#include <rh_cameras/SetCameraInfo.h>
#include <rh_cameras/AcquireSrv.h>
#include <rh_cameras/GetCameraInfo.h>

bool showImage = false;

static const char WINDOW_LEFT[] = "Left image";
static const char WINDOW_RIGHT[] = "Right image";

static const char CAM_NAMEL[] = "left_camera";
static const char CAM_NAMER[] = "right_camera";

// Messages
static const char CAML_SUB[] = "/RH/left_camera/image";
static const char CAMR_SUB[] = "/RH/right_camera/image";
static const char CAM_ACQUIRE[] = "/RH/cmd/acquire";
static const char CAM_ACQUIRE_SRVL[] = "/RH/cmd/acquireLeftSrv";
static const char CAM_ACQUIRE_SRVR[] = "/RH/cmd/acquireRightSrv";
static const char CIL[] = "/RH/left_camera/set_camera_info";
static const char CIR[] = "/RH/right_camera/set_camera_info";
static const char OUT_CAM_INFOL[] = "/RH/left_camera/camera_info";
static const char OUT_CAM_INFOR[] = "/RH/right_camera/camera_info";
static const char GCIL[] = "/RH/left_camera/get_camera_info";
static const char GCIR[] = "/RH/right_camera/get_camera_info";

// Paramter Server
static const char INPUT_XML[] = "/RH/cameras/image_list";
static const char CAMERA_INFOL[] = "/RH/left_camera/camera_info_url";
static const char CAMERA_INFOR[] = "/RH/right_camera/camera_info_url";

namespace enc = sensor_msgs::image_encodings;

class RHcam_node
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    image_transport::Publisher imageL_pub_;
    image_transport::Publisher imageR_pub_;
    ros::Publisher pub_, infoL_pub_, infoR_pub_;
    ros::ServiceServer set_camera_info_srvL_, acquire_cam_left_srv_;
    ros::ServiceServer set_camera_info_srvR_, acquire_cam_right_srv_;
    ros::ServiceServer get_camera_infoL_srv_, get_camera_infoR_srv_;

	std::string urlL, urlR;
	Mat left, right;

    Mat leftIm_srv, rightIm_srv;

public:

	Settings s_;
    sensor_msgs::CameraInfo infoL, infoR;

    RHcam_node() : it_(nh_)
    {

		s_.node_path = ros::package::getPath("rh_cameras");
		nh_.getParam(CAMERA_INFOL, urlL);
		nh_.getParam(CAMERA_INFOR, urlR);

		ROS_INFO_STREAM(urlL);
		ROS_INFO_STREAM(urlR);

        if(nh_.hasParam(INPUT_XML))
        {
            nh_.getParam(INPUT_XML, s_.input);
        }else{
            ROS_ERROR_STREAM("Could not open image list: " << INPUT_XML);
        }
        s_.readList();
        ROS_INFO_STREAM("Input file read!");

		if (!s_.goodInput)
		{
		    ROS_ERROR("Invalid input detected. Application stopping.");
            nh_.shutdown();
		    return;
		}

		if(showImage)
		{
			cv::namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
			cv::resizeWindow(WINDOW_LEFT, 640, 480);
			cvMoveWindow(WINDOW_LEFT, 10, 10);

			cv::namedWindow(WINDOW_RIGHT, CV_WINDOW_NORMAL);
			cv::resizeWindow(WINDOW_RIGHT, 640, 480);
			cvMoveWindow(WINDOW_RIGHT, 650, 10);

			cv::startWindowThread();
			ROS_INFO("Windows initialised");
		}

        infoL = loadCameraInfo(urlL);
        infoR = loadCameraInfo(urlR);

        // Setup advertise and subscribe ROS messages
        //pub_ = nh_.advertise<sensor_msgs::JointState>(PTU_OUT, 1);
        imageL_pub_ = it_.advertise(CAML_SUB, 1);
        imageR_pub_ = it_.advertise(CAMR_SUB, 1);
        infoL_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(OUT_CAM_INFOL,1);
        infoR_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(OUT_CAM_INFOR,1);

        // Service call for setting calibration.
    	set_camera_info_srvL_ = nh_.advertiseService(CIL, &RHcam_node::setCameraInfoL, this);
    	set_camera_info_srvR_ = nh_.advertiseService(CIR, &RHcam_node::setCameraInfoR, this);
        acquire_cam_left_srv_ = nh_.advertiseService(CAM_ACQUIRE_SRVL, &RHcam_node::captureImageSrvLeft, this);
        acquire_cam_right_srv_ = nh_.advertiseService(CAM_ACQUIRE_SRVR, &RHcam_node::captureImageSrvRight, this);
        get_camera_infoL_srv_ = nh_.advertiseService(GCIL, &RHcam_node::getCameraInfoLSrv, this);
        get_camera_infoR_srv_ = nh_.advertiseService(GCIR, &RHcam_node::getCameraInfoRSrv, this);

        sub_ = nh_.subscribe(CAM_ACQUIRE, 1, &RHcam_node::captureImage, this);

        ROS_INFO("Node Initialised");

    }

    ~RHcam_node()
    {
        ROS_INFO("Exit node... ");
    }

    void captureImage(const rh_cameras::CamerasSync::ConstPtr& msg)
    {
        string str1 ("preview");
        string str2 ("full");

		ROS_INFO("Reading Images...");
        try
        {
            if (str1.compare(msg->data.c_str())) // full
            {

            }
            else if (str2.compare(msg->data.c_str())) // preview
            {
                ROS_FATAL("This capture mode is not supported");
            }
            else
            {
                ROS_ERROR("Capture mode not known: %s", msg->data.c_str());
                return;
            }

			// For each call, read images from the list
	        left = s_.nextImage(); // Get left image
	        right = s_.nextImage(); // Get right image

            cv_bridge::CvImage cvi;
            ros::Time time = ros::Time::now();

            // convert OpenCV image to ROS message (Left image)
            cvi.header.stamp = time;
            cvi.header.frame_id = CAM_NAMEL;
            cvi.encoding = "rgb8";
            cvi.image = left;

            infoL.header.stamp = time;
            infoL.width = left.cols;
            infoL.height = left.rows;
            infoL.header.frame_id = CAM_NAMEL;

            ROS_INFO_STREAM("Left Rows: " << left.rows << " Cols: " << left.cols);

            imageL_pub_.publish(cvi.toImageMsg());
            infoL_pub_.publish(infoL);

            //imageL_pub_.publish(cvi.toImageMsg());

            // convert OpenCV image to ROS message (Right image)
            cvi.header.stamp = time;
            cvi.header.frame_id = CAM_NAMER;
            cvi.encoding = "rgb8";
            cvi.image = right;

            infoR.header.stamp = time;
            infoR.width = right.cols;
            infoR.height = right.rows;
            infoR.header.frame_id = CAM_NAMER;

            ROS_INFO_STREAM("Right Rows: " << right.rows << " Cols: " << right.cols);

            imageR_pub_.publish(cvi.toImageMsg());
            infoR_pub_.publish(infoR);

            // Display Image
            if(showImage)
            {
                imshow(WINDOW_LEFT, left);
                imshow(WINDOW_RIGHT, right);
                waitKey(3);
            }

			ROS_INFO("Messages published");

        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("RHcam_node exception: %s", e.what());
            return;
        }
    }

    void printMatrix(Mat M, bool printType)
    {
        if(printType)
            ROS_INFO_STREAM("Matrix type: " << M.type());
        // dont print empty matrices
        if (M.empty()){
            ROS_INFO("---");
            return;
        }
        // loop through columns and rows of the matrix
        for(int i=0; i < M.rows; i++){
            for(int j=0; j < M.cols ; j++){
                if(M.type() == 6)
                    cout << M.at<double>(i,j) << "\t";
                else
                    cout << M.at<double>(i,j) << "\t";
            }
            cout<<endl;
        }
        cout<<endl;
    }

    sensor_msgs::CameraInfo loadCameraInfo(std::string url)
    {
        sensor_msgs::CameraInfo outInfo;

        try
		{
            ROS_INFO_STREAM("Loading URL: " << url);
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

                ROS_INFO("K:");
                printMatrix(K, true);
                ROS_INFO("P:");
                printMatrix(P, true);
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

    bool setCameraInfoL(rh_cameras::SetCameraInfo::Request& req,
                     rh_cameras::SetCameraInfo::Response& rsp)
	{
        std::string cam_name = req.camera_info.header.frame_id;

        string str1(CAM_NAMEL);

        if (str1.compare(cam_name) == 0) // left
        {
            ROS_INFO("New camera info of the left camera received");        }
        else
        {
            ROS_ERROR("Unknown camera name: %s", cam_name.c_str());
            rsp.status_message = "Unknown camera name!";
            rsp.success = false;
        }

        FileStorage fs(urlL, FileStorage::WRITE);
        ROS_INFO_STREAM("Saving camera parameters in: " << urlL);

        fs << "distortion_model" << "plumb_bob";

        fs << "camera_name" << cam_name;
        fs << "height" << (int)req.camera_info.width;
        fs << "width" << (int)req.camera_info.height;

        Mat K, D, P;//, F;
        K = Mat::zeros(3,3,CV_64F);
        //F = Mat::zeros(3,3,CV_64F);
        P = Mat::zeros(3,4,CV_64F);
        D = Mat::zeros(1,5,CV_64F);

        //vector<double>::iterator it_K = req.camera_info.K.begin();
        //vector<double>::iterator it_F = req.camera_info.F.begin();
        //vector<double>::iterator it_P = req.camera_info.P.begin();
        for(int i = 0; i < D.cols; i++)
        {
            D.at<double>(0,i) = req.camera_info.D[i];

            if(i < K.rows)
            {
                for(int j = 0 ; j < P.cols; j++)
                {
                    if(j < K.cols)
                    {
                        K.at<double>(i,j) = req.camera_info.K[3*i+j];
                        //F.at<double>(i,j) = *it_F; ++it_F;
                    }
                    P.at<double>(i,j) = req.camera_info.P[4*i+j];
                }
            }
        }

        fs << "K" << K;
        fs << "D" << D;
        fs << "P" << P;
        //fs << "F" << F;

        fs.release();

        rsp.status_message = "Saved camera_info to disk";
        rsp.success = true;

        return true;
	}

    bool setCameraInfoR(rh_cameras::SetCameraInfo::Request& req,
                     rh_cameras::SetCameraInfo::Response& rsp)
	{
        std::string cam_name = req.camera_info.header.frame_id;

        string str1 (CAM_NAMER);

        if (str1.compare(cam_name) == 0) // left
        {
            ROS_INFO("New camera info of the right camera received");
        }
        else
        {
            ROS_ERROR("Unknown camera name: %s", cam_name.c_str());
            rsp.status_message = "Unknown camera name!";
            rsp.success = false;
        }

        FileStorage fs(urlR, FileStorage::WRITE );
        ROS_INFO_STREAM("Saving camera parameters in: " << urlR);

        fs << "distortion_model" << "plumb_bob";

        fs << "camera_name" << cam_name;
        fs << "height" << (int)req.camera_info.width;
        fs << "width" << (int)req.camera_info.height;

        Mat K, D, P;//, F;
        K = Mat::zeros(3,3,CV_64F);
        //F = Mat::zeros(3,3,CV_64F);
        P = Mat::zeros(3,4,CV_64F);
        D = Mat::zeros(1,5,CV_64F);

        //vector<double>::iterator it_K = req.camera_info.K.begin();
        //vector<double>::iterator it_F = req.camera_info.F.begin();
        //vector<double>::iterator it_P = req.camera_info.P.begin();
        for(int i = 0; i < D.cols; i++)
        {
            D.at<double>(0,i) = req.camera_info.D[i];

            if(i < K.rows)
            {
                for(int j = 0 ; j < P.cols; j++)
                {
                    if(j < K.cols)
                    {
                        K.at<double>(i,j) = req.camera_info.K[3*i+j];
                        //F.at<double>(i,j) = *it_F; ++it_F;
                    }
                    P.at<double>(i,j) = req.camera_info.P[4*i+j];
                }
            }
        }

        fs << "K" << K;
        fs << "D" << D;
        fs << "P" << P;
        //fs << "F" << F;

        fs.release();

        rsp.status_message = "Saved camera_info to disk";
        rsp.success = true;

        return true;
	}
	
	bool getCameraInfoLSrv(rh_cameras::GetCameraInfo::Request& req, rh_cameras::GetCameraInfo::Response& rsp)
    {
        sensor_msgs::CameraInfo srv_info = loadCameraInfo(urlL);
        srv_info.width = left.cols;
        srv_info.height = left.rows;
        rsp.camera_info = srv_info;
        return true;
    }
    
    bool getCameraInfoRSrv(rh_cameras::GetCameraInfo::Request& req, rh_cameras::GetCameraInfo::Response& rsp)
    {
        sensor_msgs::CameraInfo srv_info = loadCameraInfo(urlR);
        srv_info.width = right.cols;
        srv_info.height = right.rows;
        rsp.camera_info = srv_info;
        return true;
    }

    bool captureImageSrvLeft(rh_cameras::AcquireSrv::Request& req, rh_cameras::AcquireSrv::Response& rsp)
    {
        string str1 ("preview");
        string str2 ("full");
        ROS_INFO("Capturing Image Left (Service)...");
        try
        {
            if (str1.compare(req.data.c_str())) // full
            {

            }
            else if (str2.compare(req.data.c_str())) // preview
            {
                ROS_FATAL("This capture mode is not supported");
                return false;
            }
            else
            {
                ROS_ERROR("Capture mode not known: %s", req.data.c_str());
                return false;
            }

            // For each call, read images from the list
            leftIm_srv = s_.nextImage(); // Get left image
            rightIm_srv = s_.nextImage(); // Get right image

            cv_bridge::CvImage cvi;
            ros::Time time = req.timeStamp;

            // convert OpenCV image to ROS message (Left image)
            cvi.header.stamp = time;
            cvi.header.frame_id = CAM_NAMEL;
            cvi.encoding = "rgb8";
            cvi.image = leftIm_srv;

            sensor_msgs::Image msg;
            cvi.toImageMsg(msg);
            rsp.img = msg;
            rsp.info = infoL;

            return true;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("RHcam_node exception (Service): %s", e.what());
            return false;
        }

    }

    bool captureImageSrvRight(rh_cameras::AcquireSrv::Request& req, rh_cameras::AcquireSrv::Response& rsp)
    {
        string str1 ("preview");
        string str2 ("full");
        ROS_INFO("Capturing Image Right (Service)...");
        try
        {
            if (str1.compare(req.data.c_str())) // full
            {

            }
            else if (str2.compare(req.data.c_str())) // preview
            {
                ROS_FATAL("This capture mode is not supported");
                return false;
            }
            else
            {
                ROS_ERROR("Capture mode not known: %s", req.data.c_str());
                return false;
            }

            cv_bridge::CvImage cvi;
            ros::Time time = req.timeStamp;

            // convert OpenCV image to ROS message (Left image)
            cvi.header.stamp = time;
            cvi.header.frame_id = CAM_NAMEL;
            cvi.encoding = "rgb8";
            cvi.image = rightIm_srv;

            sensor_msgs::Image msg;
            cvi.toImageMsg(msg);
            rsp.img = msg;
            rsp.info = infoR;

            return true;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("RHcam_node exception (Service): %s", e.what());
            return false;
        }

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RHcamera_simulation");
    RHcam_node c_;
    ros::spin();
    return 0;
}
