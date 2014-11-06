// Code based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
// SoCS, UoG. Gerardo Aragon-Camarasa. July, 2012

// Set /usbcam_left in the parameter server before invoking the node

#include <RHcam/RHcam.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <rh_cameras/CamerasSync.h>
#include <sensor_msgs/CameraInfo.h>
#include <rh_cameras/SetCameraInfo.h>
#include <rh_cameras/GetCameraInfo.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <Utils/utils.h>
#include <rh_cameras/AcquireSrv.h>


bool showImage = false;
using namespace std;

static const char CAMERA_NAME[] = "left_camera";

// For tf
static const char LEFT_TILT[] = "left_tilt_ptu";
static const char RIGHT_TILT[] = "right_tilt_ptu";
static const char LEFT_CAMERA[] = "left_camera";
static const char RIGHT_CAMERA[] = "right_camera";

// Parameter Server
static const char LEFT_CAM_DEVICE[] = "/left_camera_device";
static const char USBPARAM[] = "/RH/usbcam_left";
static const char CAMERA_INFO_URL[] = "/RH/left_camera/camera_info_url";
// Only done in the left camera as both cameras recieve the capture command at the same time
static const char CAPTURE_MODE[] = "/CAM/capturemode";
static const char HE_CALIBFILE[] = "/RH/gHc_calibration_file";

// Messages
static const char CAM_SUB[] = "/RH/left_camera/image";
static const char CAM_ACQUIRE[] = "/RH/cmd/acquire";
static const char CAM_ACQUIRE_SRV[] = "/RH/cmd/acquireLeftSrv";
static const char CI[] = "/RH/left_camera/set_camera_info";
static const char GCI[] = "/RH/left_camera/get_camera_info";
static const char OUT_CAM_INFO[] = "/RH/left_camera/camera_info";

namespace enc = sensor_msgs::image_encodings;

class RHcam_node
{

public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    image_transport::Publisher image_pub_;
    boost::mutex photo_mutex_;
    ros::Publisher camera_info_pub_;

    DSLR_cam cam_;

    ros::ServiceServer set_camera_info_srv_, acquire_cam_srv_, get_camera_info_srv_;

    std::string url;
    sensor_msgs::CameraInfo info;

    //bool load_heQ;

    Mat mat_he;
    tf::Quaternion quaternion_part;
    tf::Vector3 translation_part;

    int prevMode;

    bool nodeOK()
    {
        return nh_.ok();
    }

    RHcam_node() : it_(nh_), cam_(NULL)
    {
        std::string dev_param;
        if (nh_.hasParam(LEFT_CAM_DEVICE))
        {
            nh_.getParam(LEFT_CAM_DEVICE, dev_param);
        }

        // PARSE USB PORT FROM DEVICE
        // ==========================================================================================
        string usb_param = getOtuputFromCommand("readlink " + dev_param);
        stringReplace(usb_param, "bus/usb/", "usb:");
        stringReplace(usb_param, "/", ",");
        nh_.setParam(USBPARAM, usb_param);
        // ==========================================================================================


        nh_.getParam(CAMERA_INFO_URL, url);

        photo_mutex_.lock();
        cam_ = camera_init();
        cam_->port = usb_param.c_str();

        ROS_INFO("PORT: %s", cam_->port);

        // Loading hand-eye information
        std::string param_he;
        if (nh_.hasParam(HE_CALIBFILE))
        {
            nh_.getParam(HE_CALIBFILE, param_he);
        }
        else
        {
            ROS_ERROR("RHcam_node exception: hand-eye calibrations is not defined yet!");
            ros::shutdown();
            return;
        }

        // FileStorage fs(param_he, FileStorage::READ); // Read the matrices

        // if (fs.isOpened())
        // {
        //     fs["gripperHcamera_LeftCamera"] >> mat_he;

        //     //ROS_INFO("Left hand-eye:");
        //     //printMatrix(mat_he,true);

        //     fs.release(); // close Settings file
        //     ROS_INFO("Hand-eye calibration info loaded!");

        //     tf::Matrix3x3 r_temp(mat_he.at<float>(0,0), mat_he.at<float>(0,1), mat_he.at<float>(0,2),
        //                          mat_he.at<float>(1,0), mat_he.at<float>(1,1), mat_he.at<float>(1,2),
        //                          mat_he.at<float>(2,0), mat_he.at<float>(2,1), mat_he.at<float>(2,2));

        //     tf::Vector3 t_temp(mat_he.at<float>(0,3), mat_he.at<float>(1,3), mat_he.at<float>(2,3));

        //     translation_part = t_temp;

        //     r_temp.getRotation(quaternion_part);

        //     ROS_INFO_STREAM(quaternion_part.getX() << ", " << quaternion_part.getY() << ", " << quaternion_part.getZ() << ", " << quaternion_part.getW());

        //     load_heQ = true;
        // }
        // else
        // {
        //     ROS_INFO_STREAM("Couldn't read HE parameters of the left camera");
        //     fs.release(); // close Settings file
        //     load_heQ = false;
        // }

        prevMode = 0;
        if(camera_open(cam_) < GP_OK)
        {
            ROS_ERROR("RHcam_node exception: camera is not available");
            ros::shutdown();
        }
        photo_mutex_.unlock();
        //info = loadCameraInfo();

        ROS_INFO("Camera opened... ");

        // Service call for setting calibration.
    	set_camera_info_srv_ = nh_.advertiseService(CI, &RHcam_node::setCameraInfo, this);
        acquire_cam_srv_ = nh_.advertiseService(CAM_ACQUIRE_SRV, &RHcam_node::captureImageSrv, this);
        get_camera_info_srv_ = nh_.advertiseService(GCI, &RHcam_node::getCameraInfoSrv, this);

        // Setup advertise and subscribe ROS messages
        image_pub_ = it_.advertise(CAM_SUB, 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(OUT_CAM_INFO,1);
        sub_ = nh_.subscribe(CAM_ACQUIRE, 1, &RHcam_node::captureImage, this);

        // Setup OpenCV image display windows
        if(showImage)
        {
            namedWindow("DSLR image", CV_WINDOW_NORMAL);
            cvStartWindowThread();
        }

    }

    ~RHcam_node()
    {
        if(showImage)
            destroyWindow("DSLR image");

        ROS_INFO("Exit and free cameras... ");
        photo_mutex_.lock();
        camera_close(cam_);
        photo_mutex_.unlock();
    }

    void captureImage(const rh_cameras::CamerasSync::ConstPtr& msg)
    {
        string str1 ("preview");
        string str2 ("full");

        ROS_INFO("Capturing Image...");
        try
        {
            if (str1.compare(msg->data.c_str())) // full
            {
                cam_->mode = 0;
                ROS_INFO("Full resolution mode");
                nh_.setParam(CAPTURE_MODE, 0); // Set full resolution
            }
            else if (str2.compare(msg->data.c_str())) // preview
            {
                cam_->mode = 1;
                ROS_INFO("Preview mode");
                nh_.setParam(CAPTURE_MODE, 1); // Set preview resolution
            }
            else
            {
                ROS_ERROR("Capture mode not known: %s", msg->data.c_str());
                return;
            }

            if (prevMode != cam_->mode)
            {
                cam_->firstCapture = 1;
            }
            prevMode = cam_->mode;

            photo_mutex_.lock();
            // Capture image
            if (camera_capture(cam_) < GP_OK)
            {
                ROS_ERROR("RHcam_node exception: cannot capture image");
                ros::shutdown();
                return;
            }
            photo_mutex_.unlock();

            Mat bgr, rgb;	// image from camera
            cv_bridge::CvImage cvi;
            ros::Time time = msg->timeStamp;

            //LEFT IMAGE
            bgr = cvarrToMat(cam_->imgBGR_left);
            rgb = cvarrToMat(cam_->imgRGB_left);
            cvtColor(bgr, rgb, CV_BGR2RGB);

            // convert OpenCV image to ROS message
            cvi.header.stamp = time;
            cvi.header.frame_id = CAMERA_NAME;
            cvi.encoding = "rgb8";
            cvi.image = rgb;

            info = loadCameraInfo();
            info.header.stamp = cvi.header.stamp;
            info.width = rgb.cols;
            info.height = rgb.rows;

            image_pub_.publish(cvi.toImageMsg());
            camera_info_pub_.publish(info);

            // Display Image
            if(showImage)
            {
                imshow("DSLR image", cvi.image);
                waitKey(3);
            }
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("RHcam_node exception: %s", e.what());
            return;
        }
    }

    sensor_msgs::CameraInfo loadCameraInfo()
    {
        sensor_msgs::CameraInfo outInfo;

        FileStorage fs(url.c_str(), FileStorage::READ);
        if(!fs.isOpened())
        {
            ROS_ERROR("Failed to open file %s\n", url.c_str());
            ros::shutdown();
        }

        Mat K, D, P;//, F
        fs["K"] >> K;
        fs["D"] >> D;
        //fs["F"] >> F;
        fs["P"] >> P;

        Mat P_updated;
        // if(load_heQ)
        // {
            if(!get_rcHlc(K, P_updated))
            {
                ROS_WARN("LEFT_CAMERA::Camera projection matrix was not updated!");
                P_updated = P.clone();
            }
            else
                ROS_INFO("LEFT_CAMERA::Camera projection matrix was updated!");
        // }
        // else
        // {
        //     ROS_WARN("LEFT_CAMERA::Camera projection matrix was not updated!");
        //     P_updated = P.clone();
        // }

        ROS_INFO("Projection matrix (left_camera):");
        printMatrix(P_updated,true);

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
                    outInfo.P[4*i+j] = P_updated.at<double>(i,j);
                }
            }
        }

        outInfo.header.frame_id = CAMERA_NAME;

        return outInfo;

    }

    bool captureImageSrv(rh_cameras::AcquireSrv::Request& req, rh_cameras::AcquireSrv::Response& rsp)
    {
        string str1 ("preview");
        string str2 ("full");
        ROS_INFO("Capturing Image Left (Service)...");
        try
        {
            if (str1.compare(req.data.c_str())) // full
            {
                cam_->mode = 0;
                ROS_INFO("Full resolution mode");
                nh_.setParam(CAPTURE_MODE, 0); // Set full resolution
            }
            else if (str2.compare(req.data.c_str())) // preview
            {
                cam_->mode = 1;
                ROS_INFO("Preview mode");
                nh_.setParam(CAPTURE_MODE, 1); // Set preview resolution
            }
            else
            {
                ROS_ERROR("Capture mode not known: %s", req.data.c_str());
                return false;
            }

            if (prevMode != cam_->mode)
            {
                cam_->firstCapture = 1;
            }
            prevMode = cam_->mode;

            photo_mutex_.lock();
            // Capture image
            if (camera_capture(cam_) < GP_OK)
            {
                ROS_ERROR("RHcam_node exception: cannot capture image");
                ros::shutdown();
                return false;
            }
            photo_mutex_.unlock();

            Mat bgr, rgb; // image from camera
            cv_bridge::CvImage cvi;
            ros::Time time = req.timeStamp;

            //LEFT IMAGE
            bgr = cvarrToMat(cam_->imgBGR_left);
            rgb = cvarrToMat(cam_->imgRGB_left);
            cvtColor(bgr, rgb, CV_BGR2RGB);

            // convert OpenCV image to ROS message
            cvi.header.stamp = time;
            cvi.header.frame_id = CAMERA_NAME;
            cvi.encoding = "rgb8";
            cvi.image = rgb;

            info = loadCameraInfo();
            info.header.stamp = cvi.header.stamp;
            info.width = rgb.cols;
            info.height = rgb.rows;

            sensor_msgs::Image msg;
            cvi.toImageMsg(msg);
            rsp.img = msg;
            rsp.info = info;

            return true;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("RHcam_node exception (Service): %s", e.what());
            return false;
        }

    }

    bool setCameraInfo(rh_cameras::SetCameraInfo::Request& req, rh_cameras::SetCameraInfo::Response& rsp)
    {

        std::string cam_name = req.camera_info.header.frame_id;

        string str1(CAMERA_NAME);

        if (str1.compare(cam_name) == 0) // left
        {
            ROS_INFO("New camera info of the left camera received");
        }
        else
        {
            ROS_ERROR("Unknown camera name: %s", cam_name.c_str());
            rsp.status_message = "Unknown camera name!";
            rsp.success = false;
        }

        FileStorage fs(url, FileStorage::WRITE);
        ROS_INFO_STREAM("Saving camera parameters in: " << url);

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

    bool getCameraInfoSrv(rh_cameras::GetCameraInfo::Request& req, rh_cameras::GetCameraInfo::Response& rsp)
    {
        sensor_msgs::CameraInfo srv_info = loadCameraInfo();
        Mat bgr = cvarrToMat(cam_->imgBGR_left);
        srv_info.width = bgr.cols;
        srv_info.height = bgr.rows;
        rsp.camera_info = srv_info;
        return true;
    }

    bool get_rcHlc(Mat cameraMatrix, Mat& P_out)
    {
        //printMatrix(cameraMatrix, true);
        hconcat(cameraMatrix, Mat::zeros(3,1,CV_64F), P_out);
        return true;
    }

    Mat getRotation(double x, double y, double z, double w)
    {
        Mat rotK = Mat::zeros(3,3,CV_64F);

        double sqw = w*w;
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        double m11 = (-sqx + sqy - sqz + sqw)*invs ;
        double m22 = (-sqx - sqy + sqz + sqw)*invs ;

        double tmp1 = x*y;
        double tmp2 = z*w;
        double m10 = 2.0 * (tmp1 + tmp2)*invs ;
        double m01 = 2.0 * (tmp1 - tmp2)*invs ;

        tmp1 = x*z;
        tmp2 = y*w;
        double m20 = 2.0 * (tmp1 - tmp2)*invs ;
        double m02 = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = y*z;
        tmp2 = x*w;
        double m21 = 2.0 * (tmp1 + tmp2)*invs ;
        double m12 = 2.0 * (tmp1 - tmp2)*invs ;

        rotK.at<double>(0,0) = m00;
        rotK.at<double>(0,1) = m01;
        rotK.at<double>(0,2) = m02;
        rotK.at<double>(1,0) = m10;
        rotK.at<double>(1,1) = m11;
        rotK.at<double>(1,2) = m12;
        rotK.at<double>(2,0) = m20;
        rotK.at<double>(2,1) = m21;
        rotK.at<double>(2,2) = m22;

        return rotK;
    }

    void printMatrix(Mat M, bool printType = true)
    {
        if(printType)
            ROS_INFO_STREAM("Matrix type:" << M.type());
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
                    cout << M.at<float>(i,j) << "\t";
            }
            cout<<endl;
        }
        cout<<endl;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RH_camera_left");
    RHcam_node c_;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    bool onceQ = true;
    ros::Rate rate(10.0);
    while (c_.nodeOK()){
        // if(c_.load_heQ){
        //     transform.setOrigin(c_.translation_part);
        //     transform.setRotation(c_.quaternion_part);
        //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), LEFT_TILT, LEFT_CAMERA));
        // }
        // else
        // {
        //     if(onceQ)
        //         ROS_ERROR("LEFT_CAMERA::Hand-eye transformation not defined!!");

        //     onceQ = false;
        // }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
