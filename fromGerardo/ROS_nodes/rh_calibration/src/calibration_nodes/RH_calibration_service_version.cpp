//
//  RHcalibration_service_version.cpp
//  Camera Calibration ROS node
//
//  Created by Gerardo Aragon on 01/2014.
//  Copyright (c) 2014 Gerardo Aragon. All rights reserved.

#include <rh_calibration/RHcalibration.h>
#include <rh_calibration/parameters.h>
#include <rh_calibration/camera_calibration.h>
#include <rh_calibration/handeye_calibration.h>

#define VELOCITY 1000

enum SaveOp {nothing, images, all};
enum CameraUsed {leftCam, rightCam};

class CmainCalibration
{
public:

    Calibration c_;
    Chandeye hE_left, hE_right;
    Chandeye hE_robotL, hE_robotR;
    tf::TransformListener tf_;

    vector<vector<Point2f> > imagePointsL;
    vector<vector<Point2f> > imagePointsR;

    vector<vector<Point2f> > pointsL_handEye;
    vector<vector<Point2f> > pointsR_handEye;

    vector<Mat> rvecsL_cam, tvecsL_cam;
    vector<Mat> rvecsR_cam, tvecsR_cam;

    vector<Mat> rvecsL_he, tvecsL_he;
    vector<Mat> rvecsR_he, tvecsR_he;

    vector<Mat> rvecsL_tilt2world, tvecsL_tilt2world;
    vector<Mat> rvecsR_tilt2world, tvecsR_tilt2world;

    vector<Mat> rvecs_rb2gripperL, tvecs_rb2gripperL;
    vector<Mat> rvecs_rb2gripperR, tvecs_rb2gripperR;

    string ROBOT_GRIPPER;

    vector<Point2f> ptuL, ptuR;

    Mat cameraMatrixL, distCoeffsL;
    Mat cameraMatrixR, distCoeffsR;
    Mat_<float> H_stereo;

    int key;
    float ptX, ptY;

    float stpPerPixelX;
    float stpPerPixelY;

    Mat imLeft, imRight, cornersImgLeft, cornersImgRight;
    bool found_chees;

    ros::Publisher movePTU_pub;
    ros::Publisher acquire_pub;

    cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;

    ros::ServiceClient camera_infoL, camera_infoR, ptu_infoStatus, move_ptuSrv, getMarkerData_srv;

    ros::ServiceServer process_target_srv_;
    ros::ServiceServer camera_calib_srv_;
    ros::ServiceServer capture_handeye_srv_;

    bool debugQ;
    int noImgPtu, countImgPtu;
    vector<Mat> bigMatQ, bigMatE;

    CmainCalibration() : it_(nh_),
        imL_sub_(it_, CAM_SUB_LEFT, 5),
        imR_sub_(it_, CAM_SUB_RIGHT, 5),
        sync(syncPolicy(5), imL_sub_, imR_sub_)
    {
        c_.init();

        double param;
        if (nh_.hasParam(STP_PIX_X) && nh_.hasParam(STP_PIX_Y))
        {
            nh_.getParam(STP_PIX_X, param);
            stpPerPixelX = (float)param;
            nh_.getParam(STP_PIX_Y, param);
            stpPerPixelY = (float)param;

        }
        else
        {
            ROS_ERROR("\"%s\" and/or \"%s\" parameters are not set in the server", STP_PIX_X, STP_PIX_Y);
            nh_.shutdown();
            return;
        }

        if(nh_.hasParam(CALIB_TARGET))
        {
            nh_.getParam(CALIB_TARGET, c_.calibTarget);
        }else{
            ROS_ERROR("\"%s\" parameter is not set in the server", CALIB_TARGET);
        }

        if(nh_.hasParam(ARM_Q))
        {
            nh_.getParam(ARM_Q, ROBOT_GRIPPER);
        }else{
            ROS_ERROR("\"%s\" parameter is not set in the server", ARM_Q);
        }

        if(nh_.hasParam(OUTPUT_IMAGE_DIR) && nh_.hasParam(OUTPUT_CALIB_DIR) && nh_.hasParam(MAX_ERROR_TH))
        {
            nh_.getParam(OUTPUT_IMAGE_DIR, c_.imageOutputDir);
            nh_.getParam(OUTPUT_CALIB_DIR, c_.calibOutputDir);
            double temp;
            nh_.getParam(MAX_ERROR_TH, temp);
            c_.maxError = (float)temp;

        }else{
            ROS_ERROR("\"%s\", \"%s\" and/or \"%s\" parameters are not set in the server", OUTPUT_IMAGE_DIR, OUTPUT_CALIB_DIR, MAX_ERROR_TH);
            nh_.shutdown();
            return;
        }

        if(nh_.hasParam(WIDTH_MARKER) && nh_.hasParam(MARKER_SIZE_X) && nh_.hasParam(MARKER_SIZE_Y) && nh_.hasParam(CALIB_TARGET)
                && nh_.hasParam(SAVE_MODE) && nh_.hasParam(INPUT_SCALE))
        {
            nh_.getParam(WIDTH_MARKER,param);
            c_.squareSize = (float)param;
            nh_.getParam(MARKER_SIZE_X,param);
            c_.boardSize.width = (int)param;
            nh_.getParam(MARKER_SIZE_Y,param);
            c_.boardSize.height = (int)param;
            nh_.getParam(CALIB_TARGET, c_.calibTarget);
            nh_.getParam(SAVE_MODE,param);
            c_.save_mode = (int)param;
            nh_.getParam(INPUT_SCALE,param);
            c_.scaleInput = (float)param;
            nh_.getParam(HE_CALIB_FILE_URL, c_.he_calib_url);
        }else{
            ROS_ERROR("Check that \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\" and/or \"%s\" parameters are set in the server", WIDTH_MARKER, MARKER_SIZE_X, MARKER_SIZE_Y, CALIB_TARGET, SAVE_MODE, INPUT_SCALE, HE_CALIB_FILE_URL);
        }

        if(nh_.hasParam(DEBUGQ) && nh_.hasParam(NO_IMGS_PTU))
        {
        	nh_.getParam(DEBUGQ, debugQ);
        	nh_.getParam(NO_IMGS_PTU, noImgPtu);
        }else{
        	ROS_ERROR("Check that \"%s\", and/or \"%s\" parameters are set in the server", DEBUGQ, NO_IMGS_PTU);
        }

        c_.nrFrames = 1;
        countImgPtu = 1;
        
        string cmd = "exec rm -r " + c_.calibOutputDir + "*.xml";
        system(cmd.c_str());
        cmd = "exec rm -r " + c_.imageOutputDir + "*.tif";
        system(cmd.c_str());
        cmd = "exec rm -r " + c_.calibOutputDir + "*.py";
        system(cmd.c_str());
        cmd = "exec mkdir " + c_.imageOutputDir;
        system(cmd.c_str());
        // ***********************
        rvecsL_he.resize(0);
        tvecsL_he.resize(0);
        rvecsR_he.resize(0);
        tvecsR_he.resize(0);

        rvecsL_tilt2world.resize(0);
        tvecsL_tilt2world.resize(0);
        rvecsR_tilt2world.resize(0);
        tvecsR_tilt2world.resize(0);

        rvecs_rb2gripperL.resize(0);
        tvecs_rb2gripperL.resize(0);
        rvecs_rb2gripperR.resize(0);
        tvecs_rb2gripperR.resize(0);

        imagePointsL.resize(0);
        imagePointsR.resize(0);

        bigMatQ.resize(0);
        bigMatE.resize(0);

        nh_.setParam(CAPTURE_MODE, 0); // Set full resolution

        cv::namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_LEFT, 640, 480);
        cvMoveWindow(WINDOW_LEFT, 10, 10);

        cv::namedWindow(WINDOW_RIGHT, CV_WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_RIGHT, 640, 480);
        cvMoveWindow(WINDOW_RIGHT, 705, 10);

//        cv::namedWindow(WINDOW_LEFT_CORNERS, CV_WINDOW_NORMAL);
//        cv::resizeWindow(WINDOW_LEFT_CORNERS, 640, 480);
//        cvMoveWindow(WINDOW_LEFT_CORNERS, 10, 530);

//        cv::namedWindow(WINDOW_RIGHT_CORNERS, CV_WINDOW_NORMAL);
//        cv::resizeWindow(WINDOW_RIGHT_CORNERS, 640, 480);
//        cvMoveWindow(WINDOW_RIGHT_CORNERS, 705, 530);

        // Service calls
        process_target_srv_ = nh_.advertiseService(CAPTURE_CAMERAS, &CmainCalibration::processTargetSrv, this);
        camera_calib_srv_ = nh_.advertiseService(CAMERA_CALIB, &CmainCalibration::cameraCalibrationSrv, this);
        capture_handeye_srv_ = nh_.advertiseService(CAPTURE_HANDEYE, &CmainCalibration::captureHandEyeSrv, this);

        //movePTU_pub = nh_.advertise<sensor_msgs::JointState>(PTU_OUT, 1);
        //acquire_pub = nh_.advertise<rh_cameras::CamerasSync>(CAM_ACQUIRE, 1);

        //capture_images();
        cv::startWindowThread();

        key = -1;
        cv::setMouseCallback(WINDOW_LEFT, on_mouse, this);
        ROS_INFO("Node RH_caibration initialised, capturing images...");

        camera_infoL = nh_.serviceClient<rh_cameras::SetCameraInfo>(CIL);
        camera_infoR = nh_.serviceClient<rh_cameras::SetCameraInfo>(CIR);
        ptu_infoStatus = nh_.serviceClient<rh_ptu::GetPtuInfo>(PTU_INFO);
        move_ptuSrv = nh_.serviceClient<rh_ptu::MovePtu>(PTU_MOVE_SRV);
        getMarkerData_srv = nh_.serviceClient<rh_integration::MarkerDetection>(MARKER_SERVICE);

        sync.registerCallback(boost::bind(&CmainCalibration::mainRoutine, this, _1, _2));//, _3, _4));
    }

    ~CmainCalibration()
    {
        ROS_INFO("Bye!");
    }

    void mainRoutine(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR)
    {
        ROS_INFO("Reading messages!");
        // Get images
        try
        {
            cv_ptrL = cv_bridge::toCvCopy(imL, enc::RGB8);
            cv_ptrR = cv_bridge::toCvCopy(imR, enc::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'rgb8'.", imL->encoding.c_str());
            return;
        }

        imLeft = cv_ptrL->image;
        imRight = cv_ptrR->image;

        // Display images
        Mat dispImgL, dispImgR;
        imLeft.copyTo(dispImgL);
        imRight.copyTo(dispImgR);

        line(dispImgL, Point(imLeft.cols/2.0, 0), Point(imLeft.cols/2.0, imLeft.rows), Scalar( 0, 255, 0), 5);
        line(dispImgL, Point(0, imLeft.rows/2.0), Point(imLeft.cols, imLeft.rows/2.0), Scalar( 0, 255, 0), 5);
        line(dispImgR, Point(imLeft.cols/2.0, 0), Point(imLeft.cols/2.0, imLeft.rows), Scalar( 0, 255, 0), 5);
        line(dispImgR, Point(0, imLeft.rows/2.0), Point(imLeft.cols, imLeft.rows/2.0), Scalar( 0, 255, 0), 5);

        cv::imshow(WINDOW_LEFT, dispImgL);
        cv::imshow(WINDOW_RIGHT, dispImgR);

        return;

    }


private:

    vector<Point2f> pointBuf;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    typedef image_transport::SubscriberFilter ImageSubscriber;

    ImageSubscriber imL_sub_;
    ImageSubscriber imR_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync;

    static void on_mouse(int event, int x, int y, int flags, void *param )
    {
        CmainCalibration *node = (CmainCalibration *)param;

        node->key = -1;
        switch(event)
        {
        case CV_EVENT_LBUTTONUP: // Capture images for hand-eye calibration
            ROS_INFO("Capture images for hand-eye calibration");
            node->ptX = (float)x;
            node->ptY = (float)y;
            node->key = 0;
            ROS_INFO("Move to point [%d, %d]", x, y);
            break;
        case CV_EVENT_RBUTTONUP:
            ROS_INFO("Stop capturing images for hand-eye calibration");
            node->key = 1;
            break;
        }
    }

    bool processTargetSrv(rh_calibration::CameraCalibration::Request& req, rh_calibration::CameraCalibration::Response& rsp)
    {
        if(req.doIt == true)
        {
            if(modeCameras(imLeft, imRight, false))
            {
                rsp.status_message = "Images saved!";
                rsp.success = true;
                return true;
            }
            else
            {
                rsp.status_message = "Images were not saved";
                rsp.success = false;
                return true;
            }
        }
        else
        {
            rsp.status_message = "Nothing to do";
            rsp.success = true;
            return true;
        }

    }

    bool cameraCalibrationSrv(rh_calibration::CameraCalibration::Request& req, rh_calibration::CameraCalibration::Response& rsp)
    {
        if(req.doIt == true)
        {
            if(modeCameras(imLeft, imRight, true))
            {
                rsp.status_message = "Cameras are calibrated!";
                rsp.success = true;
            }
            else
            {
                rsp.status_message = "Camera calibration failed!!";
                rsp.success = false;
                return false;
            }
        }
        else
        {
            rsp.status_message = "Nothing to do";
            rsp.success = true;
        }

        return true;
    }

    bool captureHandEyeSrv(rh_calibration::HandEyeCalibration::Request& req, rh_calibration::HandEyeCalibration::Response& rsp)
    {
        if(req.doIt == true)
        {
            int ret = modeHandEye(imLeft, imRight, false);
            if(ret == 0)
            {
                rsp.status_message = "Images saved for hand eye!";
                rsp.success = 1;
                return true;
            }
            else if(ret == 1)
            {
                if(modeHandEye(imLeft, imRight, true) == 0)
                {
                    rsp.status_message = "Hand eye calibration done!";
                    rsp.success = 2;

                    ROS_WARN("The robot head is now calibrated!");
                    ROS_WARN("Camera intrinsic and extrensic parameters have already been saved in calL.xml and calR.xml in rh_cameras node!");
                    ROS_WARN("Press Control+C to terminate this node");
                    return true;
                }
                else
                {
                    rsp.status_message = "Hand eye calibration failed!";
                    rsp.success = 0;
                    return false;
                }
            }
            else
            {
                rsp.status_message = "Images were not saved";
                rsp.success = 1;
                return true;
            }
        }
        else
        {
            rsp.status_message = "Nothing to do";
            rsp.success = 1;
            return true;
        }
    }

    bool modeCameras(const Mat& imL, const Mat& imR, bool calibQ)
    {
        int totalFrames = 8;
        vector<Point2f> pointBufL, pointBufR;
        bool found1, found2;

        if(!calibQ)
        {
            Mat inImL, inImR;

            imL.copyTo(inImL);
            imR.copyTo(inImR);

            // Find corners
            if(c_.calibTarget == c_.alvarStr)
            {
                found1 = detectMarker(cv_ptrL, cv_ptrR);
                found2 = found1;

            }else if(c_.calibTarget == c_.opencvStr)
            {
                ROS_INFO("Finding chessboard corners");
                found_chees = false;
                Mat leftOut = findCorners(inImL);
                found1 = found_chees;
                pointBufL = pointBuf;

                found_chees = false;
                Mat rightOut = findCorners(inImR);
                pointBufR = pointBuf;
                found2 = found_chees;
                
                Mat dispImgL, dispImgR;
                leftOut.copyTo(dispImgL);
                rightOut.copyTo(dispImgR);
                
                line(dispImgL, Point(leftOut.cols/2.0, 0), Point(leftOut.cols/2.0, leftOut.rows), Scalar( 0, 255, 0), 5);
                line(dispImgL, Point(0, leftOut.rows/2.0), Point(leftOut.cols, leftOut.rows/2.0), Scalar( 0, 255, 0), 5);
                line(dispImgR, Point(leftOut.cols/2.0, 0), Point(leftOut.cols/2.0, leftOut.rows), Scalar( 0, 255, 0), 5);
                line(dispImgR, Point(0, leftOut.rows/2.0), Point(leftOut.cols, leftOut.rows/2.0), Scalar( 0, 255, 0), 5);

                cv::imshow(WINDOW_LEFT, dispImgL);
                cv::imshow(WINDOW_RIGHT, dispImgR);

                if(found1 && found2){
                    imagePointsL.push_back(pointBufL);
                    imagePointsR.push_back(pointBufR);
                }
            }

            if(found1 & found2)
            {
                ROS_INFO("Saving images for camera calibration");
                // It saves the images on hardisk for debug purposes and
                // find chessboard corners

                if(c_.save_mode >= images)
                    saveImages("_L.tif", "_R.tif", imL, imR);

                ROS_INFO_STREAM("Transform from: " << ROBOT_BASE << " to " << ROBOT_GRIPPER+"_gripper");
                // Get transformation from base to left camera (tilt axes)
                try
                {
                    tf::StampedTransform echo_transform;
                    tf_.lookupTransform(ROBOT_BASE, ROBOT_GRIPPER+"_gripper", ros::Time(), echo_transform);
                    std::cout.precision(3);
                    std::cout.setf(std::ios::fixed,std::ios::floatfield);
                    std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
                    tf::Quaternion q = echo_transform.getRotation();
                    tf::Vector3 v = echo_transform.getOrigin();
                    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
                    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                              << q.getZ() << ", " << q.getW() << "]" << std::endl;

                    Mat rotK = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());

                    // Translation
                    Mat transK = Mat::zeros(3,1, CV_32F);
                    transK.at<float>(0) = v.getX();
                    transK.at<float>(1) = v.getY();
                    transK.at<float>(2) = v.getZ();

                    Mat rTemp;
                    Rodrigues(rotK, rTemp);

                    printMatrix(rTemp);
                    printMatrix(transK);

                    rvecs_rb2gripperL.push_back(rTemp);
                    tvecs_rb2gripperL.push_back(transK);
                    rvecs_rb2gripperR.push_back(rTemp);
                    tvecs_rb2gripperR.push_back(transK);

                }
                catch(tf::TransformException& ex)
                {
                    std::cout << "Failure at "<< ros::Time::now() << std::endl;
                    std::cout << "Exception thrown:" << ex.what()<< std::endl;
                    std::cout << "The current list of frames is:" <<std::endl;
                    std::cout << tf_.allFramesAsString() << std::endl;
                }

                ROS_INFO_STREAM("Transform from: " << "base_link" << " to " << ROBOT_GRIPPER+"_gripper");
                // Get transformation from base to left camera (tilt axes)
                try
                {
                    tf::StampedTransform echo_transform;
                    // Here base_link in order to debug results!!
                    tf_.lookupTransform("base_link", ROBOT_GRIPPER+"_ee", ros::Time(), echo_transform);
                    std::cout.precision(3);
                    std::cout.setf(std::ios::fixed,std::ios::floatfield);
                    std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
                    tf::Quaternion q = echo_transform.getRotation();
                    tf::Vector3 v = echo_transform.getOrigin();
                    double yaw, pitch, roll;
                    echo_transform.getBasis().getRPY(roll, pitch, yaw);
                    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
                    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                              << q.getZ() << ", " << q.getW() << "]" << std::endl;

                    savePose(v.getX(), v.getY(), v.getZ(), q.getX(), q.getY(), q.getZ(), q.getW(), yaw, pitch, roll);

                }
                catch(tf::TransformException& ex)
                {
                    std::cout << "Failure at "<< ros::Time::now() << std::endl;
                    std::cout << "Exception thrown:" << ex.what()<< std::endl;
                    std::cout << "The current list of frames is:" <<std::endl;
                    std::cout << tf_.allFramesAsString() << std::endl;
                }

                ROS_INFO_STREAM("Number of images processed: " << c_.nrFrames);
                c_.nrFrames++;

                // Capture next images
                return true;
            }
            else
            {
                ROS_INFO("Images were not good... capturing new ones!");
                return false;
            }
        }
        else
        {
            // At least totalFrames images have to be captured
            if( (int)imagePointsL.size() > totalFrames-1 && (int)imagePointsR.size() > totalFrames-1 )
            {

                ROS_INFO_STREAM("Calibrating left camera. Total number of good images: " << (int)imagePointsL.size());

                vector<vector<Point2f> > stereo_imgPointsL, stereo_imgPointsR;
                stereo_imgPointsL = imagePointsL;
                stereo_imgPointsR = imagePointsR;

                c_.imageSize = imL.size();
                c_.rvecs_robot.resize(0);
                c_.tvecs_robot.resize(0);
                c_.rvecs_robot = rvecs_rb2gripperL;
                c_.tvecs_robot = tvecs_rb2gripperL;
                bool cam1 = c_.runCalibrationAndSave(cameraMatrixL, distCoeffsL, imagePointsL, leftCam);
                rvecsL_cam = c_.rvecs;
                tvecsL_cam = c_.tvecs;
                rvecs_rb2gripperL.clear();
                tvecs_rb2gripperL.clear();
                rvecs_rb2gripperL = c_.rvecs_robot;
                tvecs_rb2gripperL = c_.tvecs_robot;
                imagePointsL.clear();
                imagePointsL = c_.cal_2Dpoints;

                ROS_INFO_STREAM("Calibrating right camera. Total number of good images: " << (int)imagePointsR.size());
                c_.imageSize = imR.size();
                c_.rvecs_robot.resize(0);
                c_.tvecs_robot.resize(0);
                c_.rvecs_robot = rvecs_rb2gripperR;
                c_.tvecs_robot = tvecs_rb2gripperR;
                bool cam2 = c_.runCalibrationAndSave(cameraMatrixR, distCoeffsR, imagePointsR, rightCam);
                rvecsR_cam = c_.rvecs;
                tvecsR_cam = c_.tvecs;
                rvecs_rb2gripperR.clear();
                tvecs_rb2gripperR.clear();
                rvecs_rb2gripperR = c_.rvecs_robot;
                tvecs_rb2gripperR = c_.tvecs_robot;
                imagePointsR.clear();
                imagePointsR = c_.cal_2Dpoints;

                if(cam1 && cam2)
                {
                    ROS_INFO_STREAM("Running stereo calibration...");
                    Mat F_stereo, P_right, P_left, Stereo_mat;
                    hconcat(cameraMatrixL,Mat::zeros(3,1,CV_64F), P_left);
                    bool st = c_.runStereoCalibrationAndSave(cameraMatrixL, cameraMatrixR, distCoeffsL, distCoeffsR, F_stereo, P_right, Stereo_mat, stereo_imgPointsL, stereo_imgPointsR);
                    H_stereo = Stereo_mat.clone();

                    if(c_.calibTarget == c_.opencvStr)
                    {
                        //runBA(imagePointsL, cameraMatrixL, rvecsL_cam, tvecsL_cam);
                        //runBA(imagePointsR, cameraMatrixR, rvecsR_cam, tvecsR_cam);
                    }

                    if(st)
                    {
                        ROS_INFO("Saving camera calibration parameters using SetCameraInfo service");

                        c_.saveRobotPoses("robot_poses.py", bigMatQ);

                        rh_cameras::SetCameraInfo srvL, srvR;
                        c_.nrFrames = 1;
                        // LEFT CAMERA
                        srvL.request.camera_info.header.frame_id = "left_camera";
                        srvL.request.camera_info.width = c_.imageSize.width;
                        srvL.request.camera_info.height = c_.imageSize.height;
                        for(int i = 0; i < distCoeffsL.rows; i++)
                        {
                            srvL.request.camera_info.D.push_back(distCoeffsL.at<double>(i));
                            if(i < P_left.rows)
                            {
                                for(int j = 0 ; j < P_left.cols; j++)
                                {
                                    if(j < cameraMatrixL.cols)
                                    {
                                        srvL.request.camera_info.K[3*i+j] = cameraMatrixL.at<double>(i,j);
                                        //srvL.request.camera_info.F.push_back(F_stereo.at<double>(i,j));
                                    }
                                    srvL.request.camera_info.P[4*i+j] = P_left.at<double>(i,j);
                                }
                            }
                        }

                        // RIGHT CAMERA
                        srvR.request.camera_info.header.frame_id = "right_camera";
                        srvR.request.camera_info.width = c_.imageSize.width;
                        srvR.request.camera_info.height = c_.imageSize.height;
                        for(int i = 0; i < distCoeffsR.rows; i++)
                        {
                            srvR.request.camera_info.D.push_back(distCoeffsR.at<double>(i));
                            if(i < P_right.rows)
                            {
                                for(int j = 0 ; j < P_right.cols; j++)
                                {
                                    if(j < cameraMatrixR.cols)
                                    {
                                        srvR.request.camera_info.K[3*i+j] = cameraMatrixR.at<double>(i,j);
                                        //srvR.request.camera_info.F.push_back(F_stereo.at<double>(i,j));
                                    }
                                    srvR.request.camera_info.P[4*i+j] = P_right.at<double>(i,j);
                                }
                            }
                        }

                        // *****************
                        ROS_INFO("Calibration from calibration target to robot gripper (left camera)");
                        // false so the transformation is from camera to calibration target
                        if(!(hE_robotL.loadParameters(rvecs_rb2gripperL, tvecs_rb2gripperL, rvecsL_cam, tvecsL_cam, false)))
                            ROS_FATAL("Wrong size in rotation and translation vectors for hand eye calibration");

                        // Perform calibration!
                        hE_robotL.calibrate();
                        Mat robotMat_left = hE_robotL.gHc.clone();

                        ROS_INFO("Calibration from calibration target to robot gripper (right camera)");

                        // false so the transformation is from camera to calibration target
                        if(!(hE_robotR.loadParameters(rvecs_rb2gripperR, tvecs_rb2gripperR, rvecsR_cam, tvecsR_cam, false)))
                            ROS_FATAL("Wrong size in rotation and translation vectors for hand eye calibration");

                        // Perform calibration!
                        hE_robotR.calibrate();
                        Mat robotMat_right = hE_robotR.gHc.clone();

                        Mat rb2camL = hE_robotL.bHg[1] * robotMat_left * hE_robotL.tHc[1].inv();
                        Mat rb2camR = hE_robotR.bHg[1] * robotMat_right * hE_robotR.tHc[1].inv();

                        ROS_INFO("Robot base to camera (left and right):");
                        printMatrix(rb2camL);
                        printMatrix(rb2camR);
                        
                        // Save stuff
                        if(c_.save_mode >= all)
                        {
                            c_.saveTransformations("robotBase_to_gripper_L.xml", "base2target_L", rvecs_rb2gripperL, tvecs_rb2gripperL);
                            c_.saveTransformations("robotBase_to_gripper_R.xml", "base2target_R", rvecs_rb2gripperR, tvecs_rb2gripperR);
                            c_.saveTransformations("camL_to_target.xml", "camL2target", rvecsL_cam, tvecsL_cam);
                            c_.saveTransformations("camR_to_target.xml", "camR2target", rvecsR_cam, tvecsR_cam);
                        }
                        c_.saveHandEyeTransform("gripper2target.xml", robotMat_left, robotMat_right, "gripperHtarget");
                        saveCalibration(rb2camL.inv(), "left_camera"); //OK!
                        saveCalibration(rb2camR.inv(), "right_camera"); //OK!

                        // *****************

                        // Save stuff
                        if (!camera_infoL.call(srvL))
                        {
                            ROS_ERROR("Failed to call service camera_infoL");
                            return false;
                        }
                        if (!camera_infoR.call(srvR))
                        {
                            ROS_ERROR("Failed to call service camera_infoR");
                            return false;
                        }

                        imagePointsL.resize(0);
                        imagePointsR.resize(0);
                        ROS_INFO("Camera calibration done!");
                        return true;
                    }
                    else
                    {
                        ROS_ERROR("Error during stereo camera calibration!!");
                        return false;
                    }
                }
                else
                {
                    ROS_ERROR("Error during camera calibration!! [Left: %d, Right %d]", cam1, cam2);
                    return false;
                }
            }
            else
            {
                ROS_INFO("At least %i images must be captured", totalFrames);
                // Capture next images
                //capture_images();
                return false;
            }
        }
    }

    int modeHandEye(const Mat& imL, const Mat& imR, bool calibQ)
    {
        double res_pL, res_tL, res_pR, res_tR;
        double pL, tL, pR, tR;
        vector<Point2f> pointBufL, pointBufR;
        bool found1, found2;

        if(!calibQ)
        {
            Mat tempImL = imL.clone();
            Mat tempImR = imR.clone();

            if(c_.calibTarget == c_.alvarStr)
            {
                found1 = detectMarker(cv_ptrL, cv_ptrR);
                found2 = found1;
                int idx = (int)imagePointsL.size();
                pointBufL = imagePointsL[idx-1];
                pointBufR = imagePointsR[idx-1];

            }else if(c_.calibTarget == c_.opencvStr)
            {
                // Find corners
                ROS_INFO("Finding chessboard corners");
                found_chees = false;
                Mat leftOut = findCorners(imL);
                found1 = found_chees;
                pointBufL = pointBuf;

                found_chees = false;
                Mat rightOut = findCorners(imR);
                pointBufR = pointBuf;
                found2 = found_chees;
                
                Mat dispImgL, dispImgR;
                leftOut.copyTo(dispImgL);
                rightOut.copyTo(dispImgR);
                
                line(dispImgL, Point(leftOut.cols/2.0, 0), Point(leftOut.cols/2.0, leftOut.rows), Scalar( 0, 255, 0), 5);
                line(dispImgL, Point(0, leftOut.rows/2.0), Point(leftOut.cols, leftOut.rows/2.0), Scalar( 0, 255, 0), 5);
                line(dispImgR, Point(leftOut.cols/2.0, 0), Point(leftOut.cols/2.0, leftOut.rows), Scalar( 0, 255, 0), 5);
                line(dispImgR, Point(0, leftOut.rows/2.0), Point(leftOut.cols, leftOut.rows/2.0), Scalar( 0, 255, 0), 5);

                cv::imshow(WINDOW_LEFT, dispImgL);
                cv::imshow(WINDOW_RIGHT, dispImgR);
            }

            rh_ptu::GetPtuInfo srvPTU;

            srvPTU.request.get_info = true;
            ROS_INFO("Calling PTU info status!");
            if (!ptu_infoStatus.call(srvPTU))
            {
                ROS_ERROR("Failed to call service ptu_infoStatus");
                return false;
            }
            ROS_INFO("Done!");

            pL = srvPTU.response.ptu.position[0];
            tL = srvPTU.response.ptu.position[1];
            pR = srvPTU.response.ptu.position[2];
            tR = srvPTU.response.ptu.position[3];

            //ROS_INFO_STREAM("found1: " << found1 << " found2: " << found2);

            if(found1 && found2)
            {
                if(c_.save_mode >= images)
                    saveImages("_he_L.tif", "_he_R.tif", tempImL, tempImR);

                ROS_INFO("Computing extrinsic parameters...");
                c_.runExtrinsic(cameraMatrixL, distCoeffsL, pointBufL);

                rvecsL_he.push_back(c_.rvecEx.clone());
                tvecsL_he.push_back(c_.tvecEx.clone());
                pointsL_handEye.push_back(pointBufL);

                c_.runExtrinsic(cameraMatrixR, distCoeffsR, pointBufR);

                rvecsR_he.push_back(c_.rvecEx.clone());
                tvecsR_he.push_back(c_.tvecEx.clone());
                pointsR_handEye.push_back(pointBufR);

                // Get PTU values
                nh_.getParam(PAN_L_RES, res_pL);
                nh_.getParam(TILT_L_RES, res_tL);
                nh_.getParam(PAN_R_RES, res_pR);
                nh_.getParam(TILT_R_RES, res_tR);

                // Store ptu values for current image (in radians)
                Point2f pt;
                pt.x = pL;
                pt.y = tL;
                ptuL.push_back(pt);
                pt.x = pR;
                pt.y = tR;
                ptuR.push_back(pt);

                ROS_INFO_STREAM("Number of images processed: " << c_.nrFrames);

                ROS_INFO_STREAM("Transform from: " << RH_BASE_LEFT << " to " << LEFT_CAMERA);
                // Get transformation from base to left camera (tilt axes)
                try
                {
                    tf::StampedTransform echo_transform;
                    tf_.lookupTransform(RH_BASE_LEFT, LEFT_CAMERA, ros::Time(), echo_transform);
                    std::cout.precision(3);
                    std::cout.setf(std::ios::fixed,std::ios::floatfield);
                    std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
                    tf::Quaternion q = echo_transform.getRotation();
                    tf::Vector3 v = echo_transform.getOrigin();
                    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
                    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                              << q.getZ() << ", " << q.getW() << "]" << std::endl;

                    Mat rotK = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());

                    // Translation
                    Mat transK = Mat::zeros(3,1, CV_32F);
                    transK.at<float>(0) = v.getX();
                    transK.at<float>(1) = v.getY();
                    transK.at<float>(2) = v.getZ();

                    Mat rTemp;
                    Rodrigues(rotK, rTemp);
                    rvecsL_tilt2world.push_back(rTemp);
                    tvecsL_tilt2world.push_back(transK);


                }
                catch(tf::TransformException& ex)
                {
                    std::cout << "Failure at "<< ros::Time::now() << std::endl;
                    std::cout << "Exception thrown:" << ex.what()<< std::endl;
                    std::cout << "The current list of frames is:" <<std::endl;
                    std::cout << tf_.allFramesAsString() << std::endl;
                }

                ROS_INFO_STREAM("Transform from: " << RH_BASE_RIGHT << " to " << RIGHT_CAMERA);
                // Get transformation from base to right camera (tilt axes)
                try
                {
                    tf::StampedTransform echo_transform;
                    tf_.lookupTransform(RH_BASE_RIGHT, RIGHT_CAMERA, ros::Time(), echo_transform);
                    std::cout.precision(3);
                    std::cout.setf(std::ios::fixed,std::ios::floatfield);
                    std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
                    tf::Quaternion q = echo_transform.getRotation();
                    tf::Vector3 v = echo_transform.getOrigin();
                    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
                    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                              << q.getZ() << ", " << q.getW() << "]" << std::endl;

                    Mat rotK = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());

                    // Translation
                    Mat transK = Mat::zeros(3,1, CV_32F);
                    transK.at<float>(0) = v.getX();
                    transK.at<float>(1) = v.getY();
                    transK.at<float>(2) = v.getZ();

                    Mat rTemp;
                    Rodrigues(rotK, rTemp);
                    rvecsR_tilt2world.push_back(rTemp);
                    tvecsR_tilt2world.push_back(transK);

                }
                catch(tf::TransformException& ex)
                {
                    std::cout << "Failure at "<< ros::Time::now() << std::endl;
                    std::cout << "Exception thrown:" << ex.what()<< std::endl;
                    std::cout << "The current list of frames is:" <<std::endl;
                    std::cout << tf_.allFramesAsString() << std::endl;

                }

                ROS_WARN_STREAM("Number of images processed: " << c_.nrFrames);
                c_.nrFrames++;

                ROS_WARN("Choose a point in %s with the left button", WINDOW_LEFT);
                ROS_WARN("or right click to finish acquiring images");
                ROS_WARN("Waiting for user input...");
                for(;;)
                {
                    printf("%d\b", key);

                    if(debugQ && countImgPtu < noImgPtu)
                    {
                    	key = 0;
                    	ptX = 100;
                    	ptY = 100;
                    	countImgPtu += 1;
                    }
                    else if(debugQ && countImgPtu == noImgPtu)
                    {
                    	key = 1;
                    	ptX = 100;
                    	ptY = 100;
                    }

                    if (key == 0){

                        movePTU(res_pL, res_tL, res_pR, res_tR, pL, tL, pR, tR, imL, imR);
                        // Images will be captured after moving the PTUs (RHptu does this)
                        return 0;
                    }
                    else if(key == 1)
                    {
                        return 1;
                    }
                    else
                        key = 3;
                }

            }
            else
            {
                ROS_ERROR("Discarding image!");

                ROS_WARN("Choose a point in %s with the left button", WINDOW_LEFT);
                ROS_WARN("or right click to finish acquiring images");
                ROS_WARN("Waiting for user input...");
                for(;;)
                {
                    printf("%d\b", key);

                    if(debugQ && countImgPtu < noImgPtu)
                    {
                    	key = 0;
                    	ptX = 100;
                    	ptY = 100;
                    	countImgPtu += 1;
                    }
                    else if(debugQ && countImgPtu == noImgPtu)
                    {
                    	key = 1;
                    	ptX = 100;
                    	ptY = 100;
                    }

                    if (key == 0){

                        movePTU(res_pL, res_tL, res_pR, res_tR, pL, tL, pR, tR, imL, imR);
                        // Images will be captured after moving the PTUs (RHptu does this)
                        return 0;
                    }
                    else if(key == 1)
                    {
                        // Run bundle adjustment to optimise pose
                        if(c_.calibTarget == c_.opencvStr)
                        {
                            //runBA(pointsL_handEye, cameraMatrixL, rvecsL_he, tvecsL_he);
                            //runBA(pointsR_handEye, cameraMatrixR, rvecsR_he, tvecsR_he);
                        }

                        return 1;
                    }
                    else
                        key = 3;
                }

                return -1;
            }

            return 0;
        }
        else
        {
            // HE LEFT CAMERA
            ROS_INFO("Hand Eye calibration for the Left camera!");
            if(!(hE_left.loadParameters(rvecsL_tilt2world, tvecsL_tilt2world, rvecsL_he, tvecsL_he)))
                ROS_FATAL("Wrong size in rotation and translation vectors for hand eye calibration");

            hE_left.calibrate();
            Mat heMat_left = hE_left.gHc.clone();

            // HE RIGHT CAMERA
            ROS_INFO("Hand Eye calibration for the Right camera!");
            if(!(hE_right.loadParameters(rvecsR_tilt2world, tvecsR_tilt2world, rvecsR_he, tvecsR_he)))
                ROS_FATAL("Wrong size in rotation and translation vectors for hand eye calibration");

            hE_right.calibrate();
            Mat heMat_right = hE_right.gHc.clone();

            // ************************************************************
            // Get transformation from base_link to ROBOT_BASE
            Mat r2b;
            try
            {
                tf::StampedTransform echo_transform;
                tf_.lookupTransform("base_link", ROBOT_BASE, ros::Time(), echo_transform);
                tf::Quaternion q = echo_transform.getRotation();
                tf::Vector3 v = echo_transform.getOrigin();
                double yaw, pitch, roll;
                echo_transform.getBasis().getRPY(roll, pitch, yaw);

                r2b = Mat::eye(4,4, CV_32F);
                // Rotation
                Mat rotK = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());
                r2b.at<float>(0,0) = rotK.at<float>(0,0);
                r2b.at<float>(0,1) = rotK.at<float>(0,1);
                r2b.at<float>(0,2) = rotK.at<float>(0,2);
                r2b.at<float>(1,0) = rotK.at<float>(1,0);
                r2b.at<float>(1,1) = rotK.at<float>(1,1);
                r2b.at<float>(1,2) = rotK.at<float>(1,2);
                r2b.at<float>(2,0) = rotK.at<float>(2,0);
                r2b.at<float>(2,1) = rotK.at<float>(2,1);
                r2b.at<float>(2,2) = rotK.at<float>(2,2);
                // Translation
                r2b.at<float>(0,3) = v.getX();
                r2b.at<float>(1,3) = v.getY();
                r2b.at<float>(2,3) = v.getZ();

            }
            catch(tf::TransformException& ex)
            {
                std::cout << "Failure at "<< ros::Time::now() << std::endl;
                std::cout << "Exception thrown:" << ex.what()<< std::endl;
                std::cout << "The current list of frames is:" <<std::endl;
                std::cout << tf_.allFramesAsString() << std::endl;
            }

	    	ROS_INFO("Base_link transform to ROBOT_BASE");
            printMatrix(r2b);

            // BASELINE BTW CAMERAS
            ROS_INFO("Computing optimal baseline");
            vector<Mat> r2PTUL, r2PTUR, rb2PTUL, rb2PTUR;
            r2PTUL.resize(0);
            r2PTUR.resize(0);
            rb2PTUL.resize(0);
            rb2PTUR.resize(0);
            //ROS_INFO("Left...");
            //printMatrix(hE_left.bHg[1]);
            for(int i = 0; i < hE_robotL.M; i++)
            {
                Mat temp = hE_robotL.bHct[i];
                //ROS_INFO_STREAM("Left: " << i);
                //printMatrix(r2b * temp);
                rb2PTUL.push_back(temp * heMat_left.inv() * hE_left.bHg[1].inv());
                r2PTUL.push_back(r2b * temp * heMat_left.inv() * hE_left.bHg[1].inv());
                //printMatrix(r2PTUL[i]);
            }

            //ROS_INFO("Right...");
            //printMatrix(hE_right.bHg[1]);
            for(int i = 0; i < hE_robotR.M; i++)
            {
            	Mat temp = hE_robotR.bHct[i];
            	//ROS_INFO_STREAM("Right: " << i);
                //printMatrix(r2b * temp);
                rb2PTUR.push_back(temp * heMat_right.inv() * hE_right.bHg[1].inv());
                r2PTUR.push_back(r2b * temp * heMat_right.inv() * hE_right.bHg[1].inv());
                //printMatrix(r2PTUR[i]);
            }

            Mat r2PTUL_avg = hE_robotL.avgTransformation(r2PTUL, false);
            Mat r2PTUR_avg = hE_robotR.avgTransformation(r2PTUR, false);
            ROS_INFO("r2PTUL_avg");
            printMatrix(r2PTUL_avg);
			ROS_INFO("r2PTUR_avg");
            printMatrix(r2PTUR_avg);

            Mat lpHrp = r2PTUL_avg.inv() * r2PTUR_avg;
            ROS_INFO("lpHrp matrix:");
            printMatrix(lpHrp, false);

            //lpHrp_avg_mat = hE_left.bHc[0] * H_stereo * hE_right.bHc[0].inv();
            //ROS_INFO("Baseline matrix (stereo method):");
            //printMatrix(lpHrp_avg_mat, false);
            Mat lpHrp_avg = Mat::zeros(1,6, CV_32F);
            Mat lpHrp_avg_mat = Mat::zeros(4,4, CV_32F);

            // ROBOT_BASE TO LEFT CAMERA
            ROS_INFO_STREAM("Computing " << ROBOT_BASE << " to left camera transformation");
            vector<Mat> rbHcl;
            rbHcl.resize(0);
            for(int i = 0; i < hE_robotL.M; i++)
            {
                Mat rbHct = hE_robotL.bHc[i];
                rbHcl.push_back(rbHct * hE_robotL.tHc[i].inv());
            }

            Mat rbHcl_avg = hE_robotL.avgTransformation(rbHcl, false);
            printMatrix(r2b * rbHcl_avg, true);

            // BASE LINK TO RIGHT CAMERA
            ROS_INFO_STREAM("Computing " << ROBOT_BASE << " to right camera transformation");
            vector<Mat> rbHcr;
            rbHcr.resize(0);
            for(int i = 0; i < hE_robotR.M; i++)
            {
                Mat rbHct = hE_robotR.bHc[i];
                rbHcr.push_back(rbHct * hE_robotR.tHc[i].inv());
            }

            Mat rbHcr_avg = hE_robotR.avgTransformation(rbHcr, false);
            printMatrix(r2b * rbHcr_avg, true);
            
            // From ptu base to robot base
            ROS_WARN("Computing robot base to ptu transformation...");
            Mat rbHptu_left = hE_robotL.avgTransformation(rb2PTUL, false);
            Mat rbHptu_right = hE_robotR.avgTransformation(rb2PTUR, false);

            ROS_INFO("Left");
            printMatrix(rbHptu_left, false);
            printMatrix(r2b * rbHptu_left, false);
            ROS_INFO("Right");
            printMatrix(rbHptu_right, false);
            printMatrix(r2b * rbHptu_right, false);
            
            // ************************************************************
            // ************************************************************
            // ************************************************************
            // Save stuff
            if(c_.save_mode >= all)
            {
                c_.saveCalibPoints("pointsL_he.xml", pointsL_handEye); //Left
                c_.saveCalibPoints("pointsR_he.xml", pointsR_handEye); //Right
                c_.savePTUPositions("ptu_positions.xml", ptuL, ptuR);
                c_.saveTransformations("camL_to_target_he.xml", "camL2target", rvecsL_he, tvecsL_he);
                c_.saveTransformations("camR_to_target_he.xml", "camR2target", rvecsR_he, tvecsR_he);
                c_.saveTransformations("ptu_base_to_tilt_left_he.xml", "baseL2tiltL", rvecsL_tilt2world, tvecsL_tilt2world);
                c_.saveTransformations("ptu_base_to_tilt_right_he.xml", "baseR2tiltR", rvecsR_tilt2world, tvecsR_tilt2world);
            }
            c_.saveHandEyeTransform("ptu2camera.xml", heMat_left, heMat_right, lpHrp_avg, "gripperHcamera");
            c_.calibTransform("ptuL2ptuR.xml", "left_base_ptuHright_base_ptu", lpHrp_avg_mat);

            //saveCalibration(heMat_left.inv(), "left_camera"); //OK!
            //saveCalibration(heMat_right.inv(), "right_camera"); //OK!
            saveCalibration(lpHrp.inv(), "right_base_ptu"); //OK!

            if(c_.save_mode >= all)
            {
                c_.saveTransformations("robotBase_to_camL_all.xml", "robotBase2camL", rbHcl);
                c_.saveTransformations("robotBase_to_camR_all.xml", "robotBase2camR", rbHcr);
            }
            c_.calibTransform("robotBase2camLAvg.xml", "robotBase2camL", rbHcl_avg);
            c_.calibTransform("robotBase2camRAvg.xml", "robotBase2camR", rbHcr_avg);

            c_.saveHandEyeTransform("ptu2robotBase.xml", rbHptu_left, rbHptu_right, "robotBase2ptu");
            saveCalibration(rbHptu_left.inv(), "left_base_ptu");

            ROS_INFO("DONE!!");
            return 0;
        }
    }

    void saveCalibration(cv::Mat cal, string child_frame)
    {
        string calFile;

        calFile = ros::package::getPath("clopema_description") + "/calibration_" + getenv("CLOPEMA_PARTNER") + "/" + child_frame + ".calib";

        std::remove((calFile).c_str());
        std::ofstream file;
        file.open((calFile).c_str(), ios::app);
        file << "2" << endl;

        file << child_frame.c_str() << endl << endl;

        for (int i=0; i<cal.rows; i++)
        {
            for (int j=0; j<cal.cols; j++)
                file << cal.at<float>(i,j) << "  ";

            file << endl;
        }

        file.close();
        ROS_INFO("Calibration saved to %s: ",calFile.c_str());
    }

    void runBA(vector<vector<Point2f> > imagePoints, Mat&  cameraMatrix_in, vector<Mat>& rvecs, vector<Mat>& tvecs)
    {
        vector<cv::Point3d> points3D;
        vector<vector<Point2d> > pointsImg;
        vector<vector<int > > visibility;
        vector<Mat > cameraMatrix, distCoeffs, R, T;

        vector<Point3f> corners;
        c_.calcBoardCornerPositions(corners);

        ROS_INFO_STREAM("size of image points: " << imagePoints.size());

        int NPOINTS = corners.size(); 	// number of 3d points
        int NCAMS = imagePoints.size(); 	// number of cameras

        ROS_INFO_STREAM("Number of points for BA: " << NPOINTS);

        // fill 3d points
        points3D.resize(NPOINTS);
        for(int i = 0; i < NPOINTS; i++)
        {
            points3D[i].x = (double)corners[i].x;
            points3D[i].y = (double)corners[i].y;
            points3D[i].z = (double)corners[i].z;
        }


        // fill 2d image points
        pointsImg.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
        {
            pointsImg[i].resize(NPOINTS);
            for(int j = 0; j < NPOINTS; j++)
            {
                pointsImg[i][j].x = (double)imagePoints[i][j].x;
                pointsImg[i][j].y = (double)imagePoints[i][j].y;
            }
        }

        // fill visibility (all points are visible)
        visibility.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)  {
            visibility[i].resize(NPOINTS);
            for(int j=0; j<NPOINTS; j++) visibility[i][j]=1;
        }

        // fill camera intrinsics
        cameraMatrix.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
            cameraMatrix[i] = cameraMatrix_in.clone();

        // fill distortion (assume no distortion)
        distCoeffs.resize(NCAMS);
        for(int i=0; i<NCAMS; i++) distCoeffs[i] = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));

        //        // fill rotation
        //        R.resize(NCAMS);
        //        for(int i=0; i<NCAMS; i++)
        //            R[i] = rvecs[i].clone();

        //        // fill translation (3 units away in camera Z axis)
        //        T.resize(NCAMS);
        //        for(int i=0; i<NCAMS; i++)
        //            T[i] = tvecs[i].clone();

        cvsba::Sba sba;

        cvsba::Sba::Params params;
        params.type = cvsba::Sba::MOTION;
        //params.iterations = 1000;
        //params.minError = 1e-16;
        params.fixedIntrinsics = 5;
        params.fixedDistortion = 5;
        sba.setParams(params);

        sba.run(points3D,  pointsImg,  visibility,  cameraMatrix,  rvecs,  tvecs, distCoeffs);

        ROS_ERROR_STREAM("Initial error=" << sba.getInitialReprjError() << ". Final error=" << sba.getFinalReprjError());

        //        ROS_ERROR_STREAM("Camera");
        //        printMatrix(cameraMatrix_in, false);
        //        printMatrix(cameraMatrix[0], false);
        //        printMatrix(cameraMatrix[1], false);

        //        ROS_ERROR_STREAM("Rotation");
        //        for(int i = 0; i < NCAMS; i ++)
        //        {
        //            printMatrix(rvecs[i], false);
        //            printMatrix(R[i], false);
        //        }

        //        ROS_ERROR_STREAM("translation");
        //        for(int i = 0; i < NCAMS; i ++)
        //        {
        //            printMatrix(tvecs[i], false);
        //            printMatrix(T[i], false);
        //        }

        //        ROS_ERROR_STREAM("3D points");
        //        ROS_INFO_STREAM(corners[0].x << " " << corners[0].y  << " " << corners[0].z);
        //        ROS_INFO_STREAM(points3D[0].x << " " << points3D[0].y  << " " << points3D[0].z);
        //        ROS_INFO_STREAM(corners[1].x << " " << corners[1].y  << " " << corners[1].z);
        //        ROS_INFO_STREAM(points3D[1].x << " " << points3D[1].y  << " " << points3D[1].z);

        //        ROS_ERROR_STREAM("2D points");
        //        ROS_INFO_STREAM(imagePoints[0][0].x << " " << imagePoints[0][0].y);
        //        ROS_INFO_STREAM(pointsImg[0][0].x << " " << pointsImg[0][0].y);
        //        ROS_INFO_STREAM(imagePoints[1][0].x << " " << imagePoints[1][0].y);
        //        ROS_INFO_STREAM(pointsImg[1][0].x << " " << pointsImg[1][0].y);


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

    bool detectMarker(cv_bridge::CvImagePtr& cv_ptrL, cv_bridge::CvImagePtr& cv_ptrR)
    {
        ROS_INFO("Detecting target...");

        rh_integration::MarkerDetection srv;
        sensor_msgs::Image msgL, msgR;
        cv_ptrL->toImageMsg(msgL);
        cv_ptrR->toImageMsg(msgR);
        srv.request.leftIm = msgL;
        srv.request.rightIm = msgR;

        if(!getMarkerData_srv.call(srv)){
            ROS_ERROR_STREAM("Error while detecting target, get new images!");
            return false;
        }else
        {
            readMarkerResponse(srv);
            ROS_INFO("Target detected!");
            //nrFrames += 1;
            // Displaying Images!
            cv::imshow(WINDOW_LEFT, cornersImgLeft);
            cv::imshow(WINDOW_RIGHT, cornersImgRight);
        }

        return true;
    }

    // Get service marker detection stuff
    void readMarkerResponse(rh_integration::MarkerDetection srv)
    {
        cv_bridge::CvImagePtr cv_ptrLtmp, cv_ptrRtmp;
        try
        {
            cv_ptrLtmp = cv_bridge::toCvCopy(srv.response.dispImL, enc::RGB8);
            cv_ptrRtmp = cv_bridge::toCvCopy(srv.response.dispImR, enc::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert to 'rgb8'.");
            return;
        }

        cornersImgLeft = cv_ptrLtmp->image.clone();
        cornersImgRight = cv_ptrRtmp->image.clone();

        std::vector<float> cornersLeftFromSrv = srv.response.cornersLeft;
        std::vector<float> cornersRightFromSrv = srv.response.cornersRight;
        int numCornersL = cornersLeftFromSrv.size()/2;
        int numCornersR = cornersRightFromSrv.size()/2;
        std::vector<Point2f> cornersLeft, cornersRight;
        if(numCornersL == numCornersR)
        {
            std::vector<float>::const_iterator itL = cornersLeftFromSrv.begin();
            std::vector<float>::const_iterator itR = cornersRightFromSrv.begin();
            for(int i = 0; i < numCornersL; i++)
            {
                Point2f ptL;
                ptL.x = *itL; itL++;
                ptL.y = *itL; itL++;
                cornersLeft.push_back(ptL);
                Point2f ptR;
                ptR.x = *itR; itR++;
                ptR.y = *itR; itR++;
                cornersRight.push_back(ptR);
            }

            //drawChessboardCorners(cornersImgLeft, calib_.boardSize, Mat(cornersLeft), true);
            //drawChessboardCorners(cornersImgRight, calib_.boardSize, Mat(cornersRight), true);

            imagePointsL.push_back(cornersLeft);
            imagePointsR.push_back(cornersRight);

        }
    }

    Mat findCorners(const Mat& view)
    {
        Mat view_scaled;
        if(c_.scaleInput > 1.0)
        {
            ROS_INFO_STREAM("Scale factor: " << c_.scaleInput);
            ROS_INFO_STREAM("Size: " << view.cols/c_.scaleInput << " " << view.rows/c_.scaleInput);
            resize(view, view_scaled, Size(view.cols/c_.scaleInput, view.rows/c_.scaleInput),
                   0, 0, cv::INTER_CUBIC);
        }
        else
            view_scaled = view;

        pointBuf.resize(0);
        found_chees = findChessboardCorners( view_scaled, c_.boardSize, pointBuf,
                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(c_.scaleInput > 1.0)
        {
            for(int i = 0; i < (int)pointBuf.size(); i++)
            {
                pointBuf[i].x *= c_.scaleInput;
                pointBuf[i].y *= c_.scaleInput;
            }
        }

        if (found_chees) // If done with success,
        {
            ROS_INFO("Found corners!");
            // improve the found corners' coordinate accuracy for chessboard
            Mat viewGray;
            cvtColor(view, viewGray, CV_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                          Size(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                                      30, 0.01));

            // Draw the corners.
            drawChessboardCorners(view, c_.boardSize, Mat(pointBuf), found_chees);
        }
        else
            ROS_ERROR("No corners found...");

        return view;
    }

    void saveImages(string str1, string str2, const Mat& imL, const Mat& imR)
    {
        stringstream ss;
        ss << c_.nrFrames;

        string out_imageL = c_.imageOutputDir + ss.str() + str1;
        string out_imageR = c_.imageOutputDir + ss.str() + str2;

        ROS_INFO("Saving left image to: %s", out_imageL.c_str());
        ROS_INFO("Saving right image to: %s", out_imageR.c_str());

        imwrite(out_imageL, imL);
        imwrite(out_imageR, imR);

        ROS_INFO("Images saved!");
    }

    Mat getRotation(double x, double y, double z, double w)
    {
        Mat rotK = Mat::zeros(3,3,CV_32F);

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

        rotK.at<float>(0,0) = (float)m00;
        rotK.at<float>(0,1) = (float)m01;
        rotK.at<float>(0,2) = (float)m02;
        rotK.at<float>(1,0) = (float)m10;
        rotK.at<float>(1,1) = (float)m11;
        rotK.at<float>(1,2) = (float)m12;
        rotK.at<float>(2,0) = (float)m20;
        rotK.at<float>(2,1) = (float)m21;
        rotK.at<float>(2,2) = (float)m22;

        return rotK;
    }

    void movePTU(double res_pL, double res_tL, double res_pR, double res_tR,
                 double pL, double tL, double pR, double tR, const Mat& imL, const Mat& imR)
    {
        sensor_msgs::JointState js;

        if(res_pL == 0.0 || res_tL == 0.0 || res_pR == 0.0 || res_tR == 0)
        {
            nh_.getParam(PAN_L_RES, res_pL);
            nh_.getParam(TILT_L_RES, res_tL);
            nh_.getParam(PAN_R_RES, res_pR);
            nh_.getParam(TILT_R_RES, res_tR);
        }

        // Publish point
        js.header.stamp = ros::Time::now();
        js.name.push_back(LEFT_PAN);
        js.name.push_back(LEFT_TILT);
        js.name.push_back(RIGHT_PAN);
        js.name.push_back(RIGHT_TILT);
        js.position.resize(4,0.0);
        js.velocity.resize(4,0.0);

        //ROS_INFO("PTX: %f, PTY: %f", ptX, ptY);

        ptX = ptX - ((float)imL.cols / 2);
        ptY = ptY - ((float)imR.rows / 2);

        // Unit 1
        js.position[0] = (-1 * ptX * stpPerPixelX) + (pL/res_pL); // in radians
        js.velocity[0] = VELOCITY * res_pL;
        js.position[1] = (-1 * ptY * stpPerPixelY) + (tL/res_tL);
        js.velocity[1] = VELOCITY * res_tL;
        // Unit 2
        js.position[2] = (-1 * ptX * stpPerPixelX) + (pR/res_pR);
        js.velocity[2] = VELOCITY * res_pR;
        js.position[3] = (-1 * ptY * stpPerPixelY) + (tR/res_tR);
        js.velocity[3] = VELOCITY * res_tR;
        //ROS_INFO("P1: %f, T1: %f, P2: %f, T2: %f", js.position[0], js.position[1], js.position[2], js.position[3]);

        // Unit 1
        js.position[0] = js.position[0] * res_pL; // in radians
        js.position[1] = js.position[1] * res_tL;
        // Unit 2
        js.position[2] = js.position[2] * res_pR;
        js.position[3] = js.position[3] * res_tR;

        //ROS_INFO("P1: %f, T1: %f, P2: %f, T2: %f", js.position[0], js.position[1], js.position[2], js.position[3]);

        key = -1;

        // Service
        rh_ptu::MovePtu srv_movePTU;

        srv_movePTU.request.ptu = js;
        if (!move_ptuSrv.call(srv_movePTU))
        {
            ROS_ERROR("Failed to call service srv_movePTU!");
            return;
        }

    }

    void savePose(double x, double y, double z, double qx, double qy, double qz, double qw, double yaw, double pitch, double roll)
    {
        Mat m(1, 7, CV_32F);
        Mat mE(1, 7, CV_32F);
        
        // Position
        m.at<float>(0,0) = x;
        m.at<float>(0,1) = y;
        m.at<float>(0,2) = z;
        
        // Orientation (quaternion)
        m.at<float>(0,3) = qx;
        m.at<float>(0,4) = qy;
        m.at<float>(0,5) = qz;
        m.at<float>(0,6) = qw;
        
        bigMatQ.push_back(m);
        
        // Position
        mE.at<float>(0,0) = x;
        mE.at<float>(0,1) = y;
        mE.at<float>(0,2) = z;
        
        // Orientation (quaternion)
        mE.at<float>(0,3) = yaw;
        mE.at<float>(0,4) = pitch;
        mE.at<float>(0,5) = roll;
        
        bigMatE.push_back(mE);
        
        return;

    }

};

/* *************** MAIN PROGRAM *************** */
int main(int argc, char** argv)
{
    ros::init( argc, argv, "rh_calibration_automatic" );
    CmainCalibration cal_;

    while( ros::ok() )
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}



