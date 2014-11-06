#include <rh_calibration/RHcalibration.h>
#include <rh_calibration/parameters.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

// Parameter server
static const char AXIS[] = "/PTU/axis";

// Messages
static const char STATUS_IN[] = "/RH/calibration/new_position";

class CcalibrationYoung
{
public:
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    vector<Mat> rvecsL, tvecsL, rvecsR, tvecsR;
    vector<vector<Point2f> > imagePointsL, imagePointsR;

    int axis;

    typedef image_transport::SubscriberFilter ImageSubscriber;

    ImageSubscriber imL_sub_;
    ImageSubscriber imR_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoL_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoR_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> syncPolicy;

    message_filters::Synchronizer<syncPolicy> sync;
    
    ros::Publisher status_flag;

    Mat_<double> K1_, D1_, P1_;
    Mat_<double> K2_, D2_, P2_;

    CcalibrationYoung() : it_(nh_),
    imL_sub_(it_, CAM_SUB_LEFT, 5),
    imR_sub_(it_, CAM_SUB_RIGHT, 5),
    infoL_sub_(nh_, CAMERA_INFO_L, 5),
    infoR_sub_(nh_, CAMERA_INFO_R, 5),
    sync(syncPolicy(5), imL_sub_, imR_sub_, infoL_sub_, infoR_sub_)
    {   
        nh_.setParam(CAPTURE_MODE, 0); // Set full resolution

        if(nh_.hasParam(OUTPUT_IMAGE_DIR) && nh_.hasParam(OUTPUT_CALIB_DIR))
        {
            nh_.getParam(OUTPUT_IMAGE_DIR, imageOutputDir);
            nh_.getParam(OUTPUT_CALIB_DIR, calibOutputDir);

        }else{
            ROS_ERROR("\"%s\", \"%s\" and/or \"%s\" parameters are not set in the server", OUTPUT_IMAGE_DIR, OUTPUT_CALIB_DIR, MAX_ERROR_TH);
            nh_.shutdown();
            return;
        }

        double param;
        if(nh_.hasParam(WIDTH_MARKER) && nh_.hasParam(MARKER_SIZE_X) && nh_.hasParam(MARKER_SIZE_Y) 
            && nh_.hasParam(INPUT_SCALE))
        {
            nh_.getParam(WIDTH_MARKER,param);
            squareSize = (float)param;
            nh_.getParam(MARKER_SIZE_X,param);
            boardSize.width = (int)param;
            nh_.getParam(MARKER_SIZE_Y,param);
            boardSize.height = (int)param;
            nh_.getParam(INPUT_SCALE,param);
            scaleInput = (float)param;
        }else{
            ROS_ERROR("Check that \"%s\", \"%s\", \"%s\" and/or \"%s\" parameters are set in the server", WIDTH_MARKER, MARKER_SIZE_X, MARKER_SIZE_Y, INPUT_SCALE);
        }
        
        if (nh_.hasParam(AXIS))
        {
            nh_.getParam(AXIS, param);
            axis = (int)param; // 1: pan left and pan right, 2: tilt left and tilt right
        }
        else
        {
            ROS_ERROR("rh_ptu_range node exception: \"%s\" parameter is not set in the server", AXIS);
            return;
        }
        
        string cmd = "exec rm -r " + calibOutputDir + "*.xml";
        system(cmd.c_str());
        cmd = "exec rm -r " + imageOutputDir + "*.tif";
        system(cmd.c_str());
        cmd = "exec rm -r " + calibOutputDir + "*.py";
        system(cmd.c_str());
        cmd = "exec mkdir " + imageOutputDir;
        system(cmd.c_str());

        nrFrames = 0;
        idxL.resize(0);
        idxR.resize(0);
        rvecsL.resize(0);
        tvecsL.resize(0);
        rvecsR.resize(0);
        tvecsR.resize(0);

        cv::namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_LEFT, 320, 240);
        cvMoveWindow(WINDOW_LEFT, 20, 20);
        
        cv::namedWindow(WINDOW_RIGHT, CV_WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_RIGHT, 320, 240);
        cvMoveWindow(WINDOW_RIGHT, 380, 20);
        
        cv::startWindowThread();
        
        status_flag = nh_.advertise<std_msgs::Bool>(STATUS_IN, 1);

        sync.registerCallback(boost::bind(&CcalibrationYoung::displayImages, this, _1, _2, _3, _4));
        ROS_INFO("Node initialised");
    }
    
    ~CcalibrationYoung()
    {       
        cv::destroyWindow(WINDOW_LEFT);
        cv::destroyWindow(WINDOW_RIGHT);
    }
    
    void displayImages(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR,
        const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR)
    {        
        // Get images
        cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
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
        
        if(getCameraInfo(msgL, msgR))
        {
            ROS_INFO("Camera information read succesfully");
        }
        else
        {
            ROS_WARN("Camera information could not be read");
        }

        // Display images        
        cv::Mat dispImgL, dispImgR;
        Mat save_left, save_right;
        save_left = cv_ptrL->image.clone();
        save_right = cv_ptrR->image.clone();
        cv_ptrL->image.copyTo(dispImgL);
        cv_ptrR->image.copyTo(dispImgR);
        
        cv::line(dispImgL, cv::Point(cv_ptrR->image.cols/2.0, 0), cv::Point(cv_ptrR->image.cols/2.0, cv_ptrR->image.rows), cv::Scalar( 0, 255, 0), 5);
        cv::line(dispImgL, cv::Point(0, cv_ptrR->image.rows/2.0), cv::Point(cv_ptrR->image.cols, cv_ptrR->image.rows/2.0), cv::Scalar( 0, 255, 0), 5);
        cv::line(dispImgR, cv::Point(cv_ptrR->image.cols/2.0, 0), cv::Point(cv_ptrR->image.cols/2.0, cv_ptrR->image.rows), cv::Scalar( 0, 255, 0), 5);
        cv::line(dispImgR, cv::Point(0, cv_ptrR->image.rows/2.0), cv::Point(cv_ptrR->image.cols, cv_ptrR->image.rows/2.0), cv::Scalar( 0, 255, 0), 5);
        
        cv::imshow(WINDOW_LEFT, dispImgL);
        cv::imshow(WINDOW_RIGHT, dispImgR);

        saveImage("_left_pan.tif", save_left);
        saveImage("_right_pan.tif", save_right);
        
        // Get points from chessboard
        // Find corners
        ROS_INFO("Finding chessboard corners");
        Mat viewOutL, viewOutR;

        found_chees = false;
        viewOutL = findCorners(cv_ptrL->image);
        if(found_chees)
        {
            imagePointsL.push_back(pointBuf);
            runExtrinsic(K1_, D1_, pointBuf, 1);
            idxL.push_back(nrFrames);
            viewOutL.copyTo(dispImgL);
            line(dispImgL, Point(dispImgL.cols/2.0, 0), Point(dispImgL.cols/2.0, dispImgL.rows), Scalar( 0, 255, 0), 5);
            line(dispImgL, Point(0, dispImgL.rows/2.0), Point(dispImgL.cols, dispImgL.rows/2.0), Scalar( 0, 255, 0), 5);
            cv::imshow(WINDOW_LEFT, dispImgL);
        }
        
        found_chees = false;
        viewOutR = findCorners(cv_ptrR->image);
        if(found_chees)
        {
            imagePointsR.push_back(pointBuf);
            runExtrinsic(K2_, D2_, pointBuf, 2);
            idxR.push_back(nrFrames);
            viewOutR.copyTo(dispImgR);
            line(dispImgR, Point(dispImgR.cols/2.0, 0), Point(dispImgR.cols/2.0, dispImgR.rows), Scalar( 0, 255, 0), 5);
            line(dispImgR, Point(0, dispImgR.rows/2.0), Point(dispImgR.cols, dispImgR.rows/2.0), Scalar( 0, 255, 0), 5);
            cv::imshow(WINDOW_RIGHT, dispImgR);
        }

        nrFrames += 1;
        ROS_INFO_STREAM("Number of frames: " << nrFrames);
        std_msgs::Bool flag;
        flag.data = true;
        status_flag.publish(flag);

    }

    void saveTransformations(string output_file, string labelX2Y, const vector<Mat>& rvecs, const vector<Mat>& tvecs)
    {
            
        output_file = calibOutputDir + output_file;
        FileStorage fs(output_file, FileStorage::WRITE );
        ROS_INFO_STREAM("Saving transformation at: " << output_file);

        time_t t;
        time( &t );
        struct tm *t2 = localtime( &t );
        char buf[1024];
        strftime( buf, sizeof(buf)-1, "%c", t2 );

        fs << "calibration_Time" << buf;

        if( !rvecs.empty() && rvecs.size() == tvecs.size())
            fs << "numPoses" << (int)rvecs.size();

        if( !rvecs.empty() && !tvecs.empty())
        {
            int numPoses = (int)rvecs.size();

            for(int i = 0; i < numPoses; i++)
            {
                stringstream ss;
                ss << i;
                string name = "rvec_" + labelX2Y + "_" + ss.str();
                fs << name << rvecs[i];
            }

            for(int i = 0; i < numPoses; i++)
            {
                stringstream ss;
                ss << i;
                string name = "tvec_" + labelX2Y + "_" + ss.str();
                fs << name << tvecs[i];
            }
        }
    }

    void saveIdx(string output_file, int camQ)
    {
        output_file = calibOutputDir + output_file;
        FileStorage fs(output_file, FileStorage::WRITE );
        ROS_INFO_STREAM("Saving indeces at: " << output_file);

        if(camQ == 1)
            fs << "idx" << Mat(idxL);

        if(camQ == 2)
            fs << "idx" << Mat(idxR);
    }

    void saveCameraParams(std::string outputFile, const vector<vector<cv::Point2f> > &imagePoints)
    {
        outputFile = calibOutputDir + "/" + outputFile;

        FileStorage fs(outputFile, FileStorage::WRITE );

        time_t t;
        time( &t );
        struct tm *t2 = localtime( &t );
        char buf[1024];
        strftime( buf, sizeof(buf)-1, "%c", t2 );

        fs << "calibration_Time" << buf;
        fs << "nrOfFrames" << (int)imagePoints.size();
        fs << "board_Width" << boardSize.width;
        fs << "board_Height" << boardSize.height;
        fs << "square_Size" << squareSize;

        if( !imagePoints.empty() )
        {
            Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
            for( int i = 0; i < (int)imagePoints.size(); i++ )
            {
                Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
                Mat imgpti(imagePoints[i]);
                imgpti.copyTo(r);
            }
            fs << "Image_points" << imagePtMat;
        }
    }

    void runBA(vector<vector<Point2f> > imagePoints, Mat&  cameraMatrix_in, vector<Mat>& rvecs, vector<Mat>& tvecs)
    {
        vector<cv::Point3d> points3D;
        vector<vector<Point2d> > pointsImg;
        vector<vector<int > > visibility;
        vector<Mat > cameraMatrix, distCoeffs, R, T;

        vector<Point3f> corners;
        calcBoardCornerPositions(corners);

        cout << "size of image points: " << imagePoints.size() << endl;

        int NPOINTS = corners.size();   // number of 3d points
        int NCAMS = imagePoints.size();     // number of cameras

        cout << "Number of points for BA: " << NPOINTS << endl;

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

        cout << "Initial error=" << sba.getInitialReprjError() << ". Final error=" << sba.getFinalReprjError() << endl;
    }


private:

    bool found_chees;
    vector<Point2f> pointBuf;

    //*****************************************
    Size boardSize;                 // The size of the board -> Number of items by width and height
    float squareSize;               // The size of a square in your defined unit (point, millimeter,etc).
    string imageOutputDir;          // Folder where to save images
    string calibOutputDir;          // Folder where to save calibration info
    float scaleInput;               // Set if images should be resized
    int nrFrames;                   // Number of frames used for calibration
    Size imageSize;                 // Image size
    vector<int> idxL, idxR;
    //*****************************************
    Mat rvecEx;
    Mat tvecEx;
    sensor_msgs::CameraInfo cam_infoL, cam_infoR;

    void saveImage(string str1, const Mat& im)
    {
        stringstream ss;
        ss << nrFrames;

        string out_imageL = imageOutputDir + ss.str() + str1;

        ROS_INFO("Saving left image to: %s", out_imageL.c_str());

        imwrite(out_imageL, im);

        ROS_INFO("Images saved!");
    }

    Mat findCorners(const Mat& view)
    {
        Mat view_scaled;
        if(scaleInput > 1.0)
        {
            ROS_INFO_STREAM("Scale factor: " << scaleInput);
            ROS_INFO_STREAM("Size: " << view.cols/scaleInput << " " << view.rows/scaleInput);
            cv::resize(view, view_scaled, Size(view.cols/scaleInput, view.rows/scaleInput),
                   0, 0, INTER_CUBIC);
        }
        else
            view_scaled = view;

        pointBuf.resize(0);
        found_chees = findChessboardCorners( view_scaled, boardSize, pointBuf,
                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(scaleInput > 1.0)
        {
            for(int i = 0; i < (int)pointBuf.size(); i++)
            {
                pointBuf[i].x *= scaleInput;
                pointBuf[i].y *= scaleInput;
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
            drawChessboardCorners(view, boardSize, Mat(pointBuf), found_chees);
        }
        else
            ROS_ERROR("No corners found...");

        return view;
    }

    void calcBoardCornerPositions(vector<Point3f> &corners)
	{
        corners.clear();

        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
	}

    bool runExtrinsic(Mat cameraMatrix, Mat distCoeffs, vector<Point2f> imagePoints, int camQ)
    {
        ROS_INFO("Computing extrinsic geometry of the calibrated cameras");
        vector<Point3f> objectPoints(1);
        calcBoardCornerPositions(objectPoints);
        objectPoints.resize(imagePoints.size(), objectPoints[0]);

        //bool ok = solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecEx, tvecEx, false, CV_EPNP);

        bool ok = true;
        Mat inliers;
        solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecEx, tvecEx, false, 100, 1.0, 50, inliers);

        if(camQ == 1)
        {
            rvecsL.push_back(rvecEx.clone());
            tvecsL.push_back(tvecEx.clone());
        }
        if(camQ == 2)
        {
            rvecsR.push_back(rvecEx.clone());
            tvecsR.push_back(tvecEx.clone());
        }

        return ok;
    }

    bool getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR)
    {
        try
        {
            // Update time stamp (and frame_id if that changes for some reason)

            cam_infoL.height = msgL->height;
            cam_infoL.width = msgL->width;
            cam_infoL.D = msgL->D;
            cam_infoL.K = msgL->K;
            cam_infoL.P = msgL->P;

            cam_infoR.height = msgR->height;
            cam_infoR.width = msgR->width;
            cam_infoR.D = msgR->D;
            cam_infoR.K = msgR->K;
            cam_infoR.P = msgR->P;

            Mat K1, D1, P1, K2, D2, P2;

            D1 = Mat::zeros(1,5,CV_64F);
            K1 = Mat::zeros(3,3,CV_64F);
            P1 = Mat::zeros(3,4,CV_64F);

            D2 = Mat::zeros(1,5,CV_64F);
            K2 = Mat::zeros(3,3,CV_64F);
            P2 = Mat::zeros(3,4,CV_64F);

            for(int i = 0; i < D1.cols; i++)
            {
                D1.at<double>(0,i) = msgL->D[i];
                D2.at<double>(0,i) = msgR->D[i];

                if(i < K1.rows)
                {
                    for(int j = 0 ; j < P1.cols; j++)
                    {
                        if(j < K1.cols)
                        {
                            K1.at<double>(i,j) = msgL->K[3*i+j];
                            K2.at<double>(i,j) = msgR->K[3*i+j];
                        }
                        P1.at<double>(i,j) = msgL->P[4*i+j];
                        P2.at<double>(i,j) = msgR->P[4*i+j];
                    }
                }
            }

            D1_ = D1.clone();
            K1_ = K1.clone();
            P1_ = P1.clone();

            D2_ = D2.clone();
            K2_ = K2.clone();
            P2_ = P2.clone();

        }
        catch (...)
        {
            return false;
        }

        return true;
    }

};

/* *************** MAIN PROGRAM *************** */
int main(int argc, char** argv)
{
    ros::init( argc, argv, "RH_cal_young_capture" );
    CcalibrationYoung cY_;

    while( ros::ok() )
    {
        ros::spin();
    }

    if(cY_.axis == 1){
        cY_.saveTransformations("cHw_pan_left.xml", "camL2target", cY_.rvecsL, cY_.tvecsL);
        cY_.saveTransformations("cHw_pan_right.xml", "camL2target", cY_.rvecsR, cY_.tvecsR);
        cY_.saveCameraParams("image_points_pan_left.xml", cY_.imagePointsL);
        cY_.saveCameraParams("image_points_pan_right.xml", cY_.imagePointsR);

        cY_.runBA(cY_.imagePointsL, cY_.K1_, cY_.rvecsL, cY_.tvecsL);
        cY_.runBA(cY_.imagePointsR, cY_.K1_, cY_.rvecsR, cY_.tvecsR);
        cY_.saveTransformations("cHw_pan_left_BA.xml", "camL2target", cY_.rvecsL, cY_.tvecsL);
        cY_.saveTransformations("cHw_pan_right_BA.xml", "camL2target", cY_.rvecsR, cY_.tvecsR);

        cY_.saveIdx("idx_left_pan.xml", 1);
        cY_.saveIdx("idx_right_pan.xml", 2);
    }
    if(cY_.axis == 2){
        cY_.saveTransformations("cHw_tilt_left.xml", "camL2target", cY_.rvecsL, cY_.tvecsL);
        cY_.saveTransformations("cHw_tilt_right.xml", "camL2target", cY_.rvecsR, cY_.tvecsR);
        cY_.saveCameraParams("image_points_tilt_left.xml", cY_.imagePointsL);
        cY_.saveCameraParams("image_points_tilt_right.xml", cY_.imagePointsR);

        cY_.runBA(cY_.imagePointsL, cY_.K1_, cY_.rvecsL, cY_.tvecsL);
        cY_.runBA(cY_.imagePointsR, cY_.K1_, cY_.rvecsR, cY_.tvecsR);
        cY_.saveTransformations("cHw_tilt_left_BA.xml", "camL2target", cY_.rvecsL, cY_.tvecsL);
        cY_.saveTransformations("cHw_tilt_right_BA.xml", "camL2target", cY_.rvecsR, cY_.tvecsR);

        cY_.saveIdx("idx_left_tilt.xml", 1);
        cY_.saveIdx("idx_right_tilt.xml", 2);
    }

    return EXIT_SUCCESS;
}












