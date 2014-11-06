//
//  Camera Calibration class
//
//  Created by Gerardo Aragon on 03/2014.
//  Copyright (c) 2014 Gerardo Aragon. All rights reserved.

#include <rh_calibration/RHcalibration.h>
#include <rh_calibration/camera_calibration.h>

#define MAX_ITER 5

void Calibration::init()
{
    opencvStr = "opencv";
    alvarStr = "alvar";
}

// int cam should be 0 for left and 1 for right
bool Calibration::runCalibrationAndSave(Mat&  cameraMatrix, Mat& distCoeffs, vector<vector<Point2f> > imagePoints, int cam)
{
    cal_2Dpoints.clear();
    cal_2Dpoints = imagePoints;
    bool ok = false;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    try
    {
        for(int i = 0; i < MAX_ITERATION; i++)
        {
            vector<float> reprojErrsT;
            double totalAvgErrT = 0;
            ok = runCalibration(cameraMatrix, distCoeffs, cal_2Dpoints, reprojErrsT, totalAvgErrT);
            vector<vector<Point2f> > tempP;
            vector<Mat> rTemp, tTemp;
            tempP.resize(0);
            rTemp.resize(0);
            tTemp.resize(0);
            for(int j = 0; j < (int)reprojErrsT.size(); j++)
            {
                if(reprojErrsT.at(j) < maxError)
                {
                    tempP.push_back(cal_2Dpoints.at(j));
                    rTemp.push_back(rvecs_robot.at(j));
                    tTemp.push_back(tvecs_robot.at(j));
                }
            }
            if(tempP.size() == cal_2Dpoints.size())
            {
                reprojErrs = reprojErrsT;
                totalAvgErr = totalAvgErrT;
                ROS_WARN_STREAM("Calibrated after " << i+1 << " iterations");
                break;
            }
            if((int)tempP.size() <= 8)
            {
                reprojErrs = reprojErrsT;
                totalAvgErr = totalAvgErrT;
                ROS_ERROR("Bad calibration, re-consider to take new images! Trying to continue...");
                break;
            }
            cal_2Dpoints.resize(0);
            rvecs_robot.clear();
            tvecs_robot.clear();
            rvecs_robot.resize(0);
            tvecs_robot.resize(0);
            cal_2Dpoints = tempP;
            rvecs_robot = rTemp;
            tvecs_robot = tTemp;
            ROS_WARN_STREAM("Re-running calibration, iteration #" << i+1 << " with " << tempP.size() << " of good images!");
        }
    }
    catch (exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = "  << totalAvgErr << endl;

    if( ok )
    {
        if(cam == 0)
            saveCameraParams("camera_calibration_left.xml", cameraMatrix, distCoeffs, reprojErrs, cal_2Dpoints, totalAvgErr);

        if(cam == 1)
            saveCameraParams("camera_calibration_right.xml", cameraMatrix, distCoeffs, reprojErrs, cal_2Dpoints, totalAvgErr);
    }
    return ok;
}

bool Calibration::runStereoCalibrationAndSave(Mat &cameraMatrix1, Mat &cameraMatrix2, Mat& distCoeffs1, Mat& distCoeffs2,
                                              Mat& F, Mat& P_out, Mat& H_stereo,
                                              vector<vector<Point2f> > imagePoints1, vector<vector<Point2f> > imagePoints2)
{

    double totalAvgErr = 0;

    Mat E;

    vector<vector<Point2f> > tempPoints1, tempPoints2;
    tempPoints1.resize(0);
    tempPoints2.resize(0);
    tempPoints1 = imagePoints1;
    tempPoints2 = imagePoints2;

    bool ok = runStereoCalibration(cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2,
                                   tempPoints1, tempPoints2, R, T, E, F, totalAvgErr);

    ROS_INFO_STREAM((ok ? "Stereo Calibration succeeded" : "Stereo Calibration failed") << ". avg re projection error (stereo) = "  << totalAvgErr);

    hconcat(R, T, P_out);
    P_out = cameraMatrix2 * P_out;

    H_stereo = Mat::eye(4,4, CV_32F);
    H_stereo.at<float>(0,0) = (float)R.at<double>(0,0);
    H_stereo.at<float>(0,1) = (float)R.at<double>(0,1);
    H_stereo.at<float>(0,2) = (float)R.at<double>(0,2);
    H_stereo.at<float>(1,0) = (float)R.at<double>(1,0);
    H_stereo.at<float>(1,1) = (float)R.at<double>(1,1);
    H_stereo.at<float>(1,2) = (float)R.at<double>(1,2);
    H_stereo.at<float>(2,0) = (float)R.at<double>(2,0);
    H_stereo.at<float>(2,1) = (float)R.at<double>(2,1);
    H_stereo.at<float>(2,2) = (float)R.at<double>(2,2);

    H_stereo.at<float>(0,3) = (float)T.at<double>(0);
    H_stereo.at<float>(1,3) = (float)T.at<double>(1);
    H_stereo.at<float>(2,3) = (float)T.at<double>(2);

    // for(int jj = 0; jj <= 3; jj++)
    // {
    //     H_stereo.at<float>(jj,3) = (float)T.at<double>(jj);
    //     for(int kk = 0; kk <= 3; kk++)
    //     {
    //         H_stereo.at<float>(kk,jj) = (float)R.at<double>(kk,jj);
    //     }
    // }

    if( ok )
        saveStereoParams("stereo_calibration.xml",cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F, totalAvgErr);

    return ok;
}

bool Calibration::runExtrinsic(Mat cameraMatrix, Mat distCoeffs, vector<Point2f> imagePoints)
{
    //cout << "Computing extrinsic geometry of the calibrated cameras" << endl;
    vector<Point3f> objectPoints(1);
    calcBoardCornerPositions(objectPoints);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //bool ok = solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecEx, tvecEx, false, CV_EPNP);

    bool ok = true;
    Mat inliers;
    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecEx, tvecEx, false, 100, 1.0, 50, inliers);

    return ok;
}

void Calibration::printMatrix(Mat M, bool printType)
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

void Calibration::saveCalibPoints(string str, const vector<vector<Point2f> > &imagePoints)
{
    string out_points = calibOutputDir + str;
    ROS_INFO("Saving points of the calibration target to: %s", out_points.c_str());

    FileStorage fs(out_points, FileStorage::WRITE );

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

    fs.release();

}

void Calibration::calcBoardCornerPositions(vector<Point3f> &corners)
{
    if(calibTarget == alvarStr)
    {
        corners.clear();
        // left down
        corners.push_back(Point3f(0,0,0));
        // right down
        corners.push_back(Point3f(squareSize,0,0));
        // right up
        corners.push_back(Point3f(squareSize,squareSize,0));
        // left up
        corners.push_back(Point3f(0,squareSize,0));

    }else if(calibTarget == opencvStr)
    {
        corners.clear();

        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
    }
}

/* in_rvecs, in_tvecs => Rotation (Rodrigues form) and translation of X wrt to Y */
void Calibration::saveTransformations(string output_file, string labelX2Y,
                                      const vector<Mat>& in_rvecs, const vector<Mat>& in_tvecs)
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

    if( !in_rvecs.empty() && in_rvecs.size() == in_tvecs.size())
        fs << "numPoses" << (int)in_rvecs.size();

    if( !in_rvecs.empty() && !in_tvecs.empty())
    {
        int numPoses = (int)in_rvecs.size();

        for(int i = 0; i < numPoses; i++)
        {
            stringstream ss;
            ss << i;
            string name = "rvec_" + labelX2Y + "_" + ss.str();
            fs << name << in_rvecs[i];
        }

        for(int i = 0; i < numPoses; i++)
        {
            stringstream ss;
            ss << i;
            string name = "tvec_" + labelX2Y + "_" + ss.str();
            fs << name << in_tvecs[i];
        }
    }
}

/* in_vec -> Transformation matrix */
void Calibration::saveTransformations(string output_file, string labelX2Y,
                                      const vector<Mat>& in_vec)
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

    if( !in_vec.empty())
        fs << "numPoses" << (int)in_vec.size();

    if( !in_vec.empty())
    {
        int numPoses = (int)in_vec.size();

        for(int i = 0; i < numPoses; i++)
        {
            stringstream ss;
            ss << i;
            string name = "H_" + labelX2Y + "_" + ss.str();
            fs << name << in_vec[i];
        }
    }
}

void Calibration::saveRobotPoses(string output_file, const vector<Mat>& in)
{
    string outputFile = calibOutputDir + "/" + output_file;

    std::ofstream myfile;
    myfile.open (outputFile.c_str(), std::ios::app);
    for(unsigned int i=0; i < in.size(); i++)
    {
        Mat temp = in.at(i);
        myfile << "calib_target_.append([";
        for(int j=0; j < temp.cols; j++)
        {
            if(j < temp.cols - 1)
                myfile << temp.at<float>(0,j) << ", ";
            else
                myfile << temp.at<float>(0,j) << "])\n";
        }
    }
    myfile.close();
    
    ROS_ERROR_STREAM("File was saved in " << outputFile);
}

/* in_vec -> Transformation matrix */
void Calibration::calibTransform(string output_file, string labelX2Y,
                                      const Mat& in_vec)
{

    output_file = calibOutputDir + output_file;
    FileStorage fs(output_file, FileStorage::WRITE );
    ROS_INFO_STREAM("Saving calibration transformation at: " << output_file);

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    string name = "H_" + labelX2Y;
    fs << name << in_vec;
    
}

/* ptuL, ptuR => PTU positions in actuator steps */
void Calibration::savePTUPositions(string output_file, const vector<Point2f> &ptuL, const vector<Point2f> &ptuR)
{
    output_file = calibOutputDir + output_file;
    FileStorage fs(output_file, FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !ptuL.empty() && !ptuR.empty() && ptuR.size() == ptuL.size() )
    {
        Mat bigptu((int)ptuL.size(), 4, CV_32F);
        for( int i = 0; i < (int)ptuL.size(); i++ )
        {
            Mat p1 = bigptu(Range(i,i+1), Range(0,2));
            Mat p2 = bigptu(Range(i,i+1), Range(2,4));

            p1.at<float>(0,0) = ptuL[i].x; // pan
            p1.at<float>(0,1) = ptuL[i].y; // tilt
            p2.at<float>(0,0) = ptuR[i].x; // pan
            p2.at<float>(0,1) = ptuR[i].y; // tilt
        }
        cvWriteComment( *fs, "4 angle position of both PTUs (PTUL_X, PTUL_Y, PTUR_X, PTUR_Y)", 0 );
        fs << "PTU_angles" << bigptu;
    }
}

void Calibration::saveHandEyeTransform(string output_file, const Mat& left, const Mat& right, string hName)
{
    // Save stuff
    output_file = calibOutputDir + output_file;
    FileStorage fs1(output_file, FileStorage::WRITE );
    ROS_INFO_STREAM("gripperHcamera being saved at: " << output_file);
    fs1 << hName+"_LeftCamera" << left;
    fs1 << hName+"gripperHcamera_RightCamera" << right;
    fs1.release();

}

void Calibration::saveHandEyeTransform(string output_file, const Mat& left, const Mat& right, const Mat& baseline, string hName)
{
    // Save stuff
    output_file = calibOutputDir + output_file;
    FileStorage fs1(output_file, FileStorage::WRITE );
    ROS_INFO_STREAM("gripperHcamera and leftpanHrightpan (baseline) being saved at: " << output_file);
    fs1 << hName+"_LeftCamera" << left;
    fs1 << hName+"_RightCamera" << right;
    cvWriteComment( *fs1, "a set of 6-tuples: row, pitch yaw (euler angles) and x, y and z (translation) of the transformation", 0 );
    cvWriteComment( *fs1, "This has to be reflected in the robot_head.xacro file in rh_ptu ", 0 );
    fs1 << "leftpanHrightpan" << baseline;
    fs1.release();

    // For cameras
    output_file = he_calib_url;
    FileStorage fs2(output_file, FileStorage::WRITE );
    ROS_INFO_STREAM("gripperHcamera and leftpanHrightpan (baseline) being saved at: " << output_file);
    fs2 << hName+"_LeftCamera" << left;
    fs2 << hName+"_RightCamera" << right;
    cvWriteComment( *fs2, "a set of 6-tuples: row, pitch yaw (euler angles) and x, y and z (translation) of the transformation", 0 );
    cvWriteComment( *fs2, "This has to be reflected in the robot_head.xacro file in rh_ptu ", 0 );
    fs2 << "leftpanHrightpan" << baseline;
    fs2.release();

}


// ************************************************
// Private functions
// ************************************************
double Calibration::computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                               const vector<vector<Point2f> >& imagePoints,
                                               const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                               const Mat& cameraMatrix , const Mat& distCoeffs,
                                               vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

bool Calibration::runCalibration( Mat &cameraMatrix, Mat &distCoeffs, vector<vector<Point2f> > imagePoints, vector<float> &reprojErrs,
                                  double &totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_32F);

    distCoeffs = Mat::zeros(5, 1, CV_32F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(objectPoints[0]);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    rvecs.resize(0);
    tvecs.resize(0);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST +
                                 CV_CALIB_SAME_FOCAL_LENGTH +
                                 CV_CALIB_RATIONAL_MODEL +
                                 CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);


    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

double Calibration::computeReprojectionErrorsStereo(const vector<vector<Point2f> >& imagePoints1,
                                                    const vector<vector<Point2f> >& imagePoints2,
                                                    const Mat& cameraMatrix1, const Mat& cameraMatrix2,
                                                    const Mat& distCoeffs1, const Mat& distCoeffs2,
                                                    const Mat& F)
{
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( int i = 0; i < (int)imagePoints1.size(); i++ )
    {
        int npt = (int)imagePoints1[0].size();
        Mat imgpt[2];

        imgpt[0] = Mat(imagePoints1[i]);
        undistortPoints(imgpt[0], imgpt[0], cameraMatrix1, distCoeffs1, Mat(), cameraMatrix1);
        computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);

        imgpt[1] = Mat(imagePoints2[i]);
        undistortPoints(imgpt[1], imgpt[1], cameraMatrix2, distCoeffs2, Mat(), cameraMatrix2);
        computeCorrespondEpilines(imgpt[1], 2, F, lines[1]);

        for( int j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints1[i][j].x*lines[1][j][0] +
                                imagePoints1[i][j].y*lines[1][j][1] + lines[1][j][2]) +
                    fabs(imagePoints2[i][j].x*lines[0][j][0] +
                         imagePoints2[i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    return std::sqrt(err/npoints);
    cout << "average reprojection (stereo) err = " <<  err/npoints << endl;
}

bool Calibration::runStereoCalibration( Mat &cMatrix1, Mat &cMatrix2, Mat& d1, Mat& d2,
                                        vector<vector<Point2f> > imagePoints1, vector<vector<Point2f> > imagePoints2, Mat &R,
                                        Mat &T, Mat& E, Mat& F, double &totalAvgErr)
{
    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(objectPoints[0]);
    objectPoints.resize(imagePoints1.size(),objectPoints[0]);

    Mat cameraMatrix1 = cMatrix1;
    Mat cameraMatrix2 = cMatrix2;
    Mat distCoeffs1 = d1;
    Mat distCoeffs2 = d2;

    double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                 cameraMatrix1, distCoeffs1,
                                 cameraMatrix2, distCoeffs2,
                                 imageSize, R, T, E, F,
                                 TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                 CV_CALIB_FIX_INTRINSIC +
                                 CV_CALIB_FIX_ASPECT_RATIO +
                                 CV_CALIB_ZERO_TANGENT_DIST +
                                 CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by stereoCalibrate: "<< rms << endl;

    bool ok = checkRange(R) && checkRange(T) && checkRange(E) && checkRange(F);

    totalAvgErr = computeReprojectionErrorsStereo(imagePoints1, imagePoints2,
                                                  cameraMatrix1, cameraMatrix2,
                                                  distCoeffs1, distCoeffs2, F);

    return ok;
}

// Save stereo parameters
void Calibration::saveStereoParams(string outputFile, Mat &cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2,
                                   Mat& distCoeffs2, Mat& R, Mat& T, Mat& E, Mat& F, double totalAvgErr)
{

    outputFile = calibOutputDir + "/" + outputFile;
    ROS_INFO_STREAM("Saving stereo parameters at: " << outputFile);
    FileStorage fs(outputFile, FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;

    fs << "K1" << cameraMatrix1;
    fs << "D1" << distCoeffs1;

    fs << "K2" << cameraMatrix2;
    fs << "D2" << distCoeffs2;

    fs << "Avg_Reprojection_Error" << totalAvgErr;

    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
}

// Print camera parameters to the output file
void Calibration::saveCameraParams(std::string outputFile, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                                   const vector<float> &reprojErrs, const vector<vector<cv::Point2f> > &imagePoints,
                                   double totalAvgErr)
{
    outputFile = calibOutputDir + "/" + outputFile;

    FileStorage fs(outputFile, FileStorage::WRITE );

    time_t t;
    time( &t );
    struct tm *t2 = localtime( &t );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << boardSize.width;
    fs << "board_Height" << boardSize.height;
    fs << "square_Size" << squareSize;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

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
