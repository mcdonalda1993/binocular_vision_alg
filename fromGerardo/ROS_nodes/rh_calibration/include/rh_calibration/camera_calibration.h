//
//  camera_calibration.h
//  Camera Calibration class
//
//  Created by Gerardo Aragon on 21/11/2012.
//  Modified by Gerardo Aragon on 14/04/2014,
//  Copyright (c) 2012 Gerardo Aragon. All rights reserved.

#ifndef camera_calibration_h
#define camera_calibration_h

#define MAX_ITERATION 5

class Calibration
{
public:

    //*****************************************
    Size boardSize;					// The size of the board -> Number of items by width and height
    float squareSize;				// The size of a square in your defined unit (point, millimeter,etc).
    float maxError;                 // Maximum reprojection error allowed
    int save_mode;					// 0: Nothing, 1: Only save processed images in hard disk, 2: Save everything
    string imageOutputDir;			// Folder where to save images
    string calibOutputDir;			// Folder where to save calibration info
    float scaleInput;				// Set if images should be resized
    int nrFrames;                   // Number of frames used for calibration
    Size imageSize;                 // Image size
    string he_calib_url;
    //*****************************************

    vector<Mat> rvecs, tvecs; // For camera calibration (runCalibration)
    vector<Mat> rvecs_robot, tvecs_robot;

    //vector<double> rv(3), tv(3); // For camera position from planar surface (runExtrinsic)
    Mat rvecEx;
    Mat tvecEx;

    Mat R, T;

    string calibTarget;
    string opencvStr;
    string alvarStr;

    vector<vector<Point2f> > cal_2Dpoints;

    void init();

    // int cam should be 0 for left and 1 for right
    bool runCalibrationAndSave(Mat& cameraMatrix, Mat& distCoeffs,
                               vector<vector<Point2f> > imagePoints, int cam);

    bool runStereoCalibrationAndSave(Mat& cameraMatrix1, Mat& cameraMatrix2,
                                     Mat& distCoeffs1, Mat& distCoeffs2, Mat& F, Mat& P_out, Mat& H_stereo,
                                     vector<vector<Point2f> > imagePoints1, vector<vector<Point2f> > imagePoints2);

    bool runExtrinsic(Mat cameraMatrix, Mat distCoeffs, vector<Point2f> imagePoints);

    void printMatrix(Mat M, bool printType = true);

    void saveCalibPoints(string str, const vector<vector<cv::Point2f> > &imagePoints);

    void calcBoardCornerPositions(vector<Point3f>& corners);

    void savePTUPositions(string output_file, const vector<Point2f> &ptuL, const vector<Point2f> &ptuR);

    void saveTransformations(string output_file, string labelX2Y,
                             const vector<Mat>& in_rvecs, const vector<Mat>& in_tvecs);

    void saveTransformations(string output_file, string labelX2Y,
                             const vector<Mat>& in_vec);

    void calibTransform(string output_file, string labelX2Y, const Mat& in_vec);

    void saveHandEyeTransform(string output_file, const Mat& left, const Mat& right, std::string hName);

    void saveHandEyeTransform(string output_file, const Mat& left, const Mat& right, const Mat& baseline, std::string hName);

    void saveRobotPoses(string output_file, const vector<Mat>& in);

private:

    double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                      const vector<vector<Point2f> >& imagePoints,
                                      const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                      const Mat& cameraMatrix , const Mat& distCoeffs,
                                      vector<float>& perViewErrors);

    bool runCalibration(Mat& cameraMatrix, Mat& distCoeffs,
                        vector<vector<Point2f> > imagePoints, vector<float>& reprojErrs,  double& totalAvgErr);

    double computeReprojectionErrorsStereo(const vector<vector<Point2f> >& imagePoints1,
                                           const vector<vector<Point2f> >& imagePoints2,
                                           const Mat& cameraMatrix1, const Mat& cameraMatrix2,
                                           const Mat& distCoeffs1, const Mat& distCoeffs2,
                                           const Mat& F);

    bool runStereoCalibration(Mat& cMatrix1, Mat& cMatrix2,
                              Mat& d1, Mat& d2, vector<vector<Point2f> > imagePoints1,
                              vector<vector<Point2f> > imagePoints2, Mat& R, Mat& T, Mat& E, Mat& F,
                              double& totalAvgErr);

    // Save stereo parameters
    void saveStereoParams(std::string outputFile, Mat& cameraMatrix1,
                          Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Mat& R, Mat& T, Mat& E, cv::Mat &F, double totalAvgErr);

    // Print camera parameters to the output file
    void saveCameraParams(std::string outputFile, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                          const vector<float> &reprojErrs,const vector<vector<cv::Point2f> > &imagePoints, double totalAvgErr);

};

#endif
