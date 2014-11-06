//
//  handEye_routines.h
//  Hand Eye Calibration
//
//  Created by Gerardo Aragon on 31/11/2012.
//  Copyright (c) 2012 Gerardo Aragon. All rights reserved.
//

#ifndef handeye_calibration_h
#define handeye_calibration_h

class Chandeye
{

public:

    //Variables
    vector<Mat> bHg; // robot base 2 gripper
    vector<Mat> tHc; // calibration grid 2 camera
    vector<Mat> bHc; // base 2 camera (left or right)
    vector<Mat> bHct; // base to calibration target (either left or right chain)
    
    Mat gHc;
    Mat residuals;

    int M; // no of stations
    bool invH;

    //Chandeye(const char * filename); // For debug only
    Chandeye()
    {
        tHc.resize(0);
        bHg.resize(0);
    }

    ~Chandeye()
    {
        ROS_INFO("Closing hand eye routines");
    }

    // Rotation gripper, translation gripper, rotation camera, translation camera
    bool loadParameters(const vector<Mat>& rG_in, const vector<Mat>& tG_in,
                        const vector<Mat>& rC_in, const vector<Mat>& tC_in, bool invCamera = true);

    void calibrate();

    
    Mat avgTransformation(const vector<Mat>& H, bool decomposeQ);

private:

    //Functions
    Mat decompose_rotation(Mat R, Mat T);

    Mat rodrigues_custom(Mat R);
    
    Mat quat2rot(Mat q);

    Mat diagonal(float p);

    Mat skew(Mat V);

    Mat rot2quat(Mat R);

    Mat CrossProduct(Mat in);

    void printMatrix(Mat M, bool printType = true);

};

#endif
