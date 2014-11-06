//
//  Hand-Eye Calibration class definition
//
//  Created by Gerardo Aragon on 03/2014.
//  Copyright (c) 2014 Gerardo Aragon. All rights reserved.

#include <rh_calibration/RHcalibration.h>
#include <rh_calibration/handeye_calibration.h>

// Rotation gripper, translation gripper, rotation camera, translation camera
bool Chandeye::loadParameters(const vector<Mat>& rG_in, const vector<Mat>& tG_in,
                    const vector<Mat>& rC_in, const vector<Mat>& tC_in, bool invCamera)
{
    invH = invCamera;
    ROS_INFO_STREAM("Size G: " << rG_in.size() << " and " << tG_in.size());
    ROS_INFO_STREAM("Size C: " << rC_in.size() << " and " << tC_in.size());
    if(rG_in.size() == tG_in.size())
        M = rG_in.size();
    else
        return false;

    ROS_INFO_STREAM("Number of stations: " << M);
    ROS_INFO("bHg");
    bHg.resize(0);
    for(int i=0; i<M; i++)
    {
        Mat tempM = Mat::zeros(4, 4, CV_32F);
        Mat rTemp;// = tempM(Range(0, 3), Range(0, 3));

        tempM.at<float>(3,3) = 1;

        //Rodrigues(rG_in[i], rTemp);
        rTemp = rodrigues_custom(rG_in[i]);

        for(int ii = 0; ii < 3; ii++)
        {
            tempM.at<float>(ii,3) = tG_in[i].at<float>(ii);
            for(int jj = 0; jj < 3; jj++)
                tempM.at<float>(ii,jj) = rTemp.at<float>(ii,jj);
        }

        bHg.push_back(tempM);
    }

    ROS_INFO("tHc");
    tHc.resize(0);
    // solvePnP finds the transformation from camera to grid
    for(int i=0; i<M; i++)
    {
        Mat tempM = Mat::zeros(4, 4, CV_32F);
        Mat rTemp;// = tempM(Range(0, 3), Range(0, 3));

        tempM.at<float>(3,3) = 1;

        Mat forRod = Mat::zeros(1, 3, CV_32F);
        forRod.at<float>(0) = (float)rC_in[i].at<double>(0);
        forRod.at<float>(1) = (float)rC_in[i].at<double>(1);
        forRod.at<float>(2) = (float)rC_in[i].at<double>(2);
        //Rodrigues(forRod, rTemp);
        rTemp = rodrigues_custom(forRod);

        for(int ii = 0; ii < 3; ii++)
        {
            tempM.at<float>(ii,3) = (float)tC_in[i].at<double>(ii);
            for(int jj = 0; jj < 3; jj++)
                tempM.at<float>(ii,jj) = rTemp.at<float>(ii,jj);
        }

        // inverse so it is grid to camera
        Mat invTemp;
        if(invCamera)
            invTemp = tempM.inv();
        else
            invTemp = tempM;

        tHc.push_back(invTemp);
        //printMatrix(invTemp, false);
        //printMatrix(tHc[i], false);
        //return false;
    }

    ROS_INFO("Finish loading matrices for hand-eye calibration!!");

    return true;

}
void Chandeye::calibrate()
{
    ROS_INFO("Entering hand-eye calibration...");
    int K = (M*M-M) / 2; // Number of unique camera position pairs
    Mat A = Mat::zeros(3*K,3, CV_32F); // will store: skew(Pgij+Pcij)
    Mat B = Mat::zeros(3*K,1,  CV_32F); // will store: Pcij - Pgij
    Mat A_trans = Mat::zeros(3*K,3, CV_32F);
    Mat B_trans = Mat::zeros(3*K,1,  CV_32F);
    int k = 0;

    // Now convert from tHc notation to Hc notation used in Tsai paper.
    vector<Mat> Hg = bHg;
    // Hc = cHw = inv(tHc); We do it in a loop because tHc is given, not cHw
    vector<Mat> Hc;
    Hc.resize(0);

    for(int i = 0; i < M; i++)
        Hc.push_back(tHc[i].inv());

    for(int i = 0; i < M; i++)
    {
        for(int j = i+1; j < M; j++)
        {
            Mat Hgij = Hg[j].inv() * Hg[i]; //Transformation from i-th to j-th gripper pose
            Mat Pgij = rot2quat(Hgij) * 2; //... and the corresponding quaternion

            Mat Hcij = Hc[j] * Hc[i].inv(); //Transformation from i-th to j-th camera pose
            Mat Pcij = rot2quat(Hcij) * 2; //... and the corresponding quaternion

            k += 1;
            // Form linear system of equations for rotational component
            Mat skewMat = skew(Pgij + Pcij);
            Mat tempSubstract = Pcij - Pgij;

            for(int ii = 1; ii <= 3; ii++)
            {
                B.at<float>((3*k-3) + ii - 1, 0) = tempSubstract.at<float>(ii - 1); //right-hand side
                for(int jj = 1; jj <= 3; jj++)
                    A.at<float>((3*k-3) + ii - 1, jj - 1) = skewMat.at<float>(ii - 1,jj - 1); //left-hand side
            }

        }
    }

    // Computing Rotation
    // Rotation from camera to gripper is obtained from the set of equations:
    //    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij
    // Gripper with camera is first moved to M different poses, then the gripper
    // .. and camera poses are obtained for all poses. The above equation uses
    // .. invariances present between each pair of i-th and j-th pose.
    Mat A_inv = A.inv(DECOMP_SVD);
    Mat Pcg_ = A_inv * B;

    // Obtained non-unit quaternin is scaled back to unit value that
    // .. designates camera-gripper rotation A.dot(B);
    Mat Pcg = 2 * Pcg_ / sqrt(1 + Pcg_.dot(Pcg_) );
    Mat Rcg = quat2rot(Pcg * 0.5); // Rotation matrix

    k = 0;
    Mat Rcg_rot = Rcg(Range(0, 3), Range(0, 3));
    for(int i = 0; i < M; i++)
    {
        for(int j = i+1; j < M; j++)
        {
            Mat Hgij = Hg[j].inv() * Hg[i]; //Transformation from i-th to j-th gripper pose
            Mat Hcij = Hc[j] * Hc[i].inv(); //Transformation from i-th to j-th camera pose

            k += 1;
            Mat Hgij_rot = Hgij(Range(0, 3), Range(0, 3));
            Mat tempT1 = Hgij_rot - Mat::eye(3,3,CV_32F);

            Mat Hcij_trans = Mat::zeros(3,1,CV_32F);
            Mat Hgij_trans = Mat::zeros(3,1,CV_32F);
            for(int ii = 0; ii < 3; ii++)
            {
                Hcij_trans.at<float>(ii) = Hcij.at<float>(ii,3);
                Hgij_trans.at<float>(ii) = Hgij.at<float>(ii,3);
            }

            Mat tempT2 = Rcg_rot * Hcij_trans - Hgij_trans;

            for(int ii = 1; ii <= 3; ii++)
            {
                B_trans.at<float>((3*k-3) + ii - 1, 0) = tempT2.at<float>(ii - 1); //right-hand side
                for(int jj = 1; jj <= 3; jj++)
                    A_trans.at<float>((3*k-3) + ii - 1, jj - 1) = tempT1.at<float>(ii - 1,jj - 1);
            }
        }
    }

    Mat A1_inv = A_trans.inv(DECOMP_SVD);
    Mat Tcg = A1_inv * B_trans;

    Mat transl = Mat::eye(4,4,CV_32F);
    transl.at<float>(0,3) = Tcg.at<float>(0);
    transl.at<float>(1,3) = Tcg.at<float>(1);
    transl.at<float>(2,3) = Tcg.at<float>(2);

    gHc = transl * Rcg; // incorporate translation with rotation

    ROS_INFO("gHc:");
    printMatrix(gHc, false);

    bHc.resize(0);
    bHct.resize(0);
    for(int i = 0; i < M; i++)
    {
        Mat bHc_temp = bHg[i] * gHc;
        Mat bHct_temp = bHg[i] * gHc * tHc[i].inv();

        // if(invH)
        //     Mat bHct_temp = bHg[i] * gHc * tHc[i].inv();
        // else
        //     Mat bHct_temp = bHg[i] * gHc * tHc[i];

        bHc.push_back(bHc_temp);
        bHct.push_back(bHct_temp);
    }

    //printMatrix(bHc[3], false);
    //printMatrix(bHct[3], false);

    return;

}

Mat Chandeye::avgTransformation(const vector<Mat>& H, bool decomposeQ)
{
    // Average orientation
    Mat tempR = Mat::zeros(3,3,CV_32F);
    for(int i=0; i < M; i++)
    {
        Mat R_add = Mat::zeros(3,3,CV_32F);
        for(int j = 0; j < 3; j++)
            for(int k = 0; k < 3; k++)
                R_add.at<float>(j,k) = H[i].at<float>(j,k);

        tempR = tempR + R_add;
    }

    Mat R_bar = tempR / (float)M;
    Mat RTR = R_bar.t() * R_bar;

    Mat D = Mat::eye(3,3,CV_32F);
    Mat V;
    Mat d, Vt;
    SVD::compute(RTR, d, V, Vt);

    Mat mul_temp = V * D * V.t();
    Mat Ravg = R_bar * mul_temp;

    // Average translations
    Mat Tavg_temp = Mat::zeros(3,1,CV_32F);
    Mat Tavg = Mat::zeros(3,1,CV_32F);
    for(int i = 0; i < M; i++)
    {
        Mat temp = Mat::zeros(3,1,CV_32F);
        for(int j = 0; j < 3; j++)
            temp.at<float>(j) = H[i].at<float>(j,3);

        Tavg_temp = Tavg_temp + temp;
    }

    Tavg_temp = Tavg_temp / M;

    // Do this so in xacro file x, y and z corresponds in order to avoid confussions
    Tavg.at<float>(0) = Tavg_temp.at<float>(0);
    Tavg.at<float>(1) = Tavg_temp.at<float>(1);
    Tavg.at<float>(2) = Tavg_temp.at<float>(2);

    // For debug
    /*Mat Havg = Mat::zeros(4,4,CV_32F);
    for(int i = 0; i < 3; i++)
    {
        Havg.at<float>(i,3) = Tavg.at<float>(i);
        for(int j = 0; j < 3; j++)
            Havg.at<float>(i,j) = Ravg.at<float>(i,j);
    }
    printMatrix(Havg,false);*/

    if(decomposeQ)
        return decompose_rotation(Ravg, Tavg);
    else
    {
        Mat Havg = Mat::zeros(4,4,CV_32F);
        for(int i = 0; i < 3; i++)
        {
            Havg.at<float>(i,3) = Tavg.at<float>(i);
            for(int j = 0; j < 3; j++)
                Havg.at<float>(i,j) = Ravg.at<float>(i,j);
        }
        Havg.at<float>(3,3) = 1;
        return Havg;
    }
}

// Private functions
Mat Chandeye::decompose_rotation(Mat R, Mat T)
{
    Mat vec = Mat::zeros(1,6,CV_32F);
    vec.at<float>(0) = atan2(R.at<float>(2,1), R.at<float>(2,2));
    vec.at<float>(1) = atan2(-1*R.at<float>(2,0), sqrt(R.at<float>(2,1) * R.at<float>(2,1) + R.at<float>(2,2) * R.at<float>(2,2)));
    vec.at<float>(2) = atan2(R.at<float>(1,0), R.at<float>(0,0));

    vec.at<float>(3) = T.at<float>(0);
    vec.at<float>(4) = T.at<float>(1);
    vec.at<float>(5) = T.at<float>(2);

    return vec;
}

Mat Chandeye::rodrigues_custom(Mat R)
{
    Mat wx = Mat::zeros(3,3,CV_32F);
    wx.at<float>(0,1) = -1*R.at<float>(2);
    wx.at<float>(0,2) = R.at<float>(1);
    wx.at<float>(1,0) = R.at<float>(2);
    wx.at<float>(1,2) = -1*R.at<float>(0);
    wx.at<float>(2,0) = -1*R.at<float>(1);
    wx.at<float>(2,1) = R.at<float>(0);

    float R1_norm = sqrt(double(pow(R.at<float>(0),2) + pow(R.at<float>(1),2) + pow(R.at<float>(2),2)));

    Mat first = wx * (sin(R1_norm)/R1_norm);
    float cte = (1-cos(R1_norm))/pow(R1_norm,2);
    Mat second = (wx * wx) * cte;
    Mat R2 = Mat::eye(3,3,CV_32F) + first + second;

    return R2;

}


Mat Chandeye::quat2rot(Mat q)
{
    //Mat q_tr = q.t();
    double p = q.dot(q);
    if(p > 1)
        ROS_ERROR("HANDEYE::quat2rot: quaternion greater than 1");

    float w = (float)sqrt(1 - p);
    Mat R = Mat::eye(4,4,CV_32F);

    Mat primero = q * q.t();
    Mat segundo = skew(q) * 2 * w;
    Mat tercero = diagonal((float)p) * 2;
    Mat tempR = (primero * 2) + segundo + Mat::eye(3,3,CV_32F) - tercero;

    for(int i = 0; i<3; i++)
        for(int j = 0; j<3; j++)
            R.at<float>(i,j) = tempR.at<float>(i,j);

    return R;
}

Mat Chandeye::diagonal(float p)
{
    Mat out = Mat::zeros(3,3,CV_32F);
    out.at<float>(0,0) = p;
    out.at<float>(1,1) = p;
    out.at<float>(2,2) = p;

    return out;
}

Mat Chandeye::skew(Mat V)
{
    Mat S = Mat::zeros(3,3, CV_32F);

    S.at<float>(0,1) = -1*V.at<float>(0,2);
    S.at<float>(0,2) = V.at<float>(0,1);
    S.at<float>(1,0) = V.at<float>(0,2);
    S.at<float>(1,2) = -1*V.at<float>(0,0);
    S.at<float>(2,0) = -1*V.at<float>(0,1);
    S.at<float>(2,1) = V.at<float>(0,0);

    return S;
}


Mat Chandeye::rot2quat(Mat R)
{
    Mat q = Mat::zeros(1,3, CV_32F);

    Mat R_temp(R, Range(0,3), Range(0,3));
    CvScalar t = trace(R_temp);
    float w4 = 2 * float(sqrt( 1.0 + t.val[0] )); // can this be imaginary?

    q.at<float>(0) = ( R.at<float>(2,1) - R.at<float>(1,2) ) / w4;
    q.at<float>(1) = ( R.at<float>(0,2) - R.at<float>(2,0) ) / w4;
    q.at<float>(2) = ( R.at<float>(1,0) - R.at<float>(0,1) ) / w4;

    return q;

}

Mat Chandeye::CrossProduct(Mat in)
{
    Mat out = Mat::zeros(3,3, CV_32F);

    out.at<float>(0,1) = -1*in.at<float>(2,0);
    out.at<float>(0,2) = in.at<float>(1,0);
    out.at<float>(1,0) = in.at<float>(2,0);
    out.at<float>(1,2) = -1*in.at<float>(0,0);
    out.at<float>(2,0) = -1*in.at<float>(1,0);
    out.at<float>(2,1) = in.at<float>(0,0);

    return out;
}

void Chandeye::printMatrix(Mat M, bool printType)
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
            cout << M.at<float>(i,j) << "\t";
        }
        cout<<endl;
    }
    cout<<endl;
}
