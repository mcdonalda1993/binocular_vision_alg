#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

static const char LEFT_TILT[] = "left_tilt_ptu";
static const char RIGHT_TILT[] = "right_tilt_ptu";

static const char LEFT_CAMERA[] = "left_camera";
static const char RIGHT_CAMERA[] = "right_camera";

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

int main(int argc, char* argv[]){
    ros::init(argc, argv, "publish_HE_transform");
    ros::NodeHandle nh_;

    ROS_INFO_STREAM(argc);

    if(argc != 2)
    {
        ROS_FATAL("publish_HE_transform: filename containing the transformation between tilt and camera must be given!");
        nh_.shutdown();
        return -1;
    }

    ROS_INFO_STREAM(argv[0]);
    ROS_INFO_STREAM(argv[1]);

    FileStorage fs(argv[1], FileStorage::READ); // Read the matrices

    Mat matL, matR;

    if (!fs.isOpened())
    {
        ROS_ERROR_STREAM("Could not open file: \"" << argv[1] << "\"");
        nh_.shutdown();
        return -1;
    }

    fs["gripperHcamera_LeftCamera"] >> matL;
    fs["gripperHcamera_RightCamera"] >> matR;

    ROS_INFO("Left transformation:");
    printMatrix(matL,true);

    ROS_INFO("Right transformation:");
    printMatrix(matR,true);

    fs.release(); // close Settings file
    ROS_INFO("File read!");

    tf::TransformBroadcaster brL, brR;
    tf::Transform transformL, transformR;

    tf::Quaternion qL, qR;

    tf::Matrix3x3 m_L(matL.at<float>(0,0), matL.at<float>(0,1), matL.at<float>(0,2),
                      matL.at<float>(1,0), matL.at<float>(1,1), matL.at<float>(1,2),
                      matL.at<float>(2,0), matL.at<float>(2,1), matL.at<float>(2,2));

    tf::Matrix3x3 m_R(matR.at<float>(0,0), matR.at<float>(0,1), matR.at<float>(0,2),
                      matR.at<float>(1,0), matR.at<float>(1,1), matR.at<float>(1,2),
                      matR.at<float>(2,0), matR.at<float>(2,1), matR.at<float>(2,2));

    tf::Vector3 tL(matL.at<float>(0,3), matL.at<float>(1,3), matL.at<float>(2,3));
    tf::Vector3 tR(matR.at<float>(0,3), matR.at<float>(1,3), matR.at<float>(2,3));

    m_L.getRotation(qL);
    m_R.getRotation(qR);

    ROS_INFO_STREAM(qL.getX() << ", " << qL.getY() << ", " << qL.getZ() << ", " << qL.getW());
    ROS_INFO_STREAM(qR.getX() << ", " << qR.getY() << ", " << qR.getZ() << ", " << qR.getW());

    ros::Rate rate(10.0);
    while (nh_.ok()){
        transformL.setOrigin(tL);
        transformL.setRotation(qL);

        transformR.setOrigin(tR);
        transformR.setRotation(qR);

        brL.sendTransform(tf::StampedTransform(transformL, ros::Time::now(), LEFT_TILT, LEFT_CAMERA));
        brR.sendTransform(tf::StampedTransform(transformR, ros::Time::now(), RIGHT_TILT, RIGHT_CAMERA));

        rate.sleep();
    }
    return 0;

}
