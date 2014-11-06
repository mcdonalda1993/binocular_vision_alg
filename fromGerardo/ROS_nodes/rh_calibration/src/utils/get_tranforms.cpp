
#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <rh_calibration/RHcalibration.h>
#include <tf_conversions/tf_kdl.h>
#include <rh_cameras/CamerasSync.h>

// Messages
static const char CAM_ACQUIRE[] = "/RH/cmd/acquire";

int counter;

class echoListener
{
public:

    tf::TransformListener tf;

    //constructor with name
    echoListener()
    {

    }

    ~echoListener()
    {

    }

    Mat getTransformation(const std::string &target_frame, const std::string &source_frame)
    {
        tf::StampedTransform echo1;
        tf.lookupTransform(target_frame, source_frame, ros::Time(), echo1);
        std::cout.precision(5);
        std::cout << "From " << target_frame << " to " << source_frame << " at time " << echo1.stamp_.toSec() << std::endl;
        tf::Quaternion q = echo1.getRotation();
        tf::Vector3 v = echo1.getOrigin();

        return getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
    }

    Mat getRotation(double x, double y, double z, double w, double tx, double ty, double tz)
    {
        Mat rotK = Mat::zeros(4,4,CV_32F);

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

        rotK.at<float>(0,3) = (float)tx;
        rotK.at<float>(1,3) = (float)ty;
        rotK.at<float>(2,3) = (float)tz;

        rotK.at<float>(3,3) = 1.0;

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
                    cout << M.at<double>(i,j) << "  ";
                else
                    cout << M.at<float>(i,j) << "  ";
            }
            cout<<endl;
        }
        cout<<endl;
    }

private:

};

void xtionCallback(const rh_cameras::CamerasSync::ConstPtr& msg)
{
    std::string world = "base_link";
    std::string left_base = "left_base";
    std::string right_base = "right_base";
    std::string left_pan = "left_pan";
    std::string right_pan = "right_pan";
    std::string left_tilt = "left_tilt";
    std::string right_tilt = "right_tilt";

    //Instantiate a local listener
    echoListener listener; // Hworld2camL

    Mat Hw2b_left = listener.getTransformation(world, left_base);
    Mat Hb2p_left = listener.getTransformation(left_base, left_pan);
    Mat Hp2t_left = listener.getTransformation(left_pan, left_tilt);

    Mat Hw2b_right = listener.getTransformation(world, right_base);
    Mat Hb2p_right = listener.getTransformation(right_base, right_pan);
    Mat Hp2t_right = listener.getTransformation(right_pan, right_tilt);

    stringstream ss;
    ss << counter;
    counter++;

    string outputFile = ros::package::getPath("rh_calibration") + "/calibrations/output/" + ss.str();

    ROS_INFO_STREAM("File is saved in: " << outputFile);

    FileStorage fs( outputFile, FileStorage::WRITE );

    cvWriteComment( *fs, "**LEFT CAMERA**", 0 );
    fs << "Hw2b_left" << Hw2b_left;
    fs << "Hb2p_left" << Hb2p_left;
    fs << "Hp2t_left" << Hp2t_left;

    cvWriteComment( *fs, "**RIGHT CAMERA**", 0 );
    fs << "Hw2b_right" << Hw2b_right;
    fs << "Hb2p_right" << Hb2p_right;
    fs << "Hp2t_right" << Hp2t_right;

    fs.release();

    return;

}

int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "refine_handeye");

    ros::NodeHandle nh;
    counter = 1;

    ros::Subscriber sub_ = nh.subscribe(CAM_ACQUIRE, 1, xtionCallback);
    ros::spin();


    return 0;
}

