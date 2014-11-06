#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <rh_calibration/RHcalibration.h>
#include <tf_conversions/tf_kdl.h>

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


int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "solve_handeye");

    ros::NodeHandle nh;
    //Instantiate a local listener
    echoListener listener1; // Hworld2camL
    echoListener listener2; // Hworld2ptuR
    echoListener listener3; // HcamL2ptuR
    echoListener listener4; // Hworld2ptuL

    std::string world = "base_link";
    std::string left_camera = "left_camera";
    std::string right_tilt_optic = "right_tilt_optic";
    std::string left_tilt_optic = "left_tilt_optic";
    

    //Nothing needs to be done except wait for a quit
    //The callbacks withing the listener class
    //will take care of everything
    Mat rotK, trans;
    
    Mat Hworld2camL, Hworld2ptuR, HcamL2ptuR, Hworld2ptuL;
    while(nh.ok())
    {
        try
        {
        	// Hworld2camL
            tf::StampedTransform echo1;
            listener1.tf.lookupTransform(world, left_camera, ros::Time(), echo1);
            std::cout.precision(5);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "From " << world << " to " << left_camera << " at time " << echo1.stamp_.toSec() << std::endl;
            tf::Quaternion q = echo1.getRotation();
            tf::Vector3 v = echo1.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;

            Hworld2camL = listener1.getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
            std::cout << "Hworld2camL\n";
            listener1.printMatrix(Hworld2camL);
            
            // Hworld2ptuR
            tf::StampedTransform echo2;
            listener2.tf.lookupTransform(world, right_tilt_optic, ros::Time(), echo2);
            std::cout.precision(5);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "From " << world << " to " << right_tilt_optic << " at time " << echo2.stamp_.toSec() << std::endl;
            q = echo2.getRotation();
            v = echo2.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;

            Hworld2ptuR = listener2.getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
            std::cout << "Hworld2ptuR\n";
            listener2.printMatrix(Hworld2ptuR);
            
            // HcamL2ptuR
            tf::StampedTransform echo3;
            listener3.tf.lookupTransform(right_tilt_optic, left_camera, ros::Time(), echo3);
            std::cout.precision(5);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "From " << right_tilt_optic << " to " << left_camera << " at time " << echo3.stamp_.toSec() << std::endl;
            q = echo3.getRotation();
            v = echo3.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;

            HcamL2ptuR = listener3.getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
            std::cout << "HcamL2ptuR\n";
            listener3.printMatrix(HcamL2ptuR);
            
            // Hworld2ptuL
            tf::StampedTransform echo4;
            listener4.tf.lookupTransform(world, left_tilt_optic, ros::Time(), echo4);
            std::cout.precision(5);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "From " << world << " to " << left_tilt_optic << " at time " << echo4.stamp_.toSec() << std::endl;
            q = echo4.getRotation();
            v = echo4.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;

            Hworld2ptuL = listener4.getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
            std::cout << "Hworld2ptuL\n";
            listener4.printMatrix(Hworld2ptuL);
            
            
            std::cout << "-----------------------------\n";
            std::cout << "-----------------------------\n";
            
            
            

        }
        catch(tf::TransformException& ex)
        {
            std::cout << "Failure at "<< ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << listener1.tf.allFramesAsString()<<std::endl;

        }
        sleep(1);
    }

    return 0;
};
