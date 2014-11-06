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
                    cout << M.at<double>(i,j) << " ";
                else
                    cout << M.at<float>(i,j) << " ";
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
    ros::init(argc, argv, "tf_echo", ros::init_options::AnonymousName);

    if (argc < 3)
    {
        printf("Usage: tf_echo source_frame target_frame filename[OPTIONAL]\n\n");
        printf("This will echo the transform from the coordinate frame of the source_frame\n");
        printf("to the coordinate frame of the target_frame. \n");
        printf("Note: This is the transform to get data from source_frame into the target_frame (Hsource2target).\n");
        return -1;
    }

    ros::NodeHandle nh;
    //Instantiate a local listener
    echoListener echoListener;


    std::string source_frameid = std::string(argv[1]);
    std::string target_frameid = std::string(argv[2]);

    // Wait for up to one second for the first transforms to become avaiable.
    echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

    //Nothing needs to be done except wait for a quit
    //The callbacks withing the listener class
    //will take care of everything
    Mat rotK;
    while(nh.ok())
    {
        try
        {
            tf::StampedTransform echo_transform;
            echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
            std::cout.precision(4);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
            //double yaw, pitch, roll;
            //echo_transform.getBasis().getEulerZYX(yaw,pitch,roll);
            //echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Quaternion q = echo_transform.getRotation();
            tf::Vector3 v = echo_transform.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;
                      //<< "            in ZYX [" <<  yaw << ", " << pitch << ", " << roll << "]" << std::endl;

            //print transform
            rotK = echoListener.getRotation(q.getX(), q.getY(), q.getZ(), q.getW(), v.getX(), v.getY(), v.getZ());
            std::cout << "H" << source_frameid << "2" << target_frameid << "\n";
            echoListener.printMatrix(rotK);

        }
        catch(tf::TransformException& ex)
        {
            std::cout << "Failure at "<< ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << echoListener.tf.allFramesAsString()<<std::endl;

        }
        sleep(1);
    }

    if(argc == 4)
    {
        std::string filename = std::string(argv[3]);

        string out_transform = ros::package::getPath("rh_calibration") + "/calibrations/" + filename;
        cout << "Saving transform to: " << out_transform.c_str() << endl;

        FileStorage fs( out_transform, FileStorage::WRITE );

        std::string variable = "H" + source_frameid + "2" + target_frameid;

        fs << variable << rotK;

        fs.release();
    }

    return 0;
};
