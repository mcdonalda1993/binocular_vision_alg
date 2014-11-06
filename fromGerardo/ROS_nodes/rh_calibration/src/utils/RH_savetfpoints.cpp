#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <rh_calibration/RHcalibration.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class echoListener
{
public:

    vector<Mat> bigMatQ, bigMatE;

    tf::TransformListener tf;

    //constructor with name
    echoListener()
    {

    }

    ~echoListener()
    {

    }

    void saveQ(double x, double y, double z, double qx, double qy, double qz, double qw, double yaw, double pitch, double roll)
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

private:

};


int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv, "RHsavetfpoints", ros::init_options::AnonymousName);

    if (argc != 4)
    {
        printf("Usage: RHsavetfpoints source_frame target_frame filename.py\n\n");
        printf("This will save robot transforms from the coordinate frame of the source_frame\n");
        printf("to the coordinate frame of the target_frame. \n");
        return -1;
    }

    ros::NodeHandle nh;
    //Instantiate a local listener
    echoListener echoListener;

    struct termios oldSettings, newSettings;

    tcgetattr( fileno( stdin ), &oldSettings );
    newSettings = oldSettings;
    newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &newSettings );

    std::string source_frameid = std::string(argv[1]);
    std::string target_frameid = std::string(argv[2]);
    
    std::string node_path = ros::package::getPath("rh_calibration");
    std::string filename = std::string(argv[3]);

    // Wait for up to one second for the first transforms to become avaiable.
    echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

    //Nothing needs to be done except wait for a quit
    //The callbacks withing the listener class
    //will take care of everything    
    while(nh.ok())
    {
        fd_set set;
        struct timeval tv;

        tv.tv_sec = 1;
        tv.tv_usec = 0;

        FD_ZERO( &set );
        FD_SET( fileno( stdin ), &set );
                
        ROS_WARN("Press 's' to save this pose or 'q' to shutdown the node");
        
        int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );
        
        try
        {
            tf::StampedTransform echo_transform;
            echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
            std::cout.precision(3);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
            double yaw, pitch, roll;
            //echo_transform.getBasis().getEulerZYX(yaw,pitch,roll);
            echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Quaternion q = echo_transform.getRotation();
            tf::Vector3 v = echo_transform.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl
                      << "            in RPY [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl;

            //save stuff
            if(res)
            {
                char c;
                ssize_t temp = read( fileno( stdin ), &c, 1 );
                //ROS_INFO_STREAM("Input available: " << temp);
                if(c == 's')
                {
                    echoListener.saveQ(v.getX(), v.getY(), v.getZ(), q.getX(), q.getY(), q.getZ(), q.getW(), yaw, pitch, roll);
                    ROS_INFO("Robot pose saved!!!!");
                }
                else if(c == 'q')
                {
                    break;
                }
                else
                {
                    ROS_ERROR_STREAM("Pressed '" << c << "'!!");
                }
            }
        }
        catch(tf::TransformException& ex)
        {
            std::cout << "Failure at "<< ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << echoListener.tf.allFramesAsString()<<std::endl;

        }
        //sleep(1);
    }
    
    string outputFile = node_path + "/" + filename;
//    FileStorage fs( outputFile, FileStorage::WRITE );
//    
//    cvWriteComment(*fs, "[x, y, z, qx, qy, qz, qw]", 0);
//    fs << "quaternions" << echoListener.bigMatQ;
//    
//    cvWriteComment(*fs, "[x, y, z, r, p, y]", 0);
//    fs << "euler_angles" << echoListener.bigMatE;
//    
//    fs.release();

    std::ofstream myfile;
    myfile.open (outputFile.c_str(), std::ios::app);
    for(unsigned int i=0; i < echoListener.bigMatQ.size(); i++)
    {
        Mat temp = echoListener.bigMatQ.at(i);
        myfile << "<replace_for_variable>.append([";
        for(int j=0; j < temp.cols; j++)
        {
            if(j < temp.cols - 1)
                myfile << temp.at<float>(0,j) << ", ";
            else
                myfile << temp.at<float>(0,j) << "])\n";
        }
    }
    myfile.close();
    
    std::cout << "File was saved in " << outputFile << endl;

    tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
    return 0;
}
