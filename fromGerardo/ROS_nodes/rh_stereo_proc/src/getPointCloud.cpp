#include <rh_stereo_proc/rh_stereo_proc.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include <math.h>
#include <sstream>

//Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";
static const char CAMERA_INFO_L[] = "/RH/left_camera/camera_info";
static const char CAMERA_INFO_R[] = "/RH/right_camera/camera_info";
static const char DISPARITY_H[] = "/RH/left_camera/disparityH"; // disparity on the x axis
static const char DISPARITY_V[] = "/RH/left_camera/disparityV"; // disparity on the y axis
static const char POINT_CLOUD[] = "/RH/left_camera/point_cloud";
static const char POINT_CLOUD_RES[] = "/RH/left_camera/point_cloud_resized";

class CdynamicCalibration
{

public:
    //Variables
    ros::Publisher output;

    bool save_cloud;
    int sampling;
    int resizeFactor;

    Mat_<double> K1_, D1_, P1_, F_;
    Mat_<double> K2_, D2_, P2_;
    Mat imgL, imgR, dispX, dispY;
    sensor_msgs::CameraInfo cam_infoL, cam_infoR;

    ros::Publisher pub_cloud_;
    ros::Publisher pub_cloud_resized_;

    CdynamicCalibration() :
        it_(nh_),
        imL_sub_(it_, CAM_SUB_LEFT, 5),
        imR_sub_(it_, CAM_SUB_RIGHT, 5), // TODO: include PTU and tf broadcaster
        infoL_sub_(nh_, CAMERA_INFO_L, 5),
        infoR_sub_(nh_, CAMERA_INFO_R, 5),
        dispH_sub_(nh_, DISPARITY_H, 5),
        dispV_sub_(nh_, DISPARITY_V, 5),
        sync_imgs(syncPolicy_imgs(5), imL_sub_, imR_sub_, infoL_sub_, infoR_sub_),
        sync_disp(syncPolicy_disp(5), dispH_sub_, dispV_sub_)
    {
        resizeFactor = 0.2;
        pub_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(POINT_CLOUD, 1);
        pub_cloud_resized_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(POINT_CLOUD_RES, 1);

        sync_imgs.registerCallback(boost::bind(&CdynamicCalibration::getImages, this, _1, _2, _3, _4));
        sync_disp.registerCallback(boost::bind(&CdynamicCalibration::getDisparities, this, _1, _2));
        ROS_INFO("Node initialised");
    }
    //Functions
    void getImages(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR, const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR);

    void getDisparities(const stereo_msgs::DisparityImageConstPtr& dispX_msg, const stereo_msgs::DisparityImageConstPtr& dispY_msg);

private:
    //Variables

    //ROS related stuff
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> syncPolicy_imgs;
    typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, stereo_msgs::DisparityImage> syncPolicy_disp;

    ImageSubscriber imL_sub_;
    ImageSubscriber imR_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoL_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoR_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> dispH_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> dispV_sub_;
    message_filters::Synchronizer<syncPolicy_imgs> sync_imgs;
    message_filters::Synchronizer<syncPolicy_disp> sync_disp;

    //Functions
    void printMatrix(Mat M, bool printType);
    std::string getImageType(int number);
    void saveXYZ(const char* filename, const Mat& mat);
    bool getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr doReconstructionRGB();
    pcl::PointCloud<pcl::PointXYZ>::Ptr doReconstruction();
    vector<float> get3DPoint(int xx, int yy);
    float getRangePoint(int xx, int yy);

};

void CdynamicCalibration::getImages(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR, const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR)
{
    // Get images
    cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
    ROS_INFO("Getting images and camera info...");
    try
    {
        cv_ptrL = cv_bridge::toCvCopy(imL, enc::RGB8);
        cv_ptrR = cv_bridge::toCvCopy(imR, enc::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'rgb8'.", imL->encoding.c_str());
        return;
    }

    imgL = cv_ptrL->image.clone();
    imgR = cv_ptrR->image.clone();

    ROS_INFO("Getting camera info");

    if(getCameraInfo(msgL, msgR))
    {
        ROS_INFO("Camera information read succesfully");
    }
    else
    {
        ROS_WARN("Camera information could not be read");
    }

    ROS_INFO("Done!!");

    return;

}

void CdynamicCalibration::getDisparities(const stereo_msgs::DisparityImageConstPtr& dispX_msg, const stereo_msgs::DisparityImageConstPtr& dispY_msg)
{
    // Get images
    cv_bridge::CvImagePtr cv_dispPtrH, cv_dispPtrV;

    ROS_INFO_STREAM("Rows: " << dispX_msg->image.height << " Cols: " << dispX_msg->image.width);
    ROS_INFO_STREAM("Rows: " << dispY_msg->image.height << " Cols: " << dispY_msg->image.width);

    ROS_INFO("Processing images...");
    try
    {
        cv_dispPtrH = cv_bridge::toCvCopy(dispX_msg->image, enc::TYPE_32FC1);
        cv_dispPtrV = cv_bridge::toCvCopy(dispY_msg->image, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert disparities to 'TYPE_32FC1'");
        return;
    }

    dispX = cv_dispPtrH->image.clone();
    dispY = cv_dispPtrV->image.clone();

    cv_dispPtrH->image.release();
    cv_dispPtrV->image.release();

    ROS_INFO("Creating Point Cloud...");
    if(dispX.size() != dispY.size())
    {
        ROS_ERROR("Disparity images are not the same in dimensions");
        return;
    }

    ROS_INFO_STREAM("Dimension of disparity images: " << dispX.rows << ", " << dispX.cols);

    if(pub_cloud_.getNumSubscribers() == 0)
    {
        ROS_WARN("No one is asking for a point cloud :)");
    }
    else
    {
        ROS_INFO("Computing range and point cloud!!");
        int64 t = getTickCount();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
        point_cloud_ptr = doReconstructionRGB();

        t = getTickCount() - t;
        ROS_INFO_STREAM(" DONE!");
        ROS_INFO("Time elapsed: %fms\n", t*1000/getTickFrequency());

        point_cloud_ptr->header.frame_id = "left_camera";
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        ROS_INFO_STREAM("Size point cloud: " << (int) point_cloud_ptr->points.size());

        if(save_cloud)
        {
            ROS_INFO("Saving point cloud to a file");
            pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_ptr);
        }

        ROS_INFO("DONE!");
        pub_cloud_.publish(*point_cloud_ptr);

    }

    if(pub_cloud_resized_.getNumSubscribers() == 0)
    {
        ROS_WARN("No one is asking for a resized point cloud :)");
    }
    else
    {
        ROS_INFO("Computing resized point cloud!!");
        int64 t = getTickCount();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
        point_cloud_ptr = doReconstructionRGB();

        t = getTickCount() - t;
        ROS_INFO_STREAM(" DONE!");
        ROS_INFO("Time elapsed: %fms\n", t*1000/getTickFrequency());

        point_cloud_ptr->header.frame_id = "left_camera";
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        ROS_INFO_STREAM("Size point cloud: " << (int) point_cloud_ptr->points.size());

        if(save_cloud)
        {
            ROS_INFO("Saving point cloud to a file");
            pcl::io::savePCDFileASCII ("test_pcd_resized.pcd", *point_cloud_ptr);
        }

        ROS_INFO("DONE!");
        pub_cloud_resized_.publish(*point_cloud_ptr);

    }

    return;

}

/* *************** PRIVATE FUNCTIONS *************** */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CdynamicCalibration::doReconstructionRGB()
{
    unsigned char pr, pg, pb;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_ (new pcl::PointCloud<pcl::PointXYZRGB>);

    int barWidth = 70;

    for(int ii = 0; ii < dispX.cols; ii++)
    {
        float progress = ((float)ii/(float)dispX.cols);
        int pos = barWidth * progress;
        std::cout << "[";
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();

        for(int jj = 0; jj < dispX.rows; jj++)
        {
            if(ii % sampling == 0 && jj % sampling == 0)
            {
                vector<float> point3D = get3DPoint(ii, jj);

                //Get RGB info
                unsigned char* rgb_ptr = imgL.ptr<unsigned char>(jj);// .ptr<uchar>(ii);
                pb = rgb_ptr[3*ii];
                pg = rgb_ptr[3*ii+1];
                pr = rgb_ptr[3*ii+2];

                pcl::PointXYZRGB point_small;
                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));

                point_small.x = point3D.at(0);
                point_small.y = point3D.at(1);
                point_small.z = point3D.at(2);
                point_small.rgb = *reinterpret_cast<float*>(&rgb);
                point_cloud_ptr_->points.push_back(point_small);
            }

        }
    }
    std::cout << std::endl;
    return point_cloud_ptr_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CdynamicCalibration::doReconstruction()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_ (new pcl::PointCloud<pcl::PointXYZ>);

    int barWidth = 70;
    Mat range_map(dispX.size(), CV_32F);

    ROS_INFO("Computing range map!");
    for(int ii = 0; ii < dispX.cols; ii++)
    {
        float progress = ((float)ii/(float)dispX.cols);
        int pos = barWidth * progress;
        std::cout << "[";
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();

        for(int jj = 0; jj < dispX.rows; jj++)
        {
            range_map.at<float>(jj,ii) = getRangePoint(ii, jj);
        }
    }
    std::cout << std::endl;
    ROS_INFO("Done!");

    ROS_INFO("Computing resized point cloud!");

    Mat res_range_map;
    resize(range_map, res_range_map, Size(dispX.cols/resizeFactor, dispX.rows/resizeFactor), 0, 0, cv::INTER_CUBIC);

    for(int ii = 0; ii < res_range_map.cols; ii++)
    {
        float progress = ((float)ii/(float)dispX.cols);
        int pos = barWidth * progress;
        std::cout << "[";
        for (int i = 0; i < barWidth; ++i)
        {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();

        for(int jj = 0; jj < res_range_map.rows; jj++)
        {
            vector<float> point3D = get3DPoint(ii, jj);

            pcl::PointXYZ point_small;
            point_small.x = point3D.at(0);
            point_small.y = point3D.at(1);
            point_small.z = point3D.at(2);
            point_cloud_ptr_->points.push_back(point_small);
        }

    }
    ROS_INFO("Done!");
    std::cout << std::endl;
    return point_cloud_ptr_;

}

vector<float> CdynamicCalibration::get3DPoint(int xx, int yy)
{
    vector<float> out;
    out.resize(0);
    float x1 = xx;
    float y1 = yy;
    float x2 = xx + dispX.at<float>(yy,xx);
    float y2 = yy + dispY.at<float>(yy,xx);

    float a, b, c, d, e, f, g, h, i, j, x, y;
    a = (float)P1_.at<double>(0,0);
    b = (float)(P1_.at<double>(0,2) - x1);
    c = (float)P1_.at<double>(1,1);
    d = (float)(P1_.at<double>(1,2) - y1);
    e = (float)(P2_.at<double>(0,0) - x2*P2_.at<double>(2,0));
    f = (float)(P2_.at<double>(0,1) - x2*P2_.at<double>(2,1));
    g = (float)(P2_.at<double>(0,2) - x2*P2_.at<double>(2,2));
    h = (float)(P2_.at<double>(1,0) - y2*P2_.at<double>(2,0));
    i = (float)(P2_.at<double>(1,1) - y2*P2_.at<double>(2,1));
    j = (float)(P2_.at<double>(1,2) - y2*P2_.at<double>(2,2));
    x = (float)(x2*P2_.at<double>(2,3) - P2_.at<double>(0,3));
    y = (float)(y2*P2_.at<double>(2,3) - P2_.at<double>(1,3));

    float XUp = (d*f*h - c*g*h - d*e*i + c*e*j)*(-(d*i*x) + c*j*x + d*f*y - c*g*y) +
                pow(b,2.0)*((f*h - e*i)*(-(i*x) + f*y) + pow(c,2.0)*(e*x + h*y)) +
                a*b*((-(g*i) + f*j)*(i*x - f*y) + c*d*(f*x + i*y) - pow(c,2.0)*(g*x + j*y));
    float YUp = (pow(b,2.0)*(f*h - e*i) + d*(d*f*h - c*g*h - d*e*i + c*e*j))*(h*x - e*y) +
                a*b*((c*d*e + g*h*i - 2.0*f*h*j + e*i*j)*x + (c*d*h + f*g*h - 2.0*e*g*i + e*f*j)*y) +
                pow(a,2.0)*((g*i - f*j)*(-(j*x) + g*y) + pow(d,2.0)*(f*x + i*y) - c*d*(g*x + j*y));
    float ZUp = c*(-(d*f*h) + c*g*h + d*e*i - c*e*j)*(h*x - e*y) -a*b*((f*h - e*i)*(-(i*x) + f*y) +
                pow(c,2.0)*(e*x + h*y)) + pow(a,2.0)*((g*i - f*j)*(i*x - f*y) - c*d*(f*x + i*y) +
                pow(c,2.0)*(g*x + j*y));
    float divisor = pow(b,2.0)*(pow(c,2.0)*(pow(e,2.0) + pow(h,2.0)) + pow(f*h - e*i,2.0)) +
                    pow(d*f*h - c*g*h - d*e*i + c*e*j,2.0) - 2.0*a*b*(-(c*d*(e*f + h*i)) +
                    (f*h - e*i)*(-(g*i) + f*j) + pow(c,2.0)*(e*g + h*j)) + pow(a,2.0)*
                    (pow(d,2.0)*(pow(f,2.0) + pow(i,2.0)) + pow(g*i - f*j,2.0) - 2.0*c*d*(f*g + i*j) +
                    pow(c,2.0)*(pow(g,2.0) + pow(j,2.0)));

    out.push_back(XUp/divisor);
    out.push_back(YUp/divisor);
    out.push_back(ZUp/divisor);
    return out;
}

float CdynamicCalibration::getRangePoint(int xx, int yy)
{
    float x1 = xx;
    float y1 = yy;
    float x2 = xx + dispX.at<float>(yy,xx);
    float y2 = yy + dispY.at<float>(yy,xx);

    float a, b, c, d, e, f, g, h, i, j, x, y;
    a = (float)P1_.at<double>(0,0);
    b = (float)(P1_.at<double>(0,2) - x1);
    c = (float)P1_.at<double>(1,1);
    d = (float)(P1_.at<double>(1,2) - y1);
    e = (float)(P2_.at<double>(0,0) - x2*P2_.at<double>(2,0));
    f = (float)(P2_.at<double>(0,1) - x2*P2_.at<double>(2,1));
    g = (float)(P2_.at<double>(0,2) - x2*P2_.at<double>(2,2));
    h = (float)(P2_.at<double>(1,0) - y2*P2_.at<double>(2,0));
    i = (float)(P2_.at<double>(1,1) - y2*P2_.at<double>(2,1));
    j = (float)(P2_.at<double>(1,2) - y2*P2_.at<double>(2,2));
    x = (float)(x2*P2_.at<double>(2,3) - P2_.at<double>(0,3));
    y = (float)(y2*P2_.at<double>(2,3) - P2_.at<double>(1,3));

    float ZUp = c*(-(d*f*h) + c*g*h + d*e*i - c*e*j)*(h*x - e*y) -a*b*((f*h - e*i)*(-(i*x) + f*y) +
                pow(c,2.0)*(e*x + h*y)) + pow(a,2.0)*((g*i - f*j)*(i*x - f*y) - c*d*(f*x + i*y) +
                pow(c,2.0)*(g*x + j*y));
    float divisor = pow(b,2.0)*(pow(c,2.0)*(pow(e,2.0) + pow(h,2.0)) + pow(f*h - e*i,2.0)) +
                    pow(d*f*h - c*g*h - d*e*i + c*e*j,2.0) - 2.0*a*b*(-(c*d*(e*f + h*i)) +
                    (f*h - e*i)*(-(g*i) + f*j) + pow(c,2.0)*(e*g + h*j)) + pow(a,2.0)*
                    (pow(d,2.0)*(pow(f,2.0) + pow(i,2.0)) + pow(g*i - f*j,2.0) - 2.0*c*d*(f*g + i*j) +
                    pow(c,2.0)*(pow(g,2.0) + pow(j,2.0)));

    return ZUp/divisor;
}

void CdynamicCalibration::printMatrix(Mat M, bool printType)
{
    if(printType)
        ROS_INFO_STREAM("Matrix type: " << getImageType(M.type()));
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

std::string CdynamicCalibration::getImageType(int number)
{
    // find type
    int imgTypeInt = number%8;
    std::string imgTypeString;

    switch (imgTypeInt)
    {
    case 0:
        imgTypeString = "8U";
        break;
    case 1:
        imgTypeString = "8S";
        break;
    case 2:
        imgTypeString = "16U";
        break;
    case 3:
        imgTypeString = "16S";
        break;
    case 4:
        imgTypeString = "32S";
        break;
    case 5:
        imgTypeString = "32F";
        break;
    case 6:
        imgTypeString = "64F";
        break;
    default:
        break;
    }

    // find channel
    int channel = (number/8) + 1;

    std::stringstream type;
    type<<"CV_"<<imgTypeString<<"C"<<channel;

    return type.str();
}

void CdynamicCalibration::saveXYZ(const char* filename, const Mat& mat)
{
    const float max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < DBL_EPSILON || fabs(point[2]) > max_z)
                continue;

            if(point[2] < 0)
                continue;

            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}


bool CdynamicCalibration::getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msgL, const sensor_msgs::CameraInfoConstPtr& msgR)
{
    try
    {
        // Update time stamp (and frame_id if that changes for some reason)

        cam_infoL.height = msgL->height;
        cam_infoL.width = msgL->width;
        cam_infoL.D = msgL->D;
        cam_infoL.K = msgL->K;
        cam_infoL.P = msgL->P;
        //cam_infoL.F = msgL->F;

        cam_infoR.height = msgR->height;
        cam_infoR.width = msgR->width;
        cam_infoR.D = msgR->D;
        cam_infoR.K = msgR->K;
        cam_infoR.P = msgR->P;
        //cam_infoR.F = msgR->F;

        Mat K1, D1, P1, K2, D2, P2;//, F;

        D1 = Mat::zeros(1,5,CV_64F);
        K1 = Mat::zeros(3,3,CV_64F);
        P1 = Mat::zeros(3,4,CV_64F);

        D2 = Mat::zeros(1,5,CV_64F);
        K2 = Mat::zeros(3,3,CV_64F);
        P2 = Mat::zeros(3,4,CV_64F);

        //F = Mat::zeros(3,3,CV_64F);

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

/* *************** MAIN PROGRAM *************** */
int main(int argc, char* argv[])
{
    ros::init( argc, argv, "RH_dynamicCalibration_node" );
    CdynamicCalibration rU_;

    rU_.sampling = 5;
    if((argc == 2) && (strcmp(argv[1], "-s") == 0))
    {
        ROS_INFO("Saving point cloud option is enabled!");
        rU_.save_cloud = true;
    }
    else
    {
        ROS_INFO("Use -s to save point cloud to disk");
        ROS_INFO("Saving point cloud option is not enabled!");
    }

    ROS_INFO_STREAM("Sampling every: " << rU_.sampling << " pixels!");

    ros::MultiThreadedSpinner s(2);
    ros::spin(s);

    return EXIT_SUCCESS;
}

// SLOW IMPLEMENTATION OF STEREO TRIANGULATION
//            Mat M(4,4,CV_32F);
//            vector<float> p1, p2;

//            p1.push_back(ii); // x1
//            p1.push_back(ji); // y1
//            p2.push_back(ii + dispX.at<float>(jj,ii)); // x2
//            p2.push_back(jj + dispY.at<float>(jj,ii)); // y2

//            M.at<float>(0,0) = (float)P1_.at<double>(0,0) - (p1.at(0) * (float)P1_.at<double>(2,0));
//            M.at<float>(0,1) = (float)P1_.at<double>(0,1) - (p1.at(0) * (float)P1_.at<double>(2,1));
//            M.at<float>(0,2) = (float)P1_.at<double>(0,2) - (p1.at(0) * (float)P1_.at<double>(2,2));
//            M.at<float>(0,3) = (float)P1_.at<double>(0,3) - (p1.at(0) * (float)P1_.at<double>(2,3));

//            M.at<float>(1,0) = (float)P1_.at<double>(1,0) - (p1.at(1) * (float)P1_.at<double>(2,0));
//            M.at<float>(1,1) = (float)P1_.at<double>(1,1) - (p1.at(1) * (float)P1_.at<double>(2,1));
//            M.at<float>(1,2) = (float)P1_.at<double>(1,2) - (p1.at(1) * (float)P1_.at<double>(2,2));
//            M.at<float>(1,3) = (float)P1_.at<double>(1,3) - (p1.at(1) * (float)P1_.at<double>(2,3));

//            M.at<float>(2,0) = (float)P2_.at<double>(0,0) - (p2.at(0) * (float)P2_.at<double>(2,0));
//            M.at<float>(2,1) = (float)P2_.at<double>(0,1) - (p2.at(0) * (float)P2_.at<double>(2,1));
//            M.at<float>(2,2) = (float)P2_.at<double>(0,2) - (p2.at(0) * (float)P2_.at<double>(2,2));
//            M.at<float>(2,3) = (float)P2_.at<double>(0,3) - (p2.at(0) * (float)P2_.at<double>(2,3));

//            M.at<float>(3,0) = (float)P2_.at<double>(1,0) - (p2.at(1) * (float)P2_.at<double>(2,0));
//            M.at<float>(3,1) = (float)P2_.at<double>(1,1) - (p2.at(1) * (float)P2_.at<double>(2,1));
//            M.at<float>(3,2) = (float)P2_.at<double>(1,2) - (p2.at(1) * (float)P2_.at<double>(2,2));
//            M.at<float>(3,3) = (float)P2_.at<double>(1,3) - (p2.at(1) * (float)P2_.at<double>(2,3));

//            SVD svd(M);

//            float div = svd.vt.at<float>(3,3);
//            px = svd.vt.at<float>(3,0)/div;
//            py = svd.vt.at<float>(3,1)/div;
//            pz = svd.vt.at<float>(3,2)/div;
