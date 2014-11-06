#include <RH_stereoAddOns/RH_stereoAddOns.h>
#include <RH_stereoAddOns/stereoRectify.h>
#include <RH_siftgpu/SiftGPU_msg.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

bool useRHCalib = true;

//Parameter Server
static const char INPUT_XML[] = "/RH/calibration/stereo_input";

//Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";
static const char FEAT_LEFT[] = "/RH/left_camera/features";
static const char FEAT_RIGHT[] = "/RH/right_camera/features";

static const char WINDOW_LEFT[] = "RH camera left";
static const char WINDOW_RIGHT[] = "RH camera right";
static const char MATCHES[] = "Stereo matches";

class Cget3dPoints
{

protected:
	//Variables
	//image_transport::Publisher image_pub_left, image_pub_right;

public:

	ros::Publisher output;
	CRectification r_;
	
	Mat leftKeys, rightKeys;
	
	float scale;
	Mat K1, D1, K2, D2;
	Mat R, T, E, F, R1, P1, R2, P2;//, Q;

	Cget3dPoints() :
	it_(nh_),
	imL_sub_(it_, CAM_SUB_LEFT, 5),
    imR_sub_(it_, CAM_SUB_RIGHT, 5),
    featL_sub_(nh_, FEAT_LEFT, 5),
	featR_sub_(nh_, FEAT_RIGHT, 5),
	sync(syncPolicy(5), imL_sub_, imR_sub_, featL_sub_, featR_sub_)
	{
		if(useRHCalib)
		{
			std::string param_str;
			if (nh_.hasParam(INPUT_XML))
			{
				nh_.getParam(INPUT_XML, param_str);
			}
			else
			{
				ROS_ERROR("RH_calibration node exception: \"%s\" parameter is not set in the server", INPUT_XML);
				return;
			}
		
			// reading intrinsic parameters
		    FileStorage fs(param_str.c_str(), FileStorage::READ);
		    if(!fs.isOpened())
		    {
		        printf("Failed to open file %s\n", param_str.c_str());
		        ros::shutdown();
		    }
		    
		    fs["Camera_Matrix_Left"] >> K1;
		    fs["Distortion_Coefficients_Left"] >> D1;
		    fs["Camera_Matrix_Right"] >> K2;
		    fs["Distortion_Coefficients_Right"] >> D2;
		    
		    fs["R"] >> R;
		    fs["T"] >> T;
		    fs["E"] >> E;
		    fs["F"] >> F;
        }
        else
        {
        
        }
        
        namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
		resizeWindow(WINDOW_LEFT, 640, 480);
		namedWindow(WINDOW_RIGHT, CV_WINDOW_NORMAL);
		resizeWindow(WINDOW_RIGHT, 640, 480);
		namedWindow(MATCHES, CV_WINDOW_NORMAL);
		resizeWindow(MATCHES, 640, 480);
		startWindowThread();
		
		//image_pub_left = it_.advertise(CAM_PUB_LEFT_RECTIFIED, 1);
		//image_pub_right = it_.advertise(CAM_PUB_RIGHT_RECTIFIED, 1);
		sync.registerCallback(boost::bind(&Cget3dPoints::projectPoints, this, _1, _2, _3, _4));
		ROS_INFO("Node initialised");
	}
    //Functions
	void projectPoints(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR, const RH_siftgpu::SiftGPU_msg::ConstPtr& featL, const RH_siftgpu::SiftGPU_msg::ConstPtr& featR);

private:
    //Variables
    Mat descL;
    Mat descR;
    vector< KeyPoint > keysL;
    vector< KeyPoint > keysR;
    vector< DMatch > matches;

    //Functions
    void printMatrix(Mat M, bool printType);
    void siftgpu2opencv(const RH_siftgpu::SiftGPU_msg::ConstPtr& featL, const RH_siftgpu::SiftGPU_msg::ConstPtr& featR);
    double TriangulatePoints(const vector<KeyPoint>& pt_set1, const vector<KeyPoint>& pt_set2, const Mat& Kinv, const Matx34d& P, const Matx34d& P1, vector<Point3d>& pointcloud);
    Mat_<double> LinearLSTriangulation( Point3d u, Matx34d P, Point3d u1, Matx34d P1);
    Mat_<double> IterativeLinearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1);
    
    //ROS related stuff
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, RH_siftgpu::SiftGPU_msg, RH_siftgpu::SiftGPU_msg> syncPolicy;

    ImageSubscriber imL_sub_;
    ImageSubscriber imR_sub_;
    message_filters::Subscriber<RH_siftgpu::SiftGPU_msg> featL_sub_;
    message_filters::Subscriber<RH_siftgpu::SiftGPU_msg> featR_sub_;

    message_filters::Synchronizer<syncPolicy> sync;

};

void Cget3dPoints::projectPoints(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR, const RH_siftgpu::SiftGPU_msg::ConstPtr& featL, const RH_siftgpu::SiftGPU_msg::ConstPtr& featR)
{
	// Get images
    cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
    
	try
	{
	    cv_ptrL = cv_bridge::toCvCopy(imL, enc::RGB8);
	    cv_ptrR = cv_bridge::toCvCopy(imR, enc::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", imL->encoding.c_str());
	    return;
	}
	
	siftgpu2opencv(featL, featR);
	
	ROS_INFO_STREAM(keysL.size() << " " << leftKeys.rows);
	
	Mat recImg1, recImg2, uKeysL, uKeysR;
    //r_.rectifyImgs(cv_ptrL->image, cv_ptrR->image, K1, K2, D1, D2, R, T, recImg1, recImg2);
    
    ROS_INFO("P1:");
    printMatrix(r_.P1, false);
    ROS_INFO("P2:");
    printMatrix(r_.P2, false);
	
	//dKeysL.convertTo(leftKeys, CV_32FC2);
	//dKeysR.convertTo(rightKeys, CV_32FC2);
	
	undistortPoints(leftKeys, uKeysL, K1, D1, r_.R1, r_.P1);
	undistortPoints(rightKeys, uKeysR, K2, D2, r_.R2, r_.P2);
	
	Mat outL, outR;
	// Keypoints are not drawn according to the orientation... needs to be checked but results are consistent with Lowe's method
	
	vector< KeyPoint > tempL, tempR;
	tempL.resize(0);
	tempR.resize(0);
	for(int i = 0; i < uKeysL.rows; i++)
	{
		cv::KeyPoint kptL, kptR;
		kptL.pt.x = uKeysL.at<float>(i, 0);
		kptL.pt.y = uKeysL.at<float>(i, 1);
		kptL.size = 100;
		kptL.angle = 0;
	    kptL.response = 0; kptL.octave = 0; kptL.class_id = 0;
	    
	    kptR.pt.x = uKeysR.at<float>(i, 0);
		kptR.pt.y = uKeysR.at<float>(i, 1);
		kptR.size = 100;
		kptR.angle = 0;
	    kptR.response = 0; kptR.octave = 0; kptR.class_id = 0;
	    
	    tempL.push_back(kptL);
	    tempR.push_back(kptR);
	}
	
	drawKeypoints(recImg1, tempL, outL, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	drawKeypoints(recImg2, tempR, outR, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	
	std::vector< DMatch > good_matches;
	for( int i = 0; i < uKeysL.rows; i++ )
		good_matches.push_back(DMatch(i, i, 0, 1));
	Mat img_matches;
	drawMatches(recImg1, tempL, recImg2, tempR, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
	imshow(WINDOW_LEFT, outL);
	imshow(WINDOW_RIGHT, outR);
	imshow(MATCHES, img_matches);
	ROS_INFO("Keypoints drawn!");
	
	Mat hKeysL, hKeysR;
	convertPointsToHomogeneous(uKeysL, hKeysL);
	convertPointsToHomogeneous(uKeysR, hKeysR);
	
	vector<Point3d> pointCloud;
	pointCloud.resize(0);
	TriangulatePoints(tempL, tempR, K1.inv(), r_.P1, r_.P2, pointCloud);
	
	std::cout << "Creating Point Cloud..." <<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  	
  	double px, py, pz;

	for (size_t i = 0; i < pointCloud.size(); i++)
    {
        Point3d pt3 = pointCloud[i];
	 	pcl::PointXYZ point;
		point.x = pt3.x;
		point.y = pt3.y;
		point.z = pt3.z;

		point_cloud_ptr->points.push_back(point);
	}
  	
  	point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
  	
  	pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_ptr);
  	ROS_INFO("Done!");
	
	return;	
	
}

/* *************** PRIVATE FUNCTIONS *************** */
Mat_<double> Cget3dPoints::IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
                                            ) {
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1);
    float EPSILON = 0.01;
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
         
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
         
        //breaking point
        if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
    }
    return X;
}

//homogenous image point (u,v,1)
//camera 1 matrix
//homogenous image point in 2nd camera
//camera 2 matrix
Mat_<double> Cget3dPoints::LinearLSTriangulation( Point3d u, Matx34d P, Point3d u1, Matx34d P1)
{
	//build A matrix
	 Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
	u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
	u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
	u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2)
	 );
	//build B vector
	Matx41d B(-(u.x*P(2,3)-P(0,3)),
	 -(u.y*P(2,3)-P(1,3)),
	 -(u1.x*P1(2,3)-P1(0,3)),
	 -(u1.y*P1(2,3)-P1(1,3)));
	//solve for X
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	
	return X;
}

double Cget3dPoints::TriangulatePoints(const vector<KeyPoint>& pt_set1, const vector<KeyPoint>& pt_set2, const Mat& Kinv, const Matx34d& P, const Matx34d& P1, vector<Point3d>& pointcloud)
{
	vector<double> reproj_error;
	unsigned int pts_size = pt_set1.size();
	ROS_INFO("Triangulating points");
	for (unsigned int i=0; i<pts_size; i++)
	{
		//convert to normalized homogeneous coordinates
		Point2f kp = pt_set1[i].pt;
		Point3d u(kp.x,kp.y,1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u = um.at<Point3d>(0);
		Point2f kp1 = pt_set2[i].pt;
		Point3d u1(kp1.x,kp1.y,1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1 = um1.at<Point3d>(0);
		//triangulate
		Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);
		//calculate reprojection error
//		Mat_<double> xPt_img = Kinv * Mat(P1) * X;
//		ROS_INFO("1");
//		Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
//		reproj_error.push_back(norm(xPt_img_-kp1));
//		//store 3D point
		pointcloud.push_back(Point3d(X(0),X(1),X(2)));
	}
	//return mean reprojection error
//	Scalar me = mean(reproj_error);
	return 1;//me[0];
}


void Cget3dPoints::printMatrix(Mat M, bool printType)
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

void Cget3dPoints::siftgpu2opencv(const RH_siftgpu::SiftGPU_msg::ConstPtr& featL, const RH_siftgpu::SiftGPU_msg::ConstPtr& featR)
{

	ROS_INFO("Features in left camera msg time: %f", featL->header.stamp.toSec());
	ROS_INFO("Features in right camera msg time: %f", featR->header.stamp.toSec());
	
	vector<float> desc;
	
	// Left features and descriptors
	int16_t num = featL->numOfFeatures;
	ROS_INFO("Reading %d features in the left camera", num);
	std::vector<float>::const_iterator it = featL->keypoints.begin();
	keysL.resize(0);
	for(int i = 0; i < num; i++)
	{
		cv::KeyPoint kpt;
		kpt.pt.x = *it; it++;
		kpt.pt.y = *it; it++;
		kpt.size = *it; it++;
		kpt.angle = *it; it++;
	    kpt.response = 0; kpt.octave = 0; kpt.class_id = 0;
	    keysL.push_back(kpt);
	}
	
	descL = Mat(featL->descriptors).reshape(1, num);
	
	matches.resize(0);
	std::vector<int16_t>::const_iterator it_m = featL->matches.begin();
	for(int i = 0; i < (int)featL->matches.size()/2; i++)
	{
		int16_t query = *it_m; it_m++;
		int16_t train = *it_m; it_m++;
		matches.push_back(DMatch((int)query, (int)train, 0, 0));
	}
	ROS_INFO("Size of matches from SIFT_GPU: %d", (int)matches.size());
	
	// Right features
	num = featR->numOfFeatures;
	ROS_INFO("Reading %d features in the left camera", num);
	it = featR->keypoints.begin();
	keysR.resize(0);
	for(int i = 0; i < num; i++)
	{
		cv::KeyPoint kpt;
		kpt.pt.x = *it; it++;
		kpt.pt.y = *it; it++;
		kpt.size = *it; it++;
		kpt.angle = *it; it++;
	    kpt.response = 0; kpt.octave = 0; kpt.class_id = 0;
	    keysR.push_back(kpt);
	}
	
	descR = Mat(featR->descriptors).reshape(1, num);
	
	// Get matched list
	
	if((int)matches.size() == 0)
	{
		ROS_DEBUG("Creating FLANN database (binocular matching)");
		flann::Index treeFlannIndex(descL, flann::KDTreeIndexParams());
		int k = 2;
		Mat results(descR.rows, k, CV_32SC1);
		Mat dists(descR.rows, k, CV_32FC1);

		ROS_DEBUG("Searching flan database (binocular matching)");
		treeFlannIndex.knnSearch(descR, results, dists, k, flann::SearchParams());

		float distTreshold = 0.6;
		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		ROS_INFO("Nearest neighbours with log-likelihood ratio of: %f", distTreshold);
		matches.resize(0);
		for(int i=0; i < descR.rows; ++i)
		{
		    if(dists.at<float>(i,0) <= distTreshold * dists.at<float>(i,1))
		    {
		        matches.push_back(DMatch( results.at<int>(i,0), i, 0, dists.at<float>(i,0)));

		    }
		}
    }
    else
    {
    	ROS_INFO("Using SIFT GPU matches");
    }

    ROS_INFO("Number of matches: %d", (int)matches.size());
    vector<Point2f> left, right;
    left.resize(0);
    right.resize(0);
    for (size_t i = 0; i < matches.size(); i++)
    {
        const DMatch& match = matches[i];
	 	left.push_back(keysL[match.queryIdx].pt);
	 	right.push_back(keysR[match.trainIdx].pt);
	}
	
	leftKeys = Mat(left).clone();
	rightKeys = Mat(right).clone();
	
	ROS_INFO_STREAM("Features in left camera after matching: " << leftKeys.rows);
	ROS_INFO_STREAM("Features in right camera after matching: " << rightKeys.rows);
}

/* *************** MAIN PROGRAM *************** */
int main(int argc, char** argv)
{
    ros::init( argc, argv, "RH_get3dPoints_node" );
    Cget3dPoints rU_;

    while( ros::ok() )
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}

