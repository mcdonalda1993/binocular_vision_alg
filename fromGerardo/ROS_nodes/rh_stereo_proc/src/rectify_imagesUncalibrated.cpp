#include <rh_stereo_proc/rh_stereo_proc.h>
#include <rh_feature_extraction/SiftGPU.h>

float matchTreshold = 0.7;

//Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";
static const char FEAT_LEFT[] = "/RH/left_camera/features";
static const char FEAT_RIGHT[] = "/RH/right_camera/features";
static const char CAM_PUB_LEFT_RECTIFIED[] = "left_image_rectified";
static const char CAM_PUB_RIGHT_RECTIFIED[] = "right_image_rectified";

struct stereoFeat_s
{
    Mat descL;
    Mat descR;
    vector< KeyPoint > keysL;
    vector< KeyPoint > keysR;
    vector< DMatch > matches;
    Mat imgLeft;
    Mat imgRight;
} ;

class CrectifyUncalibrated
{

protected:
	//Variables
	stereoFeat_s featuresStereo; // opencv keypoint structure
	vector<Point2f> leftPts, rightPts;
	image_transport::Publisher image_pub_left, image_pub_right;

public:

	ros::Publisher output;

	CrectifyUncalibrated() :
	it_(nh_),
	imL_sub_(it_, CAM_SUB_LEFT, 5),
    imR_sub_(it_, CAM_SUB_RIGHT, 5),
	featL_sub_(nh_, FEAT_LEFT, 5),
	featR_sub_(nh_, FEAT_RIGHT, 5),
	sync(syncPolicy(5), imL_sub_, imR_sub_, featL_sub_, featR_sub_)
	{
		image_pub_left = it_.advertise(CAM_PUB_LEFT_RECTIFIED, 1);
		image_pub_right = it_.advertise(CAM_PUB_RIGHT_RECTIFIED, 1);
		sync.registerCallback(boost::bind(&CrectifyUncalibrated::rectify, this, _1, _2, _3, _4)); // Uncomment for debug
		ROS_INFO("Node initialised");
	}
    //Functions
	void rectify(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR,
		const rh_feature_extraction::SiftGPU::ConstPtr& featL, const rh_feature_extraction::SiftGPU::ConstPtr& featR);

private:
    //Variables

    //Functions
    int getMatches(float distTreshold); // For stereo matching
    void siftgpu2opencv(const rh_feature_extraction::SiftGPU::ConstPtr& featL, const rh_feature_extraction::SiftGPU::ConstPtr& featR);
    
    //ROS related stuff
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, rh_feature_extraction::SiftGPU, rh_feature_extraction::SiftGPU> syncPolicy;

    ImageSubscriber imL_sub_;
    ImageSubscriber imR_sub_;
    message_filters::Subscriber<rh_feature_extraction::SiftGPU> featL_sub_;
    message_filters::Subscriber<rh_feature_extraction::SiftGPU> featR_sub_;
    message_filters::Synchronizer<syncPolicy> sync;

};

void CrectifyUncalibrated::rectify(const sensor_msgs::ImageConstPtr& imL, const sensor_msgs::ImageConstPtr& imR,
		const rh_feature_extraction::SiftGPU::ConstPtr& featL, const rh_feature_extraction::SiftGPU::ConstPtr& featR)
{
	// Get images
    cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
    Mat F;
    
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
	
	featuresStereo.imgLeft = cv_ptrL->image;
	featuresStereo.imgRight = cv_ptrR->image;
	
	// Get features
	siftgpu2opencv(featL, featR);
	
	ROS_INFO("Matching descriptors between cameras");
    getMatches(matchTreshold);
    
    for( int i = 0; i < (int)featuresStereo.matches.size(); i++ )
	{
		// Get the keypoints from the good matches
		leftPts.push_back(featuresStereo.keysL[featuresStereo.matches[i].queryIdx].pt);
		rightPts.push_back(featuresStereo.keysR[featuresStereo.matches[i].trainIdx ].pt);
	}
	
	//F = findFundamentalMat(leftPts, rightPts, CV_FM_7POINT);
	//F = findFundamentalMat(leftPts, rightPts, CV_FM_8POINT);
	F = findFundamentalMat(leftPts, rightPts, CV_FM_RANSAC, 1.0, 0.99999);
	//F = findFundamentalMat(leftPts, rightPts, CV_FM_LMEDS, 1.0, 0.99999);
	
	Mat H1(4,4, featuresStereo.imgLeft.type());
	Mat H2(4,4, featuresStereo.imgLeft.type());
	stereoRectifyUncalibrated(leftPts, rightPts, F, featuresStereo.imgLeft.size(), H1, H2);

	Mat rectifiedLeft(featuresStereo.imgLeft.size(), featuresStereo.imgLeft.type());
	warpPerspective(featuresStereo.imgLeft, rectifiedLeft, H1, featuresStereo.imgLeft.size());

	Mat rectifiedRight(featuresStereo.imgRight.size(), featuresStereo.imgRight.type());
	warpPerspective(featuresStereo.imgRight, rectifiedRight, H2, featuresStereo.imgRight.size());
	
	//Publish Left camera
	cv_bridge::CvImage cvi_left;
    cvi_left.header.stamp = imL->header.stamp;
    cvi_left.header.frame_id = "camera_left_rectified";
    cvi_left.encoding = "rgb8";
    cvi_left.image = rectifiedLeft;
    
    imwrite("rectifiedLeft.png", rectifiedLeft);
    
    image_pub_left.publish(cvi_left.toImageMsg());
    
    //Publish Right camera
	cv_bridge::CvImage cvi_right;
    cvi_right.header.stamp = imR->header.stamp;
    cvi_right.header.frame_id = "camera_right_rectified";
    cvi_right.encoding = "rgb8";
    cvi_right.image = rectifiedRight;
    
    imwrite("rectifiedRight.png", rectifiedRight);
    
    image_pub_left.publish(cvi_right.toImageMsg());
	
	return;	
	
}

/* *************** PRIVATE FUNCTIONS *************** */
void CrectifyUncalibrated::siftgpu2opencv(const rh_feature_extraction::SiftGPU::ConstPtr& featL, const rh_feature_extraction::SiftGPU::ConstPtr& featR)
{

	ROS_INFO("Features in left camera msg time: %f", featL->header.stamp.toSec());
	ROS_INFO("Features in right camera msg time: %f", featR->header.stamp.toSec());
	
	vector<float> desc;
	
	// Left features and descriptors
	int16_t num = featL->numOfFeatures;
	ROS_INFO("Reading %d features in the left camera", num);
	std::vector<float>::const_iterator it = featL->keypoints.begin();
	featuresStereo.keysL.resize(0);
	for(int i = 0; i < num; i++)
	{
		cv::KeyPoint kpt;
		kpt.pt.x = *it; it++;
		kpt.pt.y = *it; it++;
		kpt.size = *it; it++;
		kpt.angle = *it; it++;
	    kpt.response = 0; kpt.octave = 0; kpt.class_id = 0;
	    featuresStereo.keysL.push_back(kpt);
	}
	
	featuresStereo.descL = Mat(featL->descriptors).reshape(1, num);
	featuresStereo.matches.resize(0);
	std::vector<int16_t>::const_iterator it_m = featL->matches.begin();
	for(int i = 0; i < (int)featL->matches.size()/2; i++)
	{
		int16_t query = *it_m; it_m++;
		int16_t train = *it_m; it_m++;
		featuresStereo.matches.push_back(DMatch((int)query, (int)train, 0, 0));
	}
	ROS_INFO("Size of matches from SIFT_GPU: %d", (int)featuresStereo.matches.size());
	
	// Right features
	num = featR->numOfFeatures;
	ROS_INFO("Reading %d features in the left camera", num);
	it = featR->keypoints.begin();
	featuresStereo.keysR.resize(0);
	for(int i = 0; i < num; i++)
	{
		cv::KeyPoint kpt;
		kpt.pt.x = *it; it++;
		kpt.pt.y = *it; it++;
		kpt.size = *it; it++;
		kpt.angle = *it; it++;
	    kpt.response = 0; kpt.octave = 0; kpt.class_id = 0;
	    featuresStereo.keysR.push_back(kpt);
	}
	
	featuresStereo.descR = Mat(featR->descriptors).reshape(1, num);
}

int CrectifyUncalibrated::getMatches(float distTreshold)
{
	if((int)featuresStereo.matches.size() == 0)
	{
		ROS_DEBUG("Creating FLANN database (binocular matching)");
		flann::Index treeFlannIndex(featuresStereo.descL, flann::KDTreeIndexParams());
		int k = 2;
		Mat results(featuresStereo.descR.rows, k, CV_32SC1);
		Mat dists(featuresStereo.descR.rows, k, CV_32FC1);

		ROS_DEBUG("Searching flan database (binocular matching)");
		treeFlannIndex.knnSearch(featuresStereo.descR, results, dists, k, flann::SearchParams());

		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		ROS_INFO("Nearest neighbours with log-likelihood ratio of: %f", distTreshold);
		featuresStereo.matches.resize(0);
		for(int i=0; i < featuresStereo.descR.rows; ++i)
		{
		    if(dists.at<float>(i,0) <= distTreshold * dists.at<float>(i,1))
		    {
		        featuresStereo.matches.push_back(DMatch( results.at<int>(i,0), i, 0, dists.at<float>(i,0)));

		    }
		}
    }

    ROS_INFO("Number of matches: %d", (int)featuresStereo.matches.size());

    return 1;

}

/* *************** MAIN PROGRAM *************** */
int main(int argc, char** argv)
{
    ros::init( argc, argv, "RH_rectifyUncalibrated_node" );
    CrectifyUncalibrated rU_;

    while( ros::ok() )
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}

