// main include file

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cvsba/cvsba.h>

#include <vector>
#include <errno.h>
#include <fstream>
#include <math.h>

using namespace cv;
using namespace std;

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

#include <rh_cameras/CamerasSync.h>
#include <rh_cameras/SetCameraInfo.h>
#include <rh_ptu/GetPtuInfo.h>
#include <rh_ptu/MovePtu.h>
#include <sensor_msgs/CameraInfo.h>
#include <rh_calibration/CameraCalibration.h>
#include <rh_calibration/HandEyeCalibration.h>
#include <cvsba/cvsba.h>
#include <rh_integration/MarkerDetection.h>

namespace enc = sensor_msgs::image_encodings;

#define DEBUG true

