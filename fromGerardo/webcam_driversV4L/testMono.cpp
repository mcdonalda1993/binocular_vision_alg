#include <iostream>
#include "liblogitech.h"
#include <cv.h>
#include <highgui.h>

using namespace std;

int main() {

	int width = 2304;
	int height = 1536;
	int qualityJPEG = 100;
	const char *filenameL = "testL";
	bool saveJPEG = false;
	bool saveYUV = false;

	// Arguments are (device name, width, height, save name, saveJPEG?, JPEG quality, saveYUV)
	LogitechCam cam1("/dev/video0", width, height, filenameL, saveJPEG, qualityJPEG, saveYUV);
	

	cout<<"*** Camera 1"<<endl;
	cam1.setControl(CC_AUTO_FOCUS, 1);
	cam1.getControl(CC_AUTO_FOCUS);
	cam1.getControl(CC_FOCUS_ABSOLUTE);
	
	cam1.getControl(CC_BRIGHTNESS);
	//cam1.setControl(CC_BRIGHTNESS, 128);
	
	/* CC_AUTO_EXPOSURE_MODE follows:
	V4L2_EXPOSURE_MANUAL = 0, Gives I/O error
	V4L2_EXPOSURE_AUTO = 1, works
	V4L2_EXPOSURE_SHUTTER_PRIORITY = 2, Gives I/O error
	V4L2_EXPOSURE_APERTURE_PRIORITY = 3 works
	
	When V4L2_EXPOSURE_AUTO = 1 turns off exposure auto (seems like a bug in the code)
	*/
	cam1.setControl(CC_AUTO_EXPOSURE_MODE, 1);
	
	// To check min and max values use: v4l2-ctl --list-ctrls
	cam1.setControl(CC_EXPOSURE_TIME_ABSOLUTE, 166); // default: 166

	// LEFT
	IplImage *left=cvCreateImage(cvSize(width, height), 8, 3);
	cvNamedWindow("LEFT", CV_WINDOW_NORMAL);
	cvResizeWindow("LEFT", 640, 480);
	
	while(1){
		
			cam1.UpdateCam();
			
			// Disable saving images in JPEG and YUV formats to avoid memory problems
			cam1.saveJPEG = false;
			cam1.saveYUV = false;
			
			// Convert to OPENCV format	and display	
			cam1.toIplImage(left);
			
			cvShowImage("LEFT", left);		
		
		if( (cvWaitKey(10) & 255) == 27 ) break;
	}

	
	cvDestroyWindow("LEFT");
	cvReleaseImage(&left);
	
	return 0;
}
