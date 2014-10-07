/*
* Copyright (C) 2009 Giacomo Spigler
* CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <iostream>
#include "liblogitech.h"
#include <cv.h>
#include <highgui.h>

using namespace std;

int main() {
	
	int width = 2592;
	int height = 1944;
	int qualityJPEG = 100;
	const char *filenameL = "testL";
	const char *filenameR = "testR";
	bool saveJPEG = false;
	bool saveYUV = true;
	
	// Arguments are (device name, width, height, save name, saveJPEG?, JPEG quality, saveYUV)	
	LogitechCam cam1("/dev/video0", width, height, filenameL, saveJPEG, qualityJPEG, saveYUV);
	LogitechCam cam2("/dev/video1", width, height, filenameR, saveJPEG, qualityJPEG, saveYUV);
	
	//cam1.getControl(CC_BRIGHTNESS);
	//cam1.setControl(CC_BRIGHTNESS, 128);
	//cam1.getControl(CC_BRIGHTNESS);
	
	cout<<"*** Camera 1"<<endl;
	cam1.setControl(CC_AUTO_FOCUS, 1);
	cam1.getControl(CC_AUTO_FOCUS);
	cam1.getControl(CC_FOCUS_ABSOLUTE);
	
	cam1.getControl(CC_AUTO_EXPOSURE_PRIORITY);
	cam1.getControl(CC_EXPOSURE_TIME_ABSOLUTE);
	
	cam1.setControl(CC_AUTO_EXPOSURE_MODE, 1);
	cam1.setControl(CC_AUTO_EXPOSURE_PRIORITY, 1);
	cam1.setControl(CC_EXPOSURE_TIME_ABSOLUTE, 166);
	
	cout<<"*** Camera 2"<<endl;
	cam2.setControl(CC_AUTO_FOCUS, 1);
	cam2.getControl(CC_AUTO_FOCUS);
	cam2.getControl(CC_FOCUS_ABSOLUTE);
	
	cam2.getControl(CC_AUTO_EXPOSURE_PRIORITY);
	cam2.getControl(CC_EXPOSURE_TIME_ABSOLUTE);
	
	cam2.setControl(CC_AUTO_EXPOSURE_MODE, 2);
	cam2.setControl(CC_AUTO_EXPOSURE_PRIORITY, 1);
	cam2.setControl(CC_EXPOSURE_TIME_ABSOLUTE, 166);
	
	
	// LEFT
	IplImage *left=cvCreateImage(cvSize(width, height), 8, 3);
	cvNamedWindow("LEFT", CV_WINDOW_NORMAL);
	cvResizeWindow("LEFT", 640, 480);
	
	//RIGHT
	IplImage *right=cvCreateImage(cvSize(width, height), 8, 3);
	cvNamedWindow("RIGHT", CV_WINDOW_NORMAL);
	cvResizeWindow("RIGHT", 640, 480);
	
	while(1){
		
			cam1.UpdateCam(&cam2);
			//cam2.UpdateCam();
			
			// Disable saving images in JPEG and YUV formats to avoid memory problems
			cam1.saveJPEG = false;
			cam1.saveYUV = false;
			cam2.saveJPEG = false;
			cam2.saveYUV = false;
			
			// Convert to OPENCV format	and display	
			cam1.toIplImage(left);
			cam2.toIplImage(right);
			
			cvShowImage("LEFT", left);
			cvShowImage("RIGHT", right);
		
		
		if( (cvWaitKey(10) & 255) == 27 ) break;
	}

	
	cvDestroyWindow("LEFT");
	cvReleaseImage(&left);
	
	cvDestroyWindow("RIGHT");
	cvReleaseImage(&right);
	
	return 0;
}
