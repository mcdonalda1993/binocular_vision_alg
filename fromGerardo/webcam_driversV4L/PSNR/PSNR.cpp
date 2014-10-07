/*
* Copyright (C) 2009 Giacomo Spigler
* CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <iostream>
#include "liblogitech.h"
#include <cv.h>
#include <highgui.h>
#include <getopt.h>

using namespace std;

int main() {
	
	int width = 2592;
	int height = 1944;
	int num = 100; // how many captures
	const char* fileName = "test";
	const char* fileNameT = NULL;
	char tL[50];
	
		
	//*****************************************************************************
	// Load cameras	
	LogitechCam cam1("/dev/video0", width, height, fileName, false, 100, false);
	//LogitechCam cam2("/dev/video1", width, height, filenameR, saveJPEG, jpegQuality, saveYUV);
		
	cout<<"*** Camera 1"<<endl;
	cam1.setControl(CC_AUTO_FOCUS, 0);
	cam1.getControl(CC_FOCUS_ABSOLUTE);
	cam1.setControl(CC_FOCUS_ABSOLUTE,85);
	
	// Capture images
	cam1.UpdateCam();
	
	
	// LEFT
	IplImage *left=cvCreateImage(cvSize(width, height), 8, 3);
	
	printf("Capturing Start!\n");
	int i;
	for (i = 0; i < num; i++){
	
		sprintf(tL, "%s%d.tif", fileName, i);
		fileNameT = (const char*)tL;
		cam1.UpdateCam();
		cam1.toIplImage(left);
		cvSaveImage(fileNameT, left);
		printf("%d\n",i);
	}
	printf("DONE!\n");
	cvReleaseImage(&left);
	
	return 0;
}
