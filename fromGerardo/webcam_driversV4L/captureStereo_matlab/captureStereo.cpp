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

/** print usage information */
static void usage(FILE* fp, int argc, char** argv)
{
	fprintf (fp,
		"Usage: %s [options]\n\n"
		"Options:\n"
		"-h | --help          Print this message\n"
		"-o | --output        Image filename header (e.g. test)\n"
		"-q | --quality       JPEG compression quality\n"
		"-W | --width         Width\n"
		"-H | --height        Height\n"
		"-J | --JPEG          Save in JPEG format\n"
		"-Y | --YUV           Save in YUV format\n"
		"-L | --focus_left    Set focus in the left camera\n"
		"-R | --focus_right   Set focus in the right camera\n"
		"",
		argv[0]);
}

static const char short_options [] = "d:ho:q:mruW:H:";

static const struct option
long_options [] = {
	{ "help",        no_argument,            NULL,           'h' },
	{ "output",      required_argument,      NULL,           'o' },
	{ "quality",     required_argument,      NULL,           'q' },
	{ "width",       required_argument,      NULL,           'W' },
	{ "height",      required_argument,      NULL,           'H' },
	{ "JPEG",        no_argument,            NULL,           'J' },
	{ "YUV",         no_argument,            NULL,           'Y' },
	{ "focus_left",  required_argument,      NULL,           'L' },
	{ "focus_right", required_argument,      NULL,           'R' },
	{ 0, 0, 0, 0 }
};


int main(int argc, char **argv) {
	
	int width = 640;
	int height = 480;
	int jpegQuality = 100;
	int focusL = 68;
	int focusR = 68;
	const char* fileName = NULL;
	const char* filenameL = NULL;
	const char* filenameR = NULL;
	bool saveJPEG = false;
	bool saveYUV = false;
		
	/* OPTIONS ************************************************************/
	for (;;) {
		int index, c = 0;
		c = getopt_long(argc, argv, short_options, long_options, &index);
		if (-1 == c)
			break;
		
		switch (c) {
		case 0: /* getopt_long() flag */
			break;
		case 'h':
			usage (stdout, argc, argv);
			exit(EXIT_SUCCESS);
		case 'o':
			fileName = optarg;
			break;
		case 'q':
			jpegQuality = atoi(optarg);
			break;
		case 'W':
			width = atoi(optarg);
			printf("Image width set to %i\n",width);
			break;
		case 'H':
			height = atoi(optarg);
			printf("Image height set to %i\n",height);
			break;
		case 'J':
			saveJPEG = true;
			break;
		case 'Y':
			saveYUV = true;
			break;
		case 'L':
			focusL = atoi(optarg);
			printf("Focus in left cameraset to %i\n",focusL);
			break;
		case 'R':
			focusR = atoi(optarg);
			printf("Focus in left cameraset to %i\n",focusR);
			break;
		default:
			usage(stderr, argc, argv);
			exit(EXIT_FAILURE);
		}
	}
		
	// check for need parameters
	if (!fileName) {
		fprintf(stderr, "You have to specify output filename!\n\n");
		usage(stdout, argc, argv);
		exit(EXIT_FAILURE);
	}else{
		char tL[50];
		char tR[50];
		sprintf(tL, "%sL", fileName);
		sprintf(tR, "%sR", fileName);
		filenameL = (const char*)tL;
		filenameR = (const char*)tR;
	}
		
	//*****************************************************************************
	// Load cameras	
	LogitechCam cam1("/dev/video0", width, height, filenameL, saveJPEG, jpegQuality, saveYUV);
	LogitechCam cam2("/dev/video1", width, height, filenameR, saveJPEG, jpegQuality, saveYUV);
		
	cout<<"*** Camera 1"<<endl;
	cam1.setControl(CC_AUTO_FOCUS, 0);
	//cam1.getControl(CC_AUTO_FOCUS);
	cam1.getControl(CC_FOCUS_ABSOLUTE);
	cam1.setControl(CC_FOCUS_ABSOLUTE,50);
	cam1.setControl(CC_FOCUS_ABSOLUTE,focusL);
	
	
	cout<<"*** Camera 2"<<endl;
	cam2.setControl(CC_AUTO_FOCUS, 0);
	cam2.getControl(CC_AUTO_FOCUS);
	cam2.getControl(CC_FOCUS_ABSOLUTE);
	cam2.setControl(CC_FOCUS_ABSOLUTE,50);
	cam2.setControl(CC_FOCUS_ABSOLUTE,focusR);
	
	
	// Capture images
	cam1.UpdateCam(&cam2);
	
	/*
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
	*/
	
	return 0;
}
