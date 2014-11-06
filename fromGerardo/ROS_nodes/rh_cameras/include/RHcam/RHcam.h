// (c) SoCS, UoG. Gerardo Aragon-Camarasa. July, 2012

#ifndef __RHCAM_H
#define __RHCAM_H

// Gphoto
#include <gphoto2/gphoto2-camera.h>

// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>

// Standard libs
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// JPEG
#include <jpeglib.h>
#include <jerror.h>

using namespace cv;
using namespace std;

#define ERROR_MSG(x,y) ROS_ERROR("%s: '%s'", x, gp_result_as_string(y));
#define INFO_MSG(x) ROS_INFO("%s", x);

typedef struct
{
    // Main camera handler
    Camera *cam;
    // camera context (see libgphoto)
    GPContext *context;
    // 1: Live preview; 0: High resolution
    int mode;
    // Camera name and port of the camera
    //const char *name;
    //const char *port;
    // Flag to indicate if this is the first time capturing
    int firstCapture;
    // Image data BGR and RGB
    IplImage *imgBGR_left;//, *imgBGR_right;
    IplImage *imgRGB_left;//, *imgRGB_right;
    // USB port of the camera
    const char *port;
} DSLR_structTemplate, *DSLR_cam;

extern DSLR_cam camera_init(void);
extern int camera_open(DSLR_cam);

extern int get_value(Camera *, const char *, GPContext *);
extern int set_value(Camera *, const char *, const char *, GPContext *);

extern int camera_capture(DSLR_cam);
extern void camera_close(DSLR_cam);


#endif
