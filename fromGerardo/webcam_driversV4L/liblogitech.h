
#include "webcam.h" // Control options of webcam

#ifndef __LIBLOGITECH__H__
#define __LIBLOGITECH__H__

#define USE_OPENCV 1

#ifdef USE_OPENCV
#include <cv.h>
#endif

struct buffer {
	void *			start;
	size_t			length;
};

class LogitechCam {
private:
	void deviceOpen();
	void deviceClose();
	void deviceInit();
	void deviceUninit();
	void captureStart();
	void captureStop();
	void mmapInit();
	void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst);
	void jpegWrite(unsigned char* img);
	
	unsigned char *imageProcess(const void* p);
	
	bool init;
	
public:
	const char		*device_name;	// Device name
	const char		*imageFilename;	// Image filename
	
	unsigned char	*image_data;	// Image data
	//unsigned char	*jpeg_data;		// JPEG data
	unsigned char	jpegQuality;	// JPEG quality
	
	int				width;			// Width image resolution
	int				height;			// Height image resolution
	int				fps;			// Frames per second
	int				fidCam;			// File identifier for camera
	int				n_buffers;
	
	CHandle			handle;	
	
	bool			saveJPEG;
	bool			saveYUV;
	
	buffer			*buffers;
	
	LogitechCam(const char *device_name, int width, int height, const char *imageFilename = NULL, bool saveJPEG = false, unsigned char jpegQuality = 70, bool saveYUV = false, int fps = 15);
	~LogitechCam();
	
	unsigned char *GetFrames();
	bool UpdateCam(unsigned int t = 100, int timeout_ms = 500); // One camera
	bool UpdateCam(LogitechCam *cam2, unsigned int t = 100, int timeout_ms = 5000); // Two cameras
	
	int getControl(CControlId option);
	void setControl(CControlId option, int val = 0);
	void queryControl(CControlId option);
	
	#ifdef USE_OPENCV
	void toIplImage(IplImage *im);
	#endif
	
	void StopCamera();
	
};

#endif

