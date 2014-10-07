#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <jpeglib.h>

#include "liblogitech.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

/** Print error message and terminate programm with EXIT_FAILURE return code. */
static void errno_exit(const char* s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit(EXIT_FAILURE);
}

/** Do ioctl and retry if error was EINTR ("A signal was caught during the ioctl() operation."). Parameters are the same as on ioctl. */
static int xioctl(int fidCam, int request, void* argp)
{
	int r, iterator = 0;
	
	do {
		r = ioctl(fidCam, request, argp);
		iterator++;
	}
	while (-1 == r && EINTR == errno && iterator < 100);
	
	return r;
}

/** LOGITECHCAM constructor */
LogitechCam::LogitechCam(const char *name, int w, int h, const char *filename, bool sJ, unsigned char q, bool sY, int f){
	
	device_name = name;
	imageFilename = filename;
	width = w;
	height = h;
	fps = f;
	jpegQuality = q;
	saveJPEG = sJ;
	saveYUV = sY;
	
	CResult ret;
	// Initialize the webcam library
	ret = c_init();
	if(ret) fprintf(stderr, "Unable to initialise LIBWEBCAM (%d).\n", ret);
	
	image_data = (unsigned char *)malloc(width*height*3*sizeof(char));//(unsigned char *)malloc(w*h*4);
	
	this->deviceOpen();
	this->deviceInit();
	this->captureStart();
	init = true;
}

/** LOGITECHCAM deconstructor */
LogitechCam::~LogitechCam() {
	this->StopCamera();
}

/** Stop cameras */
void LogitechCam::StopCamera() {
	if (init) {
		this->captureStop();
		this->deviceUninit();
		this->deviceClose();
		
		c_close_device(handle);
		
		free(image_data);
		init = false;
	}
}

/** Open device */
void LogitechCam::deviceOpen() {
	struct stat st;
	// stat file
	if (-1 == stat(device_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", device_name, errno, strerror (errno));
		exit(EXIT_FAILURE);
	}
	// check if its device
	if (!S_ISCHR (st.st_mode)) {
		fprintf(stderr, "%s is no device\n", device_name);
		exit(EXIT_FAILURE);
	}
	// open device for this lybrary
	fidCam = open(device_name, O_RDWR /* required */ | O_NONBLOCK, 0);
	
	// open device for webcam lybrary
	handle = c_open_device(device_name);
	
	// check if opening was successfull
	if (-1 == fidCam) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", device_name, errno, strerror (errno));
		exit(EXIT_FAILURE);
	}
}

/** Close device */
void LogitechCam::deviceClose() {
	if(-1==close(fidCam)) {
	   errno_exit("close");
    }
    fidCam=-1;
}

/** Initialise device */
void LogitechCam::deviceInit() {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;
	
	if (-1 == xioctl(fidCam, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",device_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
	
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n",device_name);
		exit(EXIT_FAILURE);
	}
	
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming i/o\n",device_name);
		exit(EXIT_FAILURE);
	}
	
	
	
	/* Select video input, video standard and tune here. */
	CLEAR(cropcap);
	
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (0 == xioctl(fidCam, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */
		
		if (-1 == xioctl(fidCam, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {	   
		/* Errors ignored. */
	}
	
	CLEAR (fmt);
	
	// v4l2_format
	fmt.type			 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width	  = width;
	fmt.fmt.pix.height	 = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field	  = V4L2_FIELD_INTERLACED;
	
	if (-1 == xioctl(fidCam, VIDIOC_S_FMT, &fmt))
		errno_exit("VIDIOC_S_FMT");
	
	/* Note VIDIOC_S_FMT may change width and height. */
	if (width != (int) fmt.fmt.pix.width) {
		width = fmt.fmt.pix.width;
		fprintf(stderr,"Image width set to %i by device %s.\n",width,device_name);
	}
	if (height != (int) fmt.fmt.pix.height) {
		height = fmt.fmt.pix.height;
		fprintf(stderr,"Image height set to %i by device %s.\n",height,device_name);
	}
	
	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;
	
	mmapInit();
}

/** Device uninitialisation */
void LogitechCam::deviceUninit() {
	
	int i;
	for (i = 0; i < n_buffers; ++i){
		if (-1 == munmap (buffers[i].start, buffers[i].length))
			errno_exit("munmap");
	}
	
	free(buffers);
}

/** Start capturing */
void LogitechCam::captureStart() {
	
	int i;
	enum v4l2_buf_type type;
	
	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;
		
		CLEAR (buf);
		
		buf.type	   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory	 = V4L2_MEMORY_MMAP;
		buf.index	  = i;
		
		if (-1 == xioctl(fidCam, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
	}
	
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (-1 == xioctl(fidCam, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");
}

/** Stop capturing */
void LogitechCam::captureStop() {
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (-1 == xioctl(fidCam, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
}

/** Memory initialisation */
void LogitechCam::mmapInit() {
	struct v4l2_requestbuffers req;
	
	CLEAR (req);
	
	req.count			= 4;
	req.type			 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory		    = V4L2_MEMORY_MMAP;
	
	if (-1 == xioctl(fidCam, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support memory mapping\n", device_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	
	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", device_name);
		exit(EXIT_FAILURE);
	}
	
	buffers = (buffer *)calloc(req.count, sizeof(*buffers));
	
	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}
	
	for (n_buffers = 0; n_buffers < (int) req.count; ++n_buffers) {
		struct v4l2_buffer buf;
		
		CLEAR (buf);
		
		buf.type	   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory	 = V4L2_MEMORY_MMAP;
		buf.index	  = n_buffers;
		
		if (-1 == xioctl(fidCam, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");
		
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
		mmap (NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fidCam, buf.m.offset);
		
		if (MAP_FAILED == buffers[n_buffers].start)
			errno_exit("mmap");
	}
    
}

/** Get frames from camera */
unsigned char *LogitechCam::GetFrames() {
    struct v4l2_buffer buf;
	
	CLEAR (buf);
	
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	
	if (-1 == xioctl(fidCam, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;
			
		case EIO:
		default:
			return 0;//errno_exit("VIDIOC_DQBUF");
		}
	}
	
	assert((int) buf.index < n_buffers);
	
	if (saveJPEG)
		this->imageProcess(buffers[buf.index].start);
	
	memcpy(image_data, (unsigned char *)buffers[buf.index].start, buffers[buf.index].length);
	
	if (saveYUV){
	    char sYUV[50];
	    sprintf(sYUV, "%s.yuv", imageFilename);
	    int outfd = open(sYUV, O_RDWR | O_CREAT | O_TRUNC);
	    if (-1 == outfd){
		   fprintf(stderr,"Opening YUV image file\n");
		   //return 1;
	    }
	    write(outfd, (const char *)image_data, strlen((const char *)image_data)+1);
	    close(outfd);    
	}
	
	if (-1 == xioctl(fidCam, VIDIOC_QBUF, &buf))
		return 0;//errno_exit("VIDIOC_QBUF");
	
	return image_data;
}

/** Process image */
unsigned char *LogitechCam::imageProcess(const void* p)
{
	unsigned char* src = (unsigned char*)p;
	//jpeg_data = malloc(width*height*3*sizeof(char));
	
	// convert from YUV422 to RGB888 and save YUV image if required
	YUV422toRGB888(width,height,src,image_data);
	
	// save jpeg image if required
	if (saveJPEG)
		this->jpegWrite(image_data);
	
	return image_data;
}

/** Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV */
void LogitechCam::YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int line, column;
	unsigned char *py, *pu, *pv;
	unsigned char *tmp = dst;
//	char sY[50];
//	FILE *fY;
	
	/* In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr. 
	Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. */
	py = src;
	pu = src + 1;
	pv = src + 3;
	
	#define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )
	
//	if (saveYUV > 0){
//		// Each chanel will be stored as sequential string lines in the file
//		sprintf(sY, "%sO.yuv", imageFilename);
//		fY = fopen(sY,"w");
//		fprintf (fY, "%d\n",width);
//		fprintf (fY, "%d\n",height);
//	}
	
	for (line = 0; line < height; ++line) {
		for (column = 0; column < width; ++column) {	 
			*tmp++ = CLIP((double)*py + 1.402*((double)*pv-128.0));
			*tmp++ = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));	 
			*tmp++ = CLIP((double)*py + 1.772*((double)*pu-128.0));
			
//			if (saveYUV > 0){
//				fprintf (fY, "%f\n",(double)*py);
//				fprintf (fY, "%f\n",(double)*pu);
//				fprintf (fY, "%f\n",(double)*pv);
//			}
			
			// increase py every time
			py += 2;
			
			// increase pu,pv every second time
			if ((column & 1)==1) {
				pu += 4;
				pv += 4;
			}
		}
	}
	
//	if (saveYUV > 0){
//		fclose(fY);
//	}
}

/** Write image to jpeg file. */
void LogitechCam::jpegWrite(unsigned char* img)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	
	JSAMPROW row_pointer[1];
	
	char sJPEG[50];
	sprintf(sJPEG, "%s.jpeg", imageFilename);
	
	FILE *outfile = fopen(sJPEG,"wb");
	
	// try to open file for saving
	if (!outfile) {
		errno_exit("jpeg");
	}
	
	// create jpeg data
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, outfile);
	
	// set image parameters
	cinfo.image_width = width;	
	cinfo.image_height = height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
	
	// set jpeg compression parameters to default
	jpeg_set_defaults(&cinfo);
	// and then adjust quality setting
	jpeg_set_quality(&cinfo, jpegQuality, TRUE);
	
	// start compress 
	jpeg_start_compress(&cinfo, TRUE);
	
	// feed data
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}
	
	// finish compression
	jpeg_finish_compress(&cinfo);
	
	// destroy jpeg data
	jpeg_destroy_compress(&cinfo);
	
	// close output file
	fclose(outfile);
}

/** Read frames and process them... this replaces MAINLOOP! */
bool LogitechCam::UpdateCam(unsigned int t, int timeout_ms) {
	
    bool grabbed = false;
    int grab_time_uS = 0;
    int t_ = t;
    
    while (!grabbed) {
	   if ((!grabbed) && (this->GetFrames()!=0)) grabbed = true;
	   if (!grabbed) {
		  usleep(t_);
		  grab_time_uS+=(int)t_;
		  if (grab_time_uS > timeout_ms * 1000) {
			 //fprintf(stderr,"Timeout!\n");
			 t_ = t;
			 //break;
		  }
	   }
    }
    
    return grabbed;
    
}

/** Read frames and process 2 cameras... */
bool LogitechCam::UpdateCam(LogitechCam *cam2, unsigned int t, int timeout_ms) {
    bool left_grabbed = false;
    bool right_grabbed = false;
    int grab_time_uS = 0;
    int t_ = t;
    
    while (!(left_grabbed && right_grabbed)) {
	   if ((!left_grabbed) && (this->GetFrames()!=0)) left_grabbed = true;
	   if ((!right_grabbed) && (cam2->GetFrames()!=0)) right_grabbed = true;
	   if (!(left_grabbed && right_grabbed)) {
		  usleep(t_);
		  grab_time_uS+=(int)t_;
		  if (grab_time_uS > timeout_ms * 1000) {
			 //fprintf(stderr,"Timeout!\n");
			 t_ = t;
			//break;
		  }
	   }
    }
    
    return left_grabbed & right_grabbed;
    
}

/** Get camera control according to libwebcam-0.2.1 */
int LogitechCam::getControl(CControlId opt){
	
	CControlValue value;
	CResult ret = c_get_control(this->handle, opt, &value);
	
	if(ret) {
		printf("Failed to get control %d. (ret = %d)\n", opt, ret);
		return -1;
	}
	else {
		printf("Current value for control %d = %d\n", opt, value.value);
		return value.value;
	}
}

/** Set camera control according to libwebcam-0.2.1 */
void LogitechCam::setControl(CControlId opt, int val)
{
	// This is a hack as CC_AUTO_EXPOSURE_MODE does not work with libwebcam-0.2.1 so native calls are used
	if(opt == CC_AUTO_EXPOSURE_MODE){
		struct v4l2_control control;
		int value = 0;		
		control.id = V4L2_CID_EXPOSURE_AUTO;
		control.value = val;
		if ((value = ioctl(fidCam, VIDIOC_S_CTRL, &control)) < 0)
			printf("Set Auto Exposure on error\n");
		else
			printf("Auto Exposure set to %d\n", control.value);
		
	}else{
		CControlValue value;
		value.value = val;
		CResult ret = c_set_control(this->handle, opt, &value);
		if(ret) {
			printf("Failed to set control %d. (ret = %d)\n", opt, ret);
		}else {
			printf("Successfully set control %d to = %d\n", opt, value.value);
		}
	}
}
	
/** Convert image buffer to OPENCV image */
#ifdef USE_OPENCV
void LogitechCam::toIplImage(IplImage *l) {
    unsigned char *l_=(unsigned char *)l->imageData;
    
    int w2 = (int)width / 2;
    for(int x=0; x<w2; x++) {
	   for(int y=0; y<height; y++) {
		  int y0, y1, u, v; //y0 u y1 v
		  
		  int i=(y*w2+x)*4;
		  y0=image_data[i];
		  u=image_data[i+1];
		  y1=image_data[i+2];
		  v=image_data[i+3];
		  
		  int r, g, b;
		  r = y0 + (1.370705 * (v-128));
		  g = y0 - (0.698001 * (v-128)) - (0.337633 * (u-128));
		  b = y0 + (1.732446 * (u-128));
		  
		  if(r > 255) r = 255;
		  if(g > 255) g = 255;
		  if(b > 255) b = 255;
		  if(r < 0) r = 0;
		  if(g < 0) g = 0;
		  if(b < 0) b = 0;
		  
		  i=(y*l->width+2*x)*3;
		  l_[i] = (unsigned char)(b); //B
		  l_[i+1] = (unsigned char)(g); //G
		  l_[i+2] = (unsigned char)(r); //R
		  
		  
		  r = y1 + (1.370705 * (v-128));
		  g = y1 - (0.698001 * (v-128)) - (0.337633 * (u-128));
		  b = y1 + (1.732446 * (u-128));
		  
		  if(r > 255) r = 255;
		  if(g > 255) g = 255;
		  if(b > 255) b = 255;
		  if(r < 0) r = 0;
		  if(g < 0) g = 0;
		  if(b < 0) b = 0;
		  
		  l_[i+3] = (unsigned char)(b); //B
		  l_[i+4] = (unsigned char)(g); //G
		  l_[i+5] = (unsigned char)(r); //R
		  
	   }
    }
    
}
#endif

