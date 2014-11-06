#include "RHcam/RHcam.h"

/* ************** */
/* From CONTEXT.C */
/* ************** */

/* From examples in libgphoto sources */
//static void
//ctx_error_func (GPContext *context, const char *format, va_list args, void *data)
//{
//    fprintf  (stderr, "\n");
//    fprintf  (stderr, "*** Contexterror ***              \n");
//    vfprintf (stderr, format, args);
//    fprintf  (stderr, "\n");
//    fflush   (stderr);
//}

//static void
//ctx_status_func (GPContext *context, const char *format, va_list args, void *data)
//{
//    vfprintf (stderr, format, args);
//    fprintf  (stderr, "\n");
//    fflush   (stderr);
//}

GPContext* create_context()
{
    GPContext *context;

    /* This is the mandatory part */
    /* It is set NULL in order to enable simultaneous capturing! */
    context = NULL;//gp_context_new();

    /* All the parts below are optional! */
    //gp_context_set_error_func (context, ctx_error_func, NULL);
    //gp_context_set_status_func (context, ctx_status_func, NULL);

    return context;
}


/* ***************** */
/* From AUTODETECT.C */
/* ***************** */

/* From examples in libgphoto sources */
static GPPortInfoList		*portinfolist = NULL;
static CameraAbilitiesList	*abilities = NULL;

/*
 * This detects all currently attached cameras and returns
 * them in a list. It avoids the generic usb: entry.
 *
 * This function does not open nor initialize the cameras yet.
 *
 */

int autodetect(CameraList *list, GPContext *context)
{
    int			ret, i;
    CameraList		*xlist = NULL;

    ret = gp_list_new (&xlist);
    if (ret < GP_OK) goto out;
    if (!portinfolist)
    {
        /* Load all the port drivers we have... */
        ret = gp_port_info_list_new (&portinfolist);
        if (ret < GP_OK) goto out;
        ret = gp_port_info_list_load (portinfolist);
        if (ret < 0) goto out;
        ret = gp_port_info_list_count (portinfolist);
        if (ret < 0) goto out;
    }
    /* Load all the camera drivers we have... */
    ret = gp_abilities_list_new (&abilities);
    if (ret < GP_OK) goto out;
    ret = gp_abilities_list_load (abilities, context);
    if (ret < GP_OK) goto out;

    /* ... and autodetect the currently attached cameras. */
    ret = gp_abilities_list_detect (abilities, portinfolist, xlist, context);
    if (ret < GP_OK) goto out;

    /* Filter out the "usb:" entry */
    ret = gp_list_count (xlist);
    if (ret < GP_OK) goto out;
    for (i=0; i<ret; i++)
    {
        const char *name, *value;

        gp_list_get_name (xlist, i, &name);
        gp_list_get_value (xlist, i, &value);
        if (!strcmp ("usb:",value)) continue;
        gp_list_append (list, name, value);
    }
out:
    gp_list_free (xlist);
    return gp_list_count(list);
}

/*
 * This function opens a camera depending on the specified model and port.
 */
int access_camera (Camera ** camera, const char *model, const char *port)
{
    int		ret, m, p;
    CameraAbilities	a;
    GPPortInfo	pi;

    ret = gp_camera_new (camera);
    if (ret < GP_OK) return ret;

    /* First lookup the model / driver */
    m = gp_abilities_list_lookup_model (abilities, model);
    if (m < GP_OK) return ret;
    ret = gp_abilities_list_get_abilities (abilities, m, &a);
    if (ret < GP_OK) return ret;
    ret = gp_camera_set_abilities (*camera, a);
    if (ret < GP_OK) return ret;

    /* Then associate the camera with the specified port */
    p = gp_port_info_list_lookup_path (portinfolist, port);
    if (ret < GP_OK) return ret;
    switch (p)
    {
    case GP_ERROR_UNKNOWN_PORT:
        fprintf (stderr, "The port you specified "
                 "('%s') can not be found. Please "
                 "specify one of the ports found by "
                 "'gphoto2 --list-ports' and make "
                 "sure the spelling is correct "
                 "(i.e. with prefix 'serial:' or 'usb:').",
                 port);
        break;
    default:
        break;
    }
    if (ret < GP_OK) return ret;
    ret = gp_port_info_list_get_info (portinfolist, p, &pi);
    if (ret < GP_OK) return ret;
    ret = gp_camera_set_port_info (*camera, pi);
    if (ret < GP_OK) return ret;
    return GP_OK;
}


/* ************* */
/* From CONFIG.C */
/* ************* */

/* From examples in libgphoto sources */

/*
 * This function looks up a label or key entry of
 * a configuration widget.
 * The functions descend recursively, so you can just
 * specify the last component.
 *
 * Taken from gphoto code!
 */

static int _lookup_widget(Camera *camera, GPContext *context, const char *name, CameraWidget **child, CameraWidget **rootconfig)
{
    int	ret;

    ret = gp_camera_get_config (camera, rootconfig, context);
    if (ret != GP_OK) return ret;
    ret = gp_widget_get_child_by_name (*rootconfig, name, child);
    if (ret != GP_OK)
        ret = gp_widget_get_child_by_label (*rootconfig, name, child);
    if (ret != GP_OK)
    {
        char	*part, *s, *newname;

        newname = strdup (name);
        if (!newname)
            return GP_ERROR_NO_MEMORY;

        *child = *rootconfig;
        part = newname;
        while (part[0] == '/')
            part++;
        while (1)
        {
            CameraWidget *tmp;

            s = strchr (part,'/');
            if (s)
                *s='\0';
            ret = gp_widget_get_child_by_name (*child, part, &tmp);
            if (ret != GP_OK)
                ret = gp_widget_get_child_by_label (*child, part, &tmp);
            if (ret != GP_OK)
                break;
            *child = tmp;
            if (!s) /* end of path */
                break;
            part = s+1;
            while (part[0] == '/')
                part++;
        }
        if (s)   /* if we have stuff left over, we failed */
        {
            gp_context_error (context, ("%s not found in configuration tree."), newname);
            free (newname);
            gp_widget_free (*rootconfig);
            return GP_ERROR;
        }
        free (newname);
    }
    return GP_OK;
}

/* Gets a string configuration value.
 * This can be:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int get_config_value_string (Camera *camera, const char *key, char **str, GPContext *context)
{
    CameraWidget *widget, *child;
    CameraWidgetType type;
    int	ret;
    char *val;
    float f = 0, t = 0, b = 0, s = 0;
    char current[50];


    ret = _lookup_widget (camera, context, key, &child, &widget);
    if (ret < GP_OK)
    {
        fprintf (stderr, "lookup widget failed: %s\n", gp_result_as_string(ret));
        goto out;
    }

    /* This type check is optional, if you know what type the label
     * has already. If you are not sure, better check. */
    ret = gp_widget_get_type (child, &type);
    if (ret < GP_OK)
    {
        fprintf (stderr, "widget get type failed: %s\n", gp_result_as_string(ret));
        goto out;
    }
    switch (type)
    {
    case GP_WIDGET_MENU:
    case GP_WIDGET_RADIO:
    case GP_WIDGET_TEXT:
        /* This is the actual query call. Note that we just
         * a pointer reference to the string, not a copy... */
        ret = gp_widget_get_value (child, &val);
        if (ret < GP_OK)
        {
            fprintf (stderr, "could not query widget value: %s\n", gp_result_as_string(ret));
            goto out;
        }
        /* Create a new copy for our caller. */
        *str = strdup (val);
        break;
    case GP_WIDGET_RANGE:

        ret = gp_widget_get_range(child, &b, &t, &s);
        if (ret == GP_OK)
            ret = gp_widget_get_value(child, &f);
        if (ret == GP_OK)
        {
            printf ("Type: RANGE ");	/* parsed by scripts, no i18n */
            //printf ("Current: %g\n", f);	/* parsed by scripts, no i18n */
            printf ("Bottom: %g ", b);	/* parsed by scripts, no i18n */
            printf ("Top: %g ", t);	/* parsed by scripts, no i18n */
            printf ("Step: %g\n", s);	/* parsed by scripts, no i18n */

            sprintf(current, "%f", f);
            *str = strdup(current);
        }
        else
        {
            ERROR_MSG("Failed to retrieve values of range widget", ret);
        }
        break;
    default:
        fprintf (stderr, "widget has bad type %d\n", type);
        ret = GP_ERROR_BAD_PARAMETERS;
        goto out;
    }

out:
    gp_widget_free (widget);
    return ret;
}


/* Sets a string configuration value.
 * This can set for:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int set_config_value_string (Camera *camera, const char *key, const char *val, GPContext *context)
{
    CameraWidget		*widget, *child;
    CameraWidgetType	type;
    int			ret;

    ret = _lookup_widget (camera, context, key, &child, &widget);
    if (ret < GP_OK)
    {
        fprintf (stderr, "lookup widget failed: %s\n", gp_result_as_string(ret));
        goto out;
    }

    /* This type check is optional, if you know what type the label
     * has already. If you are not sure, better check. */
    ret = gp_widget_get_type (child, &type);
    if (ret < GP_OK)
    {
        fprintf (stderr, "widget get type failed: %s\n", gp_result_as_string(ret));
        goto out;
    }
    switch (type)
    {
    case GP_WIDGET_MENU:
    case GP_WIDGET_RADIO:
    case GP_WIDGET_TEXT:
    case GP_WIDGET_TOGGLE:
        break;
    default:
        fprintf (stderr, "widget has bad type %d\n", type);
        ret = GP_ERROR_BAD_PARAMETERS;
        goto out;
    }

    /* This is the actual set call. Note that we keep
     * ownership of the string and have to free it if necessary.
     */
    ret = gp_widget_set_value (child, val);
    if (ret < GP_OK)
    {
        fprintf (stderr, "could not set widget value: %s\n", gp_result_as_string(ret));
        goto out;
    }
    /* This stores it on the camera again */
    ret = gp_camera_set_config (camera, widget, context);
    if (ret < GP_OK)
    {
        fprintf (stderr, "camera_set_config failed: %s\n", gp_result_as_string(ret));
        return ret;
    }
out:
    gp_widget_free (widget);
    return ret;
}

int get_value(Camera *cam, const char *command, GPContext *context)
{

    char *val;
    int ret;
    char buffer[100];

    ret = get_config_value_string(cam, command, &val, context);
    if (ret < GP_OK)
    {
        sprintf(buffer,"Could not get %s\n", command);
        ERROR_MSG(buffer, GP_ERROR);
    }
    else
    {
        sprintf(buffer,"%s: %s\n", command, val);
        INFO_MSG(buffer);
    }
    free(val);

    return ret;
}

int set_value(Camera *cam, const char *command, const char *val, GPContext *context)
{

    int ret;
    char buffer[100];

    ret = set_config_value_string(cam, command, val, context);
    if (ret < GP_OK)
    {
        sprintf(buffer,"Could not set %s with value %s\n", command, val);
        ERROR_MSG(buffer, GP_ERROR);
    }
    else
    {
        sprintf(buffer,"Set %s with %s\n", command, val);
        INFO_MSG(buffer);
    }

    return ret;
}

// Got it from gphoto2
//int print_info_action(Camera *camera, GPContext *context, const char *folder, const char *filename)
//{
//    CameraFileInfo info;
//    int ret;

//    ret = gp_camera_file_get_info(camera, folder, filename, &info, NULL);
//    if(ret < GP_OK)
//        fprintf(stderr,"Retrieving file info: %s\n", gp_result_as_string(ret));

//    printf ("Information on file '%s' (folder '%s'):\n", filename, folder);
//    printf ("File:\n");
//    if (info.file.fields == GP_FILE_INFO_NONE)
//    {
//        printf("  None available.\n");
//    }
//    else
//    {
//        if (info.file.fields & GP_FILE_INFO_NAME)
//            printf("  Name:        '%s'\n", info.file.name);
//        if (info.file.fields & GP_FILE_INFO_TYPE)
//            printf("  Mime type:   '%s'\n", info.file.type);
//        if (info.file.fields & GP_FILE_INFO_SIZE)
//            printf("  Size:        %lu byte(s)\n", (unsigned long int)info.file.size);
//        if (info.file.fields & GP_FILE_INFO_WIDTH)
//            printf("  Width:       %i pixel(s)\n", info.file.width);
//        if (info.file.fields & GP_FILE_INFO_HEIGHT)
//            printf("  Height:      %i pixel(s)\n", info.file.height);
//        if (info.file.fields & GP_FILE_INFO_STATUS)
//            printf("  Downloaded:  %s\n",(info.file.status == GP_FILE_STATUS_DOWNLOADED) ? ("yes") : ("no"));
//        if (info.file.fields & GP_FILE_INFO_PERMISSIONS)
//        {
//            printf("  Permissions: ");
//            if ((info.file.permissions & GP_FILE_PERM_READ) &&
//                    (info.file.permissions & GP_FILE_PERM_DELETE))
//                printf ("read/delete");
//            else if (info.file.permissions & GP_FILE_PERM_READ)
//                printf ("read");
//            else if (info.file.permissions & GP_FILE_PERM_DELETE)
//                printf ("delete");
//            else
//                printf ("none");
//            putchar ('\n');
//        }
//        if (info.file.fields & GP_FILE_INFO_MTIME)
//            //printf ("  Time:        %d\n", (int)info.file.mtime);
//            printf ("  Time:        %s\n", asctime (localtime (&info.file.mtime)));
//    }
//    printf("Thumbnail:\n");
//    if (info.preview.fields == GP_FILE_INFO_NONE)
//    {
//        printf ("  None available.\n");
//    }
//    else
//    {
//        if (info.preview.fields & GP_FILE_INFO_TYPE)
//            printf ("  Mime type:   '%s'\n", info.preview.type);
//        if (info.preview.fields & GP_FILE_INFO_SIZE)
//            printf ("  Size:        %lu byte(s)\n", (unsigned long int)info.preview.size);
//        if (info.preview.fields & GP_FILE_INFO_WIDTH)
//            printf ("  Width:       %i pixel(s)\n", info.preview.width);
//        if (info.preview.fields & GP_FILE_INFO_HEIGHT)
//            printf ("  Height:      %i pixel(s)\n", info.preview.height);
//        if (info.preview.fields & GP_FILE_INFO_STATUS)
//            printf ("  Downloaded:  %s\n",(info.preview.status == GP_FILE_STATUS_DOWNLOADED) ? ("yes") : ("no"));
//    }

//    return (GP_OK);
//}


/* ************** */
/* From CAPTURE.C */
/* ************** */

// Much of the gphoto code came from a post on the gphoto-devel mailinglist
// The code for using a custom source manager in libjpeg was modified from http://wiki.allegro.cc/Libjpeg
// Parts of this code was taken from: http://georgelandon.blogspot.co.uk/

typedef struct my_src_mgr my_src_mgr;

struct my_src_mgr
{
    struct jpeg_source_mgr pub;
    JOCTET eoi_buffer[2];
};

static void init_source(j_decompress_ptr cinfo)
{
}

static int fill_input_buffer(j_decompress_ptr cinfo)
{
    return 1;
}

static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_src_mgr *src = (my_src_mgr *)cinfo->src;

    if (num_bytes > 0)
    {
        while (num_bytes > (long)src->pub.bytes_in_buffer)
        {

            num_bytes -= (long)src->pub.bytes_in_buffer;
            fill_input_buffer(cinfo);
        }
    }

    src->pub.next_input_byte += num_bytes;
    src->pub.bytes_in_buffer -= num_bytes;
}

static void term_source(j_decompress_ptr cinfo)
{
}


void jpeg_memory_src(j_decompress_ptr cinfo, unsigned char const *buffer, size_t bufsize)
{

    my_src_mgr *src;
    if (! cinfo->src)
    {
        //cinfo->src = (*cinfo->mem->alloc_small)((j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(my_src_mgr));
        cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,sizeof(struct jpeg_source_mgr));
    }

    src = (my_src_mgr *)cinfo->src;
    src->pub.init_source = init_source;

    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;

    src->pub.resync_to_restart = jpeg_resync_to_restart;
    src->pub.term_source = term_source;

    src->pub.next_input_byte = buffer;
    src->pub.bytes_in_buffer = bufsize;
}

void nikonJPEG2opencv(const unsigned char** data, unsigned long size, int* firstCapture, IplImage** outBGR, IplImage** outRGB)
{

    //libjpeg variables
    struct jpeg_error_mgr jerr;
    struct jpeg_decompress_struct cinfo;
    char * raw_image= NULL;
    //libjpeg data structure for storing one row, that is, scanline of an image
    JSAMPROW row_pointer[1];
    unsigned long location = 0;
    int i;
    IplImage *imgBGR, *imgRGB;

    // set up the standard libjpeg error handler
    cinfo.err = jpeg_std_error( &jerr );

    // setup decompression process and source, then read JPEG header
    jpeg_create_decompress( &cinfo );

    // setup source for memory read
    jpeg_memory_src(&cinfo, *data, size);

    //reading the image header which contains image information
    jpeg_read_header( &cinfo, TRUE );

    //Uncomment the following to output image information, if needed.
    //printf( "JPEG File Information: \n" );
    //printf( "Image width and height: %d pixels and %d pixels.\n", cinfo.image_width, cinfo.image_height );
    //printf( "Color components per pixel: %d.\n", cinfo.num_components );
    //printf( "Color space: %d.\n", cinfo.jpeg_color_space );

    //Start decompression jpeg here
    jpeg_start_decompress(&cinfo);

    //Set up openCV images
    if(*firstCapture == 1)
    {
        imgBGR =cvCreateImageHeader(cvSize( cinfo.output_width, cinfo.output_height), IPL_DEPTH_8U, cinfo.num_components);
        cvCreateData(imgBGR);

        imgRGB =cvCreateImageHeader(cvSize( cinfo.output_width,cinfo.output_height), IPL_DEPTH_8U, cinfo.num_components);
        cvCreateData(imgRGB);

        *firstCapture = 0;
    }
    else
    {
        imgBGR = *outBGR;
        imgRGB = *outRGB;
    }

    //allocate memory to hold the uncompressed image
    raw_image = imgBGR->imageData;

    //now actually read the jpeg into the raw buffer
    row_pointer[0] = (unsigned char *)malloc( cinfo.output_width*cinfo.num_components );

    //read one scan line at a time
    while( cinfo.output_scanline < cinfo.image_height )
    {

        jpeg_read_scanlines( &cinfo, row_pointer, 1 );
        for( i=0; i<(int)cinfo.image_width*cinfo.num_components; i++)
        {

            raw_image[location++] = row_pointer[0][i];
        }
    }

    *outBGR = imgBGR;
    *outRGB = imgRGB;

    //wrap up decompression, destroy objects, free pointers
    jpeg_finish_decompress( &cinfo );
    jpeg_destroy_decompress( &cinfo );
    free(row_pointer[0]);

}

/*
 *  Capture an image from the cameras
 */
int capture_from_camera(Camera *cam, GPContext *context, CameraFilePath path, const unsigned char** data, unsigned long* size)
{

    int ret;
    CameraFile *file = NULL;
    const unsigned char *dataCap = NULL;
    //unsigned long	size;
    //char buffer[100];

    //Capturing image
    ret = gp_camera_capture(cam, GP_CAPTURE_IMAGE, &path, context);
    if(ret < GP_OK)
    {
        ERROR_MSG("ERROR->Capturing image", ret);
        return ret;
    }

    ret = gp_file_new(&file);
    if(ret < GP_OK)
    {
        ERROR_MSG("ERROR->Creating new file", ret);
        return ret;
    }

    //Downloading image
    ret = gp_camera_file_get(cam, path.folder, path.name, GP_FILE_TYPE_NORMAL, file, context);
    if(ret < GP_OK)
    {
        ERROR_MSG("ERROR->Retrieving file",ret);
        return ret;
    }

    //ret = print_info_action(cam, context, path.folder, path.name);
//	Getting Info
//	if(ret != GP_OK) {
//		ERROR_MSG("ERROR->Retrieve file info",ret);
//		return ret;
//	}

    ret = gp_file_get_data_and_size(file, (const char**)&dataCap, size);
    if(ret != GP_OK)
    {
        ERROR_MSG("ERROR->Retrieve file data and size", ret);
        return ret;
    }

    *data = dataCap;

    // Code from here waits for camera to complete everything.
    // Trying to take two successive captures without waiting
    // will result in the camera getting randomly stuck.
    int waittime = 10;
    CameraEventType type;
    void *dataTemp;

    //sprintf(buffer, "Wait for events from camera... ");
    //INFO_MSG(buffer);
    while(1)
    {
        ret = gp_camera_wait_for_event(cam, waittime, &type, &dataTemp, context);
        if(ret != GP_OK)
        {
            ERROR_MSG("ERROR->Wait for event", ret);
            return ret;
        }
        if(type == GP_EVENT_TIMEOUT)
        {
            //sprintf(buffer, "Capture completed");
            //INFO_MSG(buffer);
            break;
        }
        else if (type == GP_EVENT_CAPTURE_COMPLETE)
        {
            //sprintf(buffer, "Capture completed");
            //INFO_MSG(buffer);
            waittime = 10;
        }
        else if (type != GP_EVENT_UNKNOWN)
        {
            ERROR_MSG("Unexpected event received from camera", (int)type);
        }
    }

    return ret;
}

int capture_preview_from_camera(Camera *cam, GPContext *context, const unsigned char** data, unsigned long* size)
{

    int ret;
    CameraFile *file = NULL;
    unsigned char *dataCap = NULL;
    //unsigned long	size;

    ret = gp_file_new(&file);
    if(ret < GP_OK)
    {
        ERROR_MSG("ERROR->Creating new file", ret);
        return ret;
    }

    //Capturing image
    ret = gp_camera_capture_preview(cam, file, context);
    if(ret < GP_OK)
    {
        ERROR_MSG("ERROR->Capturing image", ret);
        return ret;
    }

    ret = gp_file_get_data_and_size(file, (const char**)&dataCap, size);
    if(ret != GP_OK)
    {
        ERROR_MSG("ERROR->Retrieve file data and size", ret);
        return ret;
    }

    *data = dataCap;

    // Code from here waits for camera to complete everything.
    // Trying to take two successive captures without waiting
    // will result in the camera getting randomly stuck.
    int waittime = 10;
    CameraEventType type;
    void *dataTemp;

    while(1)
    {
        ret = gp_camera_wait_for_event(cam, waittime, &type, &dataTemp, context);
        if(ret != GP_OK)
        {
            ERROR_MSG("ERROR->Wait for event", ret);
            return ret;
        }
        if(type == GP_EVENT_TIMEOUT)
        {
            //printf("Done!\n");
            break;
        }
        else if (type == GP_EVENT_CAPTURE_COMPLETE)
        {
            //printf("Capture completed\n");
            waittime = 10;
        }
        else if (type != GP_EVENT_UNKNOWN)
        {
            ERROR_MSG("Unexpected event received from camera", (int)type);
        }
    }

    return ret;
}

/* ************************* */
/* CAMERA SPECIFIC FUCNTIONS */
/* ************************* */

DSLR_cam camera_init()
{
    DSLR_cam cam_h;
    cam_h = (DSLR_cam)calloc(1, sizeof(DSLR_structTemplate));
    cam_h->context = create_context();
    cam_h->cam = NULL;
    cam_h->mode = 0; // normal mode by default
    cam_h->port = NULL;
    cam_h->firstCapture = 1;
    return cam_h;
}

int camera_open(DSLR_cam cam_h)
{
    CameraList *list; // holder for found cameras
    Camera **cams;
    int camSelected = 0;
    int ret, i, count;
    const char *name = NULL, *value = NULL;
    char buffer [100];

    // Create context
    cam_h->context = create_context (); // see context.c

    // Detect all the cameras that can be autodetected...
    ret = gp_list_new(&list);
    if (ret < GP_OK)
        return GP_ERROR;

    count = autodetect(list, cam_h->context);
    sprintf(buffer, "Number of cameras: %d\n", count);
    INFO_MSG(buffer);
    if(count == 0)
    {
        ERROR_MSG("Make sure that the cameras are connected", GP_ERROR_LIBRARY);
        return GP_ERROR_LIBRARY;
    }

    // Now open all cameras autodected previously for usage
    cams = (Camera **)calloc(sizeof (Camera*),count);
    //cam_h->cam = cams;
    for (i = 0; i < count; i++)
    {
        gp_list_get_name(list, i, &name);
        gp_list_get_value(list, i, &value);
        
        if (strcmp(value, cam_h->port) == 0)
        {
        ret = access_camera(&cams[i], name, value);
        if (ret == GP_OK)
			{
				// Open the first camera available!
				sprintf(buffer, "Camera %s on port %s opened\n", name, value);
				INFO_MSG(buffer);
				cam_h->cam = cams[i];
				//cam_h->name = name;
				//cam_h->port = value;

				camSelected += 1;
				break;
			}
			else
			{
				sprintf(buffer, "Camera %s on port %s failed to open\n", name, value);
				ERROR_MSG(buffer, GP_ERROR);
			}
        }
    }

    if(camSelected != 1)
    {
        ERROR_MSG("Camera is not available", GP_ERROR);
        return GP_ERROR;
    }
    
    // Set to capture images to the internal RAM (0: Internal RAM; 1: SD card
	ret = set_value(cam_h->cam, "capturetarget", "0", cam_h->context);
	if (ret < GP_OK)
	{
		ROS_ERROR("RHcam_node exception: cannot set *capturetarget* option");
		return ret;
	}

    return GP_OK;
}

int camera_capture(DSLR_cam cam_h)
{
    int ret;
    // image data and size
    const unsigned char *img_dataL = NULL;//, *img_dataR = NULL;
    unsigned long img_size = 0;
    int fL = cam_h->firstCapture;//, fR = cam_h->firstCapture;
    // Path of where image has been saved
    CameraFilePath path;

    if(cam_h->mode == 1)  // Captures a preview image
    {
        ret = capture_preview_from_camera(cam_h->cam, cam_h->context, &img_dataL, &img_size);
        if (ret < GP_OK)
            return GP_ERROR;
        
        // Tranform image data into OpenCV data
    	nikonJPEG2opencv(&img_dataL, img_size, &fL, &cam_h->imgBGR_left, &cam_h->imgRGB_left);
        
//        ret = capture_preview_from_camera(cam_h->cam[1], cam_h->context, &img_dataR, &img_size);
//        if (ret < GP_OK)
//            return GP_ERROR;
//        
//        // Tranform image data into OpenCV data
//    	nikonJPEG2opencv(&img_dataR, img_size, &fR, &cam_h->imgBGR_right, &cam_h->imgRGB_right);
    }
    else   // Captures a full resolution image
    {
        ret = capture_from_camera(cam_h->cam, cam_h->context, path, &img_dataL, &img_size);
        if (ret < GP_OK)
            return GP_ERROR;
        
        // Tranform image data into OpenCV data
    	nikonJPEG2opencv(&img_dataL, img_size, &fL, &cam_h->imgBGR_left, &cam_h->imgRGB_left);
        
//        ret = capture_from_camera(cam_h->cam[1], cam_h->context, path, &img_dataR, &img_size);
//        if (ret < GP_OK)
//            return GP_ERROR;
//            
//        // Tranform image data into OpenCV data
//    	nikonJPEG2opencv(&img_dataR, img_size, &fR, &cam_h->imgBGR_right, &cam_h->imgRGB_right);
    }
    
    cam_h->firstCapture = fL;
    
    INFO_MSG("Images captured...");

    return GP_OK;
}


void camera_close(DSLR_cam cam_h)
{
    int ret;
    
	ret=gp_camera_exit(cam_h->cam, NULL);
	if(ret != GP_OK)
	{
	    ERROR_MSG("Exit camera failed", ret);
	    return;
	}
	ret = gp_camera_free(cam_h->cam);
	if (ret != GP_OK)
	{
	    ERROR_MSG("Free camera failed", ret);
	    return;
	}
    free(cam_h);
}
