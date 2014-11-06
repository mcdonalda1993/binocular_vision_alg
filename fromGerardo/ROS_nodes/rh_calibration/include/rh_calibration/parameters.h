
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

static const char WINDOW_LEFT[] = "RH camera left";
static const char WINDOW_RIGHT[] = "RH camera right";
static const char WINDOW_LEFT_CORNERS[] = "Left corners";
static const char WINDOW_RIGHT_CORNERS[] = "Right corners";

// tf joints
static const char LEFT_PAN[] = "left_to_pan";
static const char LEFT_TILT[] = "left_to_tilt";
static const char RIGHT_PAN[] = "right_to_pan";
static const char RIGHT_TILT[] = "right_to_tilt";
static const char LEFT_CAMERA[] = "left_tilted_ptu";//"left_tilt_ptu";
static const char RIGHT_CAMERA[] = "right_tilted_ptu";//"right_tilt_ptu";
static const char RH_BASE_LEFT[] = "left_base_ptu";
static const char RH_BASE_RIGHT[] = "right_base_ptu";
static const char ROBOT_BASE[] = "r750";//"r1_link_1";

// Parameter server
static const char STP_PIX_X[] = "/PTU/stepsPerPixelX";
static const char STP_PIX_Y[] = "/PTU/stepsPerPixelY";
static const char PAN_L_RES[] = "/PTU/pan_left_res";
static const char TILT_L_RES[] = "/PTU/tilt_left_res";
static const char PAN_R_RES[] = "/PTU/pan_right_res";
static const char TILT_R_RES[] = "/PTU/tilt_right_res";
static const char CAPTURE_MODE[] = "/CAM/capturemode";
static const char OUTPUT_IMAGE_DIR[] = "/RH/calibration/outputImageDir";
static const char OUTPUT_CALIB_DIR[] = "/RH/calibration/outputCalibDir";
static const char WIDTH_MARKER[] = "/RH/calibration/marker_width";
static const char MARKER_SIZE_X[] = "/RH/calibration/marker_size_x";
static const char MARKER_SIZE_Y[] = "/RH/calibration/marker_size_y";
static const char MAX_ERROR_TH[] = "/RH/calibration/max_error";
static const char CALIB_TARGET[] = "/RH/calibration/target";
static const char SAVE_MODE[] = "/RH/calibration/save_mode";
static const char INPUT_SCALE[] = "/RH/calibration/resize_imgs_factor";
static const char HE_CALIB_FILE_URL[] = "/RH/gHc_calibration_file";
static const char ARM_Q[] = "/RH/calibration/arm_with_target";
static const char DEBUGQ[] = "/RH/calibration/debug";
static const char NO_IMGS_PTU[] = "/RH/calibration/noImagesPTU";

// Messages
static const char CAM_SUB_LEFT[] = "/RH/left_camera/image";
static const char CAM_SUB_RIGHT[] = "/RH/right_camera/image";
static const char PTU_IN[] = "/RH/cmd/PTU_status";
static const char CAMERA_INFO_L[] = "/RH/left_camera/camera_info";
static const char CAMERA_INFO_R[] = "/RH/right_camera/camera_info";

//Services
static const char CAPTURE_CAMERAS[] = "/RH/calibration/processTarget";
static const char CAPTURE_HANDEYE[] = "/RH/calibration/captureHandEye";
static const char CAMERA_CALIB[] = "/RH/calibration/cameraCalibration";
static const char PTU_MOVE_SRV[] = "/RH/cmd/PTU_moveSrv";
static const char PTU_INFO[] = "/RH/cmd/PTU_infoSrv";
static const char CIL[] = "/RH/left_camera/set_camera_info";
static const char CIR[] = "/RH/right_camera/set_camera_info";
static const char MARKER_SERVICE[] = "/RH/integration/MarkerDetectionService";

#endif
