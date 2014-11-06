cmake_minimum_required(VERSION 2.8.3)
project(rh_stereo_proc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  stereo_msgs
  message_generation
  rh_feature_extraction
  rh_cameras
  rh_vergence
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	foveatedstack.msg
)

add_service_files(
  FILES
  GetDisparitiesGPU.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   stereo_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rh_c3d
  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp sensor_msgs std_msgs message_runtime rh_vergence pcl_ros rh_feature_extraction rh_cameras stereo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## Old GPUmatcher (working)
#find_package(CUDA)
#if (CUDA_FOUND)
#	include(ExternalProject)
#	ExternalProject_Add(GPU_matcher
#	   SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/GPU_matcher
#	   BUILD_IN_SOURCE 1
#	   CONFIGURE_COMMAND ""
#	   BUILD_COMMAND make
#	)
#endif (CUDA_FOUND)

### testing non fovea (working)
add_executable(RHdisp_image src/RH_dispImgTest.cpp)
add_dependencies(RHdisp_image rh_cameras_generate_messages_cpp)
target_link_libraries(RHdisp_image boost_signals)
target_link_libraries(RHdisp_image ${catkin_LIBRARIES})
# testing fovea (not yet working)
#add_executable(RHFdisp_image src/RH_FdispImgTest.cpp)
#add_dependencies(RHFdisp_image rh_cameras_generate_messages_cpp)
#target_link_libraries(RHFdisp_image boost_signals)
#target_link_libraries(RHFdisp_image ${catkin_LIBRARIES})
###

add_executable(RHpoint_cloud src/getPointCloud.cpp)
add_dependencies(RHpoint_cloud rh_cameras_generate_messages_cpp)
target_link_libraries(RHpoint_cloud boost_signals)
target_link_libraries(RHpoint_cloud ${catkin_LIBRARIES})

add_executable(RHstereo_rectifyUncal src/rectify_imagesUncalibrated.cpp)
add_dependencies(RHstereo_rectifyUncal rh_feature_extraction_generate_messages_cpp)
target_link_libraries(RHstereo_rectifyUncal boost_signals)
target_link_libraries(RHstereo_rectifyUncal ${catkin_LIBRARIES})

add_executable(publish_images src/utils/publish_images.cpp)
target_link_libraries(publish_images ${catkin_LIBRARIES})

## New GPUmatcher (Not working atm)
find_package(CUDA REQUIRED)
#if (CUDA_FOUND)
SET(CMAKE_C_FLAGS "-lopencv_core -lopencv_highgui -lm -m64")
#SET(CUDA_NVCC_FLAGS "-gencode arch=compute_10,code=sm_10 -gencode arch=compute_20,code=sm_20")

set(clopema_cuda $ENV{CLOPEMA_CUDA})
if("${clopema_cuda}" STREQUAL "")
	SET(CUDA_NVCC_FLAGS "-gencode arch=compute_10,code=sm_10 -gencode arch=compute_20,code=sm_20 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=sm_35")
endif()
if("${clopema_cuda}" STREQUAL "NEW")
	SET(CUDA_NVCC_FLAGS "-gencode arch=compute_20,code=sm_20 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=sm_35 -gencode arch=compute_37,code=sm_37 -gencode arch=compute_50,code=sm_50 -gencode arch=compute_50,code=compute_50")
endif()


ADD_DEFINITIONS(-DCUDA_BUILD)
SET(CMAKE_CXX_FLAGS "-O3")
SET (LIB_NAMES opencv_core opencv_features2d opencv_flann opencv_gpu
               opencv_nonfree
               opencv_highgui opencv_imgproc opencv_calib3d)

FOREACH(lib ${LIB_NAMES})
  FIND_LIBRARY(LIB_${lib} ${lib})
  IF (LIB_${lib})
    MESSAGE(STATUS "found library '${lib}': ${LIB_${lib}}")
  ELSE (LIB_${lib})
    MESSAGE(FATAL_ERROR "cannot find library '${lib}' - go and get it!! :)")
  ENDIF (LIB_${lib})
  LIST(APPEND LIBS ${LIB_${lib}})
ENDFOREACH(lib)
include_directories(/usr/local/cuda/samples/common/inc /usr/local/cuda-5.5/lib64)  

CUDA_ADD_LIBRARY(MatchLib src/MatchLib.cu)
add_library(MatchGPULib src/MatchGPULib.cpp)
add_library(convolutionSeparable_gold src/convolutionSeparable_gold.cpp)
FOREACH(lib ${LIB_NAMES})
target_link_libraries(MatchGPULib ${LIBS} )
target_link_libraries(convolutionSeparable_gold ${LIBS})
ENDFOREACH(lib)
target_link_libraries(MatchGPULib MatchLib convolutionSeparable_gold)
FOREACH(lib ${LIB_NAMES})
target_link_libraries(MatchLib ${LIBS})
ENDFOREACH(lib)

add_executable(RHGPU_matcher src/RHGPU_matcher.cpp)
#target_link_libraries(RHGPU_matcher shrutil_x86_64 cutil_x86_64)
add_dependencies(RHGPU_matcher rh_stereo_proc_generate_messages_cpp)
target_link_libraries(RHGPU_matcher MatchGPULib ${catkin_LIBRARIES})
#endif (CUDA_FOUND)


