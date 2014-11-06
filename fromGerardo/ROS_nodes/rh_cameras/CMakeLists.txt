cmake_minimum_required(VERSION 2.8.3)
project(rh_cameras)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

add_message_files(
   DIRECTORY msg
   FILES CamerasSync.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES SetCameraInfo.srv AcquireSrv.srv GetCameraInfo.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
)

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
#  LIBRARIES RHcam
  CATKIN_DEPENDS camera_calibration_parsers camera_info_manager cv_bridge image_transport roscpp sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GPHOTO2 REQUIRED libgphoto2)
include_directories(${GPHOTO2_INCLUDE_DIRS})
link_directories(${GPHOTO2_LIBRARY_DIRS})
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a cpp library
add_library(RHcam src/libRHcam/RHcam.cpp)
target_link_libraries(RHcam jpeg)
target_link_libraries(RHcam ${GPHOTO2_LIBRARIES})
target_link_libraries(RHcam ${OpenCV_LIBRARIES})

## Declare a cpp executable
# ** RHcam_left
add_executable(RHcam_left src/RHcamleft_node/RHcamleft_node.cpp)
add_dependencies(RHcam_left rh_cameras_generate_messages_cpp)
target_link_libraries(RHcam_left RHcam)
target_link_libraries(RHcam_left ${GPHOTO2_LIBRARIES})
target_link_libraries(RHcam_left ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_left ${catkin_LIBRARIES})

# ** RHcam_right
add_executable(RHcam_right src/RHcamright_node/RHcamright_node.cpp)
add_dependencies(RHcam_right rh_cameras_generate_messages_cpp)
target_link_libraries(RHcam_right RHcam)
target_link_libraries(RHcam_right ${GPHOTO2_LIBRARIES})
target_link_libraries(RHcam_right ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_right ${catkin_LIBRARIES})

# ** RHcam_resize (resizes captured images)
add_executable(RHcam_resize src/RHcam_resize/RHcam_resize.cpp)
target_link_libraries(RHcam_resize ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_resize ${catkin_LIBRARIES})

# ** RHcam_test (displays captured images)
add_executable(RHcam_test src/extra_functions/RHcam_test.cpp)
target_link_libraries(RHcam_test ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_test ${catkin_LIBRARIES})

#** RHcam_saveImgs (saves images locally)
add_executable(RHcam_saveImgs src/extra_functions/RHcam_saveImgs.cpp)
target_link_libraries(RHcam_saveImgs ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_saveImgs ${catkin_LIBRARIES})

#** RHcam_simulation (simulate image capturing for both cameras)
add_executable(RHcam_simulation src/simulation/get_images.cpp)
add_dependencies(RHcam_simulation rh_cameras_generate_messages_cpp)
target_link_libraries(RHcam_simulation ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_simulation ${catkin_LIBRARIES})

#** RHcam_publishHandEye (simulate image capturing for both cameras)
add_executable(RHcam_publishHandEye src/simulation/publishTransform.cpp)
target_link_libraries(RHcam_publishHandEye ${OpenCV_LIBRARIES})
target_link_libraries(RHcam_publishHandEye ${catkin_LIBRARIES})


