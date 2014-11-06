cmake_minimum_required(VERSION 2.8.3)
project(rh_calibration)
SET(CMAKE_MODULE_PATH /usr/local/lib/cmake )

find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  camera_info_manager
  cv_bridge
  image_transport
  rh_cameras
  rh_ptu
  roscpp
  rospy
  smach
  smach_ros
  std_msgs
  tf
  message_generation
  rh_integration
)
#visp_hand2eye_calibration
#visp_bridge

catkin_python_setup()

add_service_files(
    FILES
    CameraCalibration.srv
    HandEyeCalibration.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS camera_calibration_parsers camera_info_manager cv_bridge image_transport rh_cameras rh_ptu roscpp rospy smach smach_ros std_msgs tf message_runtime rh_integration
)
#visp_hand2eye_calibration visp_bridge

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
find_package(cvsba REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(RH_CALIBRATION_SOURCES
    src/calibration_nodes/RH_calibration_service_version.cpp
    src/calibration_nodes/camera_calibration.cpp
    src/calibration_nodes/handeye_calibration.cpp
)

add_executable(RHcalibration_services ${RH_CALIBRATION_SOURCES})
add_dependencies(RHcalibration_services rh_calibration_generate_messages_cpp)
add_dependencies(RHcalibration_services rh_cameras_generate_messages_cpp)
add_dependencies(RHcalibration_services rh_ptu_generate_messages_cpp)
add_dependencies(RHcalibration_services rh_integration_generate_messages_cpp)
target_link_libraries(RHcalibration_services boost_signals)
target_link_libraries(RHcalibration_services ${OpenCV_LIBRARIES} ${cvsba_LIBS})
target_link_libraries(RHcalibration_services ${catkin_LIBRARIES})

add_executable(RHcalibration_young_capture src/calibration_nodes/RHcalibration_young_capture.cpp)
add_dependencies(RHcalibration_young_capture rh_calibration_generate_messages_cpp)
add_dependencies(RHcalibration_young_capture rh_cameras_generate_messages_cpp)
add_dependencies(RHcalibration_young_capture rh_ptu_generate_messages_cpp)
add_dependencies(RHcalibration_young_capture rh_integration_generate_messages_cpp)
target_link_libraries(RHcalibration_young_capture boost_signals)
target_link_libraries(RHcalibration_young_capture ${OpenCV_LIBRARIES} ${cvsba_LIBS})
target_link_libraries(RHcalibration_young_capture ${catkin_LIBRARIES})

add_executable(RHecho src/utils/rh_echo.cpp)
target_link_libraries(RHecho ${OpenCV_LIBRARIES})
target_link_libraries(RHecho ${catkin_LIBRARIES})

add_executable(RHecho_fixed src/utils/rh_echo_fixed.cpp)
target_link_libraries(RHecho_fixed ${OpenCV_LIBRARIES})
target_link_libraries(RHecho_fixed ${catkin_LIBRARIES})

add_executable(RHsavetfpoints src/utils/RH_savetfpoints.cpp)
target_link_libraries(RHsavetfpoints ${OpenCV_LIBRARIES})
target_link_libraries(RHsavetfpoints ${catkin_LIBRARIES})



