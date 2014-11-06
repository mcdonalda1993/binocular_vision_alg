"""
----
Gerardo Aragon on 08/2013.
Copyright (c) 2013 Gerardo Aragon. All rights reserved.
"""

import roslib
roslib.load_manifest('rh_calibration')
import rospy
import smach
from smach import State
from smach_ros import ServiceState
from rh_calibration.srv import CameraCalibration
from rh_calibration.srv import HandEyeCalibration
from rh_integration.srv import RobotIntegration
from rh_cameras.msg import CamerasSync

#__all__ = ['capture_images_state', 'process_target_state', 'camera_calibration_state', 'handeye_state']

class capture_images_state(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['topic'])

    def execute(self, userdata):
        rospy.loginfo("Capturing images")
        pub_acqImages = rospy.Publisher(userdata.topic, CamerasSync)
        acqString = CamerasSync()
        acqString.timeStamp = rospy.get_rostime()
        acqString.data = "full";
        pub_acqImages.publish(acqString);
        return 'succeeded'
        

class process_target_state(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/RH/calibration/processTarget', CameraCalibration, request_slots=['doIt'], input_keys=['noImages'], response_cb=self._res_cb, response_slots=['status_message', 'success'])
        self.counter = 1;

    def _res_cb(self, userdata, res):
        rospy.loginfo("@ image %d", self.counter)
        if res.success == True:
            if self.counter >= userdata.noImages:
                rospy.loginfo(res.status_message)
                rospy.loginfo('Entering to camera calibration')
                rospy.sleep(1.)
                return 'succeeded'
            else:
                self.counter = self.counter + 1; 
                rospy.loginfo(res.status_message)
                rospy.sleep(1.)
                return 'preempted'
        else:
            if self.counter >= userdata.noImages:
                rospy.logerror(res.status_message)
                rospy.sleep(1.)
                return 'aborted'
            else:
                self.counter = self.counter + 1; 
                rospy.loginfo(res.status_message)
                rospy.sleep(1.)
                return 'preempted'
            
            
class camera_calibration_state(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/RH/calibration/cameraCalibration', CameraCalibration, request_slots=['doIt'],
                              response_cb=self._res_cb, response_slots=['status_message', 'success'])

    def _res_cb(self, userdata, res):
        if res.success == True:
            rospy.loginfo(res.status_message)
            return 'succeeded'
        else:
            rospy.logerror(res.status_message)
            return 'aborted'
    

class handeye_state(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/RH/calibration/captureHandEye', HandEyeCalibration, request_slots=['doIt'],
                              response_cb=self._res_cb, response_slots=['status_message', 'success'])
        self.counter = 1;

    def _res_cb(self, userdata, res):
        rospy.loginfo("@ image %d", self.counter)
        rospy.loginfo(res.success)
        if res.success == 2:
            rospy.loginfo(res.status_message)
            return 'succeeded'
        
        if res.success == 1:
            self.counter = self.counter + 1; 
            rospy.loginfo(res.status_message)
            rospy.sleep(1.)
            return 'preempted'
        
        if res.success == 0:
            rospy.logerror(res.status_message)
            return 'aborted'

class robotInit_state(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/RH/integration/RobotInit', RobotIntegration, request_slots=['arm'], output_keys=['noImages'],
                              response_cb=self._res_cb, response_slots=['numPos'])
        self.counter = 0;

    def _res_cb(self, userdata, res):
        
        rospy.loginfo("Number of poses %d", res.numPos)
        if res.numPos > 0:
            userdata.noImages = res.numPos
            return 'succeeded'
        
        if res.numPos < 0:
            return 'aborted'

class robotMove_state(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/RH/integration/MoveRobotRandom', RobotIntegration, input_keys=['noImages'],
                              request_slots=['arm'], response_cb=self._res_cb, response_slots=['numPos', 'succeed'])
        self.counter = 0;

    def _res_cb(self, userdata, res):
        rospy.loginfo("@ iteration %d", res.numPos)
        self.counter = self.counter + 1
        if res.succeed == False:
            self.counter = self.counter - 1
            return 'aborted'
        
        if self.counter > userdata.noImages:
            return 'succeeded'
        
        if res.numPos <= userdata.noImages and self.counter <= userdata.noImages:
            return 'preempted'



