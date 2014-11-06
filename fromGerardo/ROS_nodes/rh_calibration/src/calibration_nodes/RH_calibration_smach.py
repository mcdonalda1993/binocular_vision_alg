#!/usr/bin/env python
"""
Inspired from clopema_calibration/src/xtion_calibration_sm_2.py Written by Libor Wagner
----
Gerardo Aragon on 08/2013.
Copyright (c) 2013 Gerardo Aragon. All rights reserved.
"""

NODE_PACKAGE = 'rh_calibration'
NODE_NAME = 'robothead_calibration_sm'

import roslib
roslib.load_manifest('rh_calibration')
import rospy, smach
#from clopema_planning_actions.msg import MA1400JointState
from clopema_smach import *
from smach import State
from smach import Sequence
#from clopema_smach import ProcessState
from rh_calibration.calibration_service import* 

class Done(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        
    def execute(self,userdara):
        rospy.loginfo('State machine closing down!')
        return 'succeeded'

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    
    #Main state machine
    sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'], connector_outcome='succeeded')
    
    sm.userdata.doIt = True;
    sm.userdata.noImages = 24;
    sm.userdata.topic = '/RH/cmd/acquire'
    
    with sm:
        smach.Sequence.add("INIT_CAMERAS", capture_images_state(), transitions={"succeeded":"PAUSE_INIT"}) #Check this (I do not why the topic does not get publish on the first call!
        smach.Sequence.add("PAUSE_INIT", PauseState(1), transitions={"succeeded":"CAPTURE_IMAGES"})
        
        smach.Sequence.add("CAPTURE_IMAGES", capture_images_state(), transitions={"succeeded":"PAUSE"})
        smach.Sequence.add("PAUSE", PauseState(1), transitions={"succeeded":"PROCESS_TARGET"})
        smach.Sequence.add("PROCESS_TARGET", process_target_state(), transitions={"succeeded":"CAMERA_CALIBRATION", "aborted":"FINISH", "preempted":"CAPTURE_IMAGES"})
        
        Sequence.add("CAMERA_CALIBRATION", camera_calibration_state(), transitions={"succeeded":"CAPTURE_IMAGES_HE", "aborted":"FINISH"})
        
        smach.Sequence.add("CAPTURE_IMAGES_HE", capture_images_state(), transitions={"succeeded":"PAUSE_HE"})
        smach.Sequence.add("PAUSE_HE", PauseState(2), transitions={"succeeded":"HAND_EYE"})
        smach.Sequence.add("HAND_EYE", handeye_state(), transitions={"succeeded":"FINISH", "preempted":"CAPTURE_IMAGES_HE", "aborted":"FINISH"})
        
        Sequence.add("FINISH", Done())
        
    sm.execute()
    
#     #For debug only
#     sis = smach_ros.IntrospectionServer('server_name', sm, '/RHcalibration_sm')
#     sis.start()
#     outcome = sm.execute()
#  
#     # Wait for ctrl-c to stop the application
#     rospy.spin()
#     sis.stop()
