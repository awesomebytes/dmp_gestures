#!/usr/bin/python
"""
Created on 19/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com


"""
# System dependencies
from os.path import isfile
import time
import copy
# ROS dependencies
import rospy
# Message dependencies
from moveit_msgs.msg import DisplayTrajectory
# This example dependencies
from dmp_training import LearnFromJointState
from dmp_generation import gestureGeneration
from dmp_execution import gestureExecution
from dmp_generation import dmpPlanTrajectoryPlotter
from pal_control_msgs.msg import ActuatorCurrentLimit

import smach
from speech_states.listen_to import ListenToSM
from speech_states.say import text_to_say
from speech_states.activate_asr import ActivateASR
from speech_states.deactivate_asr import DeactivateASR
from speech_states.read_asr import ReadASR
from speech_states.activate_keyword_asr import ActivateKeywordASR

GRAMMAR_NAME = 'dmp/'

# Some color codes for prints, from http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
ENDC = '\033[0m'
FAIL = '\033[91m'
OKGREEN = '\033[92m'

LEARN_GESTURE_SENTENCE = "learn gesture"
START_SENTENCE = "start"
STOP_SENTENCE = "wait here"
EXECUTE_SENTENCE = "now"

GESTURE_NAME= "gesture_name"



CURR_LIMIT_TOPIC = '/current_limit_controller/command'
CURR_LIMIT_STATE_TOPIC = '/current_limit_controller/state' 

class setJointsCurrent(smach.State):
    def __init__(self, curr_limit=0.01, joints=[]):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
        self.curr_pub = rospy.Publisher(CURR_LIMIT_TOPIC, ActuatorCurrentLimit)
        rospy.sleep(0.3) # Wait so the publisher can publish
        if len(joints) == 0:
            self.joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                               'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                               'arm_right_7_joint']
        else:
            self.joints = joints
        self.curr_limit = curr_limit

    def execute(self, userdata):
        acl = ActuatorCurrentLimit()
        motor_names = []
        for joint in self.joints:
            motor_names.append(joint.replace('joint','motor'))
        acl.actuator_names = motor_names
        acl.current_limits = [self.curr_limit] * len(motor_names)
        curr_limit_values = rospy.wait_for_message(CURR_LIMIT_STATE_TOPIC, ActuatorCurrentLimit)
        to_send_msg = self.merge_values(curr_limit_values, acl)
        rospy.loginfo("Sending to curr limit topic: " + str(to_send_msg))
        self.curr_pub.publish(to_send_msg)
        return 'succeeded'

    def merge_values(self, msg_with_all_joints, msg_with_new_values):
        """Given a message with all the joints (got from the state)
        set the new values found in the msg_with_new_values.
        This is needed because the actual current controller needs to feeded
        with a message with the current for all joints"""
        new_limits_list = [] # As we can't update the value in a tuple (Python) we need to form a new list
        for limit_val in msg_with_all_joints.current_limits:
            new_limits_list.append(limit_val)
            
        for motor_name, limit_val in zip(msg_with_new_values.actuator_names, msg_with_new_values.current_limits):
            idx_motor = msg_with_all_joints.actuator_names.index(motor_name)
            new_limits_list[idx_motor] = limit_val
        msg_with_all_joints.current_limits = new_limits_list
        return msg_with_all_joints

class gestureExecuterFromPlace(smach.State):
    def __init__(self, gesture_executor_object, gesture_generator_object):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=["gesture_dict"])
        # Initialize stuff
        self.ge = gesture_executor_object 
        self.gg = gesture_generator_object

    def execute(self, userdata):
        joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        curr_joints_pose = self.ge.getCurrentJointsPose(joints)
        gesture_dict = userdata.gesture_dict
        plan = self.gg.getPlan(curr_joints_pose, gesture_dict["final_pose"], tau=gesture_dict["duration"], dt=0.02)
        robot_traj = self.ge.robotTrajectoryFromPlan(plan, joints)
        result = self.ge.sendTrajectory(robot_traj, True)
        if result:
            return 'succeeded'
        else:
            return 'aborted'

class gestureTrainer(smach.State):
    def __init__(self, gesture_generator_object):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=["gesture_dict"])
        # Initialize stuff here 
        self.gg = gesture_generator_object

    def execute(self, userdata):
        joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        rospy.loginfo("Now we will generate a DMP from this bag.")
        gesture_dict = self.gg.loadGestureFromBagJointStatesAndRemoveJerkiness(GESTURE_NAME +".bag", joints)
        userdata.gesture_dict = gesture_dict
        return 'succeeded'


class startRecordGesture(smach.State):
    def __init__(self, learn_from_joint_state_object):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
        # Initialize stuff here for the recorder
        self.lfjs = learn_from_joint_state_object # LearnFromJointState()

    def execute(self, userdata):
        # start recording here
        joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.lfjs.start_learn(GESTURE_NAME, joints, GESTURE_NAME)
        
        return 'succeeded'

class stopRecordGesture(smach.State):
    def __init__(self, learn_from_joint_state_object):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             output_keys=['motion_info'])
        # Initialize stuff here for the recorder
        self.lfjs = learn_from_joint_state_object # LearnFromJointState()

    def execute(self, userdata):
        motion_info = self.lfjs.stop_learn()
        userdata.motion_info = motion_info
        return 'succeeded'

class LearnGestureByVoice(smach.StateMachine):
    """
    Waits for saying learn gesture.
    TODO: Then what arm, make joints loose for that arm first warning about it.
    Wait for start learning sentence.
    Start recording joints there.
    When says stop learning.
    Stop recording joints and calculate dmp.
    While doing that, tell user to put arm in normal position.
    And restore joints current.
    Tell the user if he wants to play the motion when it's computed.
    When yes, play the motion from there.
    """
    
    def __init__(self):
        smach.StateMachine.__init__(self, ['succeeded', 'preempted', 'aborted'])

        with self:
            self.lfjs = LearnFromJointState()
            self.gesture_generator = gestureGeneration()
            self.gesture_executor = gestureExecution()
            # Listen the first question
            self.userdata.grammar_name = GRAMMAR_NAME
            
#             smach.StateMachine.add('ActivateKWASR_learn_gesture',
#                     ActivateKeywordASR(keyword=LEARN_GESTURE_SENTENCE),
#                     transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 'preempted': 'preempted'})
            smach.StateMachine.add(
                 'Explain_how_it_works',
                 text_to_say("Hello, you will teach me a gesture. First say start to record and wait here when the movement is finished. I'm going to make my right arm loose now."),
                 transitions={'succeeded': 'make_right_arm_loose', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'make_right_arm_loose',
                 setJointsCurrent(curr_limit=0.01),
                 transitions={'succeeded': 'ActivateKWASR_start', 'aborted': 'aborted'})

            smach.StateMachine.add('ActivateKWASR_start',
                    ActivateKeywordASR(keyword=START_SENTENCE),
                    transitions={'succeeded': 'Say_ready', 'aborted': 'aborted', 'preempted': 'preempted'})
            
#             smach.StateMachine.add('What_arm_to_learn_from',
#                     ActivateASR(),
#                     transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 'preempted': 'preempted'})


            smach.StateMachine.add(
                 'Say_ready',
                 text_to_say("I'm ready to get the start order"),
                 transitions={'succeeded': 'listen_to_start', 'aborted': 'aborted'})
            
            
            smach.StateMachine.add(
                'listen_to_start',
                ReadASR(), # We can only listen to one word, so we dont care about the content
                transitions={'succeeded': 'ActivateKWASR_stop', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 

            smach.StateMachine.add('ActivateKWASR_stop',
                    ActivateKeywordASR(keyword=STOP_SENTENCE),
                    transitions={'succeeded': 'Say_recording', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                 'Say_recording',
                 text_to_say("Ok, I'll start recording... now."),
                 transitions={'succeeded': 'record_gesture', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'record_gesture',
                 startRecordGesture(self.lfjs),
                 transitions={'succeeded': 'listen_to_stop', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                'listen_to_stop',
                ReadASR(),
                transitions={'succeeded': 'stop_recording_gesture', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
            
            smach.StateMachine.add(
                 'stop_recording_gesture',
                 stopRecordGesture(self.lfjs),
                 transitions={'succeeded': 'Say_recorded_and_processing', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'Say_recorded_and_processing',
                 text_to_say("Got the gesture, now I'm processing it."),
                 transitions={'succeeded': 'train_gesture', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'train_gesture',
                 gestureTrainer(self.gesture_generator),
                 transitions={'succeeded': 'Say_gesture_learnt', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'Say_gesture_learnt',
                 text_to_say("Ok, I learnt the gesture, now I know kung fu. Put my arm where you want me to start the motion and say now."),
                 transitions={'succeeded': 'ActivateKWASR_now', 'aborted': 'aborted'})
            
            smach.StateMachine.add('ActivateKWASR_now',
                    ActivateKeywordASR(keyword=EXECUTE_SENTENCE),
                    transitions={'succeeded': 'listen_to_now', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'listen_to_now',
                ReadASR(),
                transitions={'succeeded': 'make_right_arm_stiff', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
        
            smach.StateMachine.add(
                 'make_right_arm_stiff',
                 setJointsCurrent(curr_limit=1.0),
                 transitions={'succeeded': 'Say_executing', 'aborted': 'aborted'})
        
            smach.StateMachine.add(
                 'Say_executing',
                 text_to_say("Ok, executing!"),
                 transitions={'succeeded': 'execute_gesture_from_current_pose', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'execute_gesture_from_current_pose',
                 gestureExecuterFromPlace(self.gesture_executor, self.gesture_generator),
                 transitions={'succeeded': 'Say_try_from_another_position', 'aborted': 'Say_fail_exec'})
            
            smach.StateMachine.add(
                 'Say_try_from_another_position',
                 text_to_say("That went well, try again from a different position. I'm making my arm loose again. When you want me to execute again the gesture say now."),
                 transitions={'succeeded': 'make_right_arm_loose_again', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'Say_fail_exec',
                 text_to_say("The safety check of this movement failed, try again from a safer position. Making right arm loose again, say now when you want to execute."),
                 transitions={'succeeded': 'make_right_arm_loose_again', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'make_right_arm_loose_again',
                 setJointsCurrent(curr_limit=0.01),
                 transitions={'succeeded': 'ActivateKWASR_now', 'aborted': 'aborted'})
            
            
if __name__ == '__main__':
    rospy.init_node('learn_gesture_by_voice_node')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add(
            'learn_gesture_by_voice',
            LearnGestureByVoice(),
            transitions={'succeeded': 'succeeded','preempted':'preempted', 'aborted':'aborted'})
    
    sm.execute()
    rospy.spin()