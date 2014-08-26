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
from geometry_msgs.msg import Point
# This example dependencies
from dmp_training import LearnFromJointState
from dmp_generation import gestureGeneration
from dmp_execution import gestureExecution
from dmp_generation import dmpPlanTrajectoryPlotter
from pal_control_msgs.msg import ActuatorCurrentLimit

import smach
from speech_states.listen_to import ListenToSM
from speech_states.listen_and_repeat import ListenRepeatSM
from speech_states.say_yes_or_no import SayYesOrNoSM_2
from speech_states.say import text_to_say
from speech_states.activate_asr import ActivateASR
from speech_states.deactivate_asr import DeactivateASR
from speech_states.read_asr import ReadASR
from speech_states.activate_keyword_asr import ActivateKeywordASR

from control_msgs.msg import PointHeadAction, PointHeadGoal, PointHeadActionGoal
from smach_ros.simple_action_state import SimpleActionState

CHOOSE_GROUP_GRAMMAR_NAME = 'dmp/choose_group'

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

class sendPointHeadGoalByPublisher(smach.State):
    def __init__(self, point):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
        self.head_pub = rospy.Publisher('/head_controller/point_head_action/goal', PointHeadActionGoal)
        rospy.sleep(0.3) # Wait so the publisher can publish
        self.point = point

    def execute(self, userdata):
        phag = PointHeadActionGoal()
        phag.goal.min_duration = rospy.Duration(2.0)
        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "stereo_link"
        phag.goal.target.header.frame_id = "base_link"
        phag.goal.target.point =self.point
        self.head_pub.publish(phag)
        return 'succeeded'
    
    
class prepareGroups(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['asr_userSaid', 'asr_userSaid_tags'],
                             output_keys=["joints"])
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated

    def execute(self, userdata):
        rospy.logwarn("asr_userSaidtags is: " + str(userdata.asr_userSaid_tags))
        side1 = side2 = None
        group1 = group2 = group3 = group4 = None
        # As I don't know if I'm assured this will come in any order...
        for tag in userdata.asr_userSaid_tags:
            if tag.key == "side1":
                side1 = tag.value
            elif tag.key == "side2":
                side2 = tag.value
            elif tag.key == "group1":
                group1 = tag.value
            elif tag.key == "group2":
                group2 = tag.value
            elif tag.key == "group3":
                group3 = tag.value
            elif tag.key == "group4":
                group4 = tag.value
        
        tmp_joints = []
        tmp_joints.extend( getattr(self, side1 +"_"+ group1) ) #this is getting self class var "right_arm"
        if group2 != None:
            tmp_joints.extend( getattr(self, side1 +"_"+ group2) )
        if group3 != None:
            tmp_joints.extend( getattr(self, side2 +"_"+ group3) )
        if group4 != None:
            tmp_joints.extend( getattr(self, side2 +"_"+ group4) )
        rospy.loginfo("We are going to ask for joints: " + str(tmp_joints))
        userdata.joints = tmp_joints
        return 'succeeded'

class setJointsCurrent(smach.State):
    def __init__(self, curr_limit=None, joints=[]):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['joints', 'curr_limit'])
        self.curr_pub = rospy.Publisher(CURR_LIMIT_TOPIC, ActuatorCurrentLimit)
        rospy.sleep(0.3) # Wait so the publisher can publish
#         if len(joints) == 0:
#             self.joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
#                                'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
#                                'arm_right_7_joint']
#         else:
        self.joints = joints
        self.curr_limit = curr_limit

    def execute(self, userdata):
        # arguments have priority over keys
        self.joints = self.joints if len(self.joints) > 0 else userdata.joints
        self.curr_limit = self.curr_limit if self.curr_limit != None else userdata.curr_limit
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
    def __init__(self, gesture_executor_object, gesture_generator_object, joints=[]):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=["gesture_dict", 'joints'])
        # Initialize stuff
        self.ge = gesture_executor_object 
        self.gg = gesture_generator_object
        self.joints = joints

    def execute(self, userdata):
        self.joints = self.joints if len(self.joints) > 0 else userdata.joints
        curr_joints_pose = self.ge.getCurrentJointsPose(self.joints)
        gesture_dict = userdata.gesture_dict
        plan = self.gg.getPlan(curr_joints_pose, gesture_dict["final_pose"], tau=gesture_dict["duration"], dt=0.02)
        robot_traj = self.ge.robotTrajectoryFromPlan(plan, self.joints)
        result = self.ge.sendTrajectory(robot_traj, True)
        if result:
            return 'succeeded'
        else:
            return 'aborted'

class gestureTrainer(smach.State):
    def __init__(self, gesture_generator_object, joints=[]):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['joints'],
                             output_keys=["gesture_dict"])
        # Initialize stuff here 
        self.gg = gesture_generator_object
        self.joints = joints

    def execute(self, userdata):
        self.joints = self.joints if len(self.joints) > 0 else userdata.joints
        rospy.loginfo("Now we will generate a DMP from this bag.")
        gesture_dict = self.gg.loadGestureFromBagJointStatesAndRemoveJerkiness(GESTURE_NAME +".bag", self.joints)
        userdata.gesture_dict = gesture_dict
        return 'succeeded'


class startRecordGesture(smach.State):
    def __init__(self, learn_from_joint_state_object, joints=[]):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'],
                             input_keys=['joints'])
        # Initialize stuff here for the recorder
        self.lfjs = learn_from_joint_state_object # LearnFromJointState()
        self.joints = joints

    def execute(self, userdata):
        # start recording here
        self.joints = self.joints if len(self.joints) > 0 else userdata.joints
        self.lfjs.start_learn(GESTURE_NAME, self.joints, GESTURE_NAME)
        
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
            
#             smach.StateMachine.add('ActivateKWASR_learn_gesture',
#                     ActivateKeywordASR(keyword=LEARN_GESTURE_SENTENCE),
#                     transitions={'succeeded': 'check_loop', 'aborted': 'aborted', 'preempted': 'preempted'})
            smach.StateMachine.add(
                 'Explain_how_it_works',
                 text_to_say("Hello, you will teach me a new gesture.\
                  First tell me which arms and hands to use. Remember, I only have two!", wait=False),
                 transitions={'succeeded': 'listen_groups_to_use', 'aborted': 'aborted'})

            smach.StateMachine.add(
                'listen_groups_to_use',
                ListenRepeatSM(grammar=CHOOSE_GROUP_GRAMMAR_NAME),
                transitions={'succeeded': 'confirm_groups', 'aborted': 'aborted', 
                'preempted': 'preempted'})

            smach.StateMachine.add(
                'confirm_groups',
                SayYesOrNoSM_2(),
                transitions={'succeeded': 'prepare_groups_to_make_loose', 'aborted': 'listen_groups_to_use', 
                'preempted': 'preempted'})
            
                       
            smach.StateMachine.add(
                 'prepare_groups_to_make_loose',
                 prepareGroups(),
                 transitions={'succeeded': 'Warn_loose_joints', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'Warn_loose_joints',
                 text_to_say("I'm going to lower the current on my joints, be careful.", wait=False),
                 transitions={'succeeded': 'look_to_place', 'aborted': 'aborted'})
            
#             def look_point_goal_cb(userdata, goal):
#                 phg = PointHeadGoal()
#                 phg.min_duration = rospy.Duration(2.0)
#                 phg.pointing_axis.x = 1.0
#                 phg.pointing_frame = "stereo_link"
#                 phg.target.header.frame_id = "base_link"
#                 phg.target.point.x = 1.0
#                 phg.target.point.y = -1.0
#                 phg.target.point.z = 1.0
#                 return phg
#             
#             smach.StateMachine.add(
#                  'look_to_place',
#                  SimpleActionState('/head_controller/point_head_action',
#                                    PointHeadAction,
#                                    goal_cb=look_point_goal_cb),
#                  transitions={'succeeded': 'make_groups_loose', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'look_to_place',
                 sendPointHeadGoalByPublisher(Point(1.0, -1.0, 1.0)),
                 transitions={'succeeded': 'make_groups_loose', 'aborted': 'aborted'})
            

            smach.StateMachine.add(
                 'make_groups_loose',
                 setJointsCurrent(curr_limit=0.01),
                 transitions={'succeeded': 'ActivateKWASR_start', 'aborted': 'aborted'})

            smach.StateMachine.add('ActivateKWASR_start',
                    ActivateKeywordASR(keyword=START_SENTENCE),
                    transitions={'succeeded': 'Say_ready', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                 'Say_ready',
                 text_to_say("Please say, start, when you want to start recording. Then say, wait here, when you finish the gesture."),
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
                 text_to_say("Got the gesture, now I'm going to process it. Give me some seconds please.", wait=False),
                 transitions={'succeeded': 'look_to_place_middle', 'aborted': 'aborted'})

#             def look_point_goal_cb2(userdata, goal):
#                 phg = PointHeadGoal()
#                 phg.min_duration = rospy.Duration(2.0)
#                 phg.pointing_axis.x = 1.0
#                 phg.pointing_frame = "stereo_link"
#                 phg.target.header.frame_id = "base_link"
#                 phg.target.point.x = 1.0
#                 phg.target.point.y = 0.0
#                 phg.target.point.z = 1.7
#                 return phg
#             
#             smach.StateMachine.add(
#                  'look_to_place_middle',
#                  SimpleActionState('/head_controller/point_head_action',
#                                    PointHeadAction,
#                                    goal_cb=look_point_goal_cb2),
#                  transitions={'succeeded': 'train_gesture', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'look_to_place_middle',
                 sendPointHeadGoalByPublisher(Point(1.0, 0.0, 1.7)),
                 transitions={'succeeded': 'train_gesture', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'train_gesture',
                 gestureTrainer(self.gesture_generator),
                 transitions={'succeeded': 'Say_gesture_learnt', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'Say_gesture_learnt',
                 text_to_say("Ok, I learnt the gesture, as Neeo would say, now I know kung fu. \
                 Put my arm where you want me to start the motion and say now."),
                 transitions={'succeeded': 'ActivateKWASR_now', 'aborted': 'aborted'})
            
            smach.StateMachine.add('ActivateKWASR_now',
                    ActivateKeywordASR(keyword=EXECUTE_SENTENCE),
                    transitions={'succeeded': 'listen_to_now', 'aborted': 'aborted', 'preempted': 'preempted'})
            
            smach.StateMachine.add(
                'listen_to_now',
                ReadASR(),
                transitions={'succeeded': 'make_group_stiff', 'aborted': 'aborted', 
                'preempted': 'preempted'}) 
        
            smach.StateMachine.add(
                 'make_group_stiff',
                 setJointsCurrent(curr_limit=1.0),
                 transitions={'succeeded': 'Say_executing', 'aborted': 'aborted'})
        
            smach.StateMachine.add(
                 'Say_executing',
                 text_to_say("Ok, I'm going to execute the gesture from my current configuration!", wait=False),
                 transitions={'succeeded': 'execute_gesture_from_current_pose', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'execute_gesture_from_current_pose',
                 gestureExecuterFromPlace(self.gesture_executor, self.gesture_generator),
                 transitions={'succeeded': 'Say_try_from_another_position', 'aborted': 'Say_fail_exec'})
            
            smach.StateMachine.add(
                 'Say_try_from_another_position',
                 text_to_say("That went well, try again from a different position. I'm making my joints loose again. When you want me to execute again the gesture say now."),
                 transitions={'succeeded': 'make_groups_loose_again', 'aborted': 'aborted'})
            
            smach.StateMachine.add(
                 'Say_fail_exec',
                 text_to_say("The safety check of this movement failed, try again from a safer position. Making my joints loose again, say now when you want to execute."),
                 transitions={'succeeded': 'make_groups_loose_again', 'aborted': 'aborted'})

            smach.StateMachine.add(
                 'make_groups_loose_again',
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