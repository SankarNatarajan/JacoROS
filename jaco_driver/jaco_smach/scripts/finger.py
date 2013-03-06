#!/usr/bin/env python
"""
Description:
    Create open and close jaco finger state state machine.
	state 1: open the gripper
	state 2: close the gripper
	state 3: open the gripper
	state 4: close the gripper

Usage:
    $> python finger.py

"""

import roslib; roslib.load_manifest('jaco_smach')
import rospy
import smach
import smach_ros

from smach import *
from smach_ros import *
from smach_msgs.msg import *
from actionlib import *
from actionlib.msg import *
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Pose, PointStamped
from arm_navigation_msgs.msg import MoveArmGoal, MoveArmAction
from std_msgs.msg import String

from kinematics_msgs.srv import *
from arm_navigation_msgs.msg import *
from jaco.msg import FingerMovementGoal
from jaco.msg import FingerMovementAction

from time import sleep


# define state MoveAction
class MoveFingerAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['set_finger'], 
                             output_keys=[''])
    
    def execute(self, userdata):	
        self.ac = actionlib.SimpleActionClient('finger_action', FingerMovementAction)
        self.ac.wait_for_server()


        # fill the goal pose       
        moveFinger_goal = FingerMovementGoal()
	
        moveFinger_goal.task = userdata.set_finger	
                
        self.ac.send_goal_and_wait(goal=moveFinger_goal, execute_timeout=rospy.Duration(50))
        
        state = self.ac.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal achieved successfully')	    
            return 'succeeded'
        elif state == GoalStatus.PREEMPTED:
            rospy.loginfo('Task preempted')
            return 'preempted'
        elif state in [GoalStatus.RECALLED, GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST]:
            rospy.loginfo('Task aborted')
            return 'aborted'

class SleepAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=[''], 
                             output_keys=[''])
    
    def execute(self, userdata):
	sleep(1.0)
	return 'succeeded'

def main():
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['finger_succeeded','finger_aborted'])

	sm.userdata.setFingerCLOSE = "CLOSE"
	sm.userdata.setFingerOPEN = "OPEN"	
 
        sm.set_initial_state(['state1'])
	
	# Open the container
	with sm:			
   
		# Add states to the container

		smach.StateMachine.add('state1', MoveFingerAction(),
							transitions = {'preempted':'finger_aborted', 'aborted':'finger_aborted', 'succeeded':'sleep1'},
							remapping={'set_finger':'setFingerOPEN'})

		smach.StateMachine.add('sleep1', SleepAction(),
							transitions = {'preempted':'finger_aborted', 'aborted':'finger_aborted', 'succeeded':'state2'})

		smach.StateMachine.add('state2', MoveFingerAction(), 
							transitions={'preempted':'finger_aborted','aborted':'finger_aborted','succeeded':'state3'},
							remapping={'set_finger':'setFingerCLOSE'})

		smach.StateMachine.add('state3', MoveFingerAction(), 
							  transitions={'preempted':'finger_aborted','aborted':'finger_aborted','succeeded':'sleep2'},
							    remapping={'set_finger':'setFingerOPEN'})

		smach.StateMachine.add('sleep2', SleepAction(),
							transitions = {'preempted':'finger_aborted', 'aborted':'finger_aborted', 'succeeded':'state4'})

		smach.StateMachine.add('state4', MoveFingerAction(), 
							transitions={'preempted':'finger_aborted','aborted':'finger_aborted','succeeded':'finger_succeeded'},
							remapping={'set_finger':'setFingerCLOSE'})

                smach.StateMachine.add('state5', MoveFingerAction(),
                                                          transitions={'preempted':'finger_aborted','aborted':'finger_aborted','succeeded':'finger_succeeded'},
                                                            remapping={'set_finger':'setFingerOPEN'})

                smach.StateMachine.add('state6', MoveFingerAction(),
                                                        transitions={'preempted':'finger_aborted','aborted':'finger_aborted','succeeded':'finger_succeeded'},
                                                        remapping={'set_finger':'setFingerCLOSE'})
		  
		
		# Create and start the introspection server
		sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
		sis.start()


	    	# Execute SMACH plan
	    	outcome = sm.execute()

		# Wait for ctrl-c to stop the application
		rospy.spin()
		sis.stop()


if __name__ == '__main__':
	rospy.init_node('finger_state_machine',anonymous=False, log_level=rospy.INFO, disable_signals=True)		
	main()

