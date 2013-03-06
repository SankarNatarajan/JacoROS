#!/usr/bin/env python
"""
Description:
    A simple test program. move the joints and open/close fingers.
	state 1: move joints
	state 2: open the gripper
Usage:
    $> python move_joint.py

"""

import roslib; roslib.load_manifest('jaco_smach')
import rospy
import smach
import smach_ros

from smach import *
from smach_ros import *
from smach_msgs.msg import *
#from smach_ros import SimpleActionState
from actionlib import *
from actionlib.msg import *
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Pose, PointStamped
from arm_navigation_msgs.msg import MoveArmGoal, MoveArmAction
from std_msgs.msg import String

from arm_navigation_msgs.msg import JointConstraint
from arm_navigation_msgs import arm_navigation_msgs_utils
from jaco.srv import SetFingers

cur_jtang = JointState
counter = 0


def jtangCallback(msg):
	
	global cur_jtang	
	global counter
	counter += 1
	cur_jtang = msg
	rospy.loginfo(rospy.get_name()+"I heard %f %f %f %f %f %f",msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
	if counter > 3:	
		counter = 0	
		sub.unregister()




def main():
	

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['pickPlace_succeeded','pickPlace_aborted'])

	sm.set_initial_state(['Move_jacoArm'])

	# Open the container
	with sm:

		# move to a point			
		def move_goal_cb(userdata, goal):
			
			move_goal = MoveArmGoal()

			joint_names = ['jaco_joint_1' , 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6' ]
			move_goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
			move_goal.motion_plan_request.group_name = "jaco_arm"
			move_goal.motion_plan_request.num_planning_attempts = 1
		        move_goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.)
        		move_goal.motion_plan_request.planner_id = ""
        		move_goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        		move_goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
        		for i in range(len(joint_names)):
            			move_goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i]
            			move_goal.motion_plan_request.goal_constraints.joint_constraints[i].position = cur_jtang.position[i]
            			move_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.08
            			move_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.08				

			move_goal.motion_plan_request.goal_constraints.joint_constraints[0].position = cur_jtang.position[0] - 0.2	
		        rospy.loginfo(rospy.get_name()+"I heard %f %f %f %f %f %f",cur_jtang.position[0],cur_jtang.position[1],cur_jtang.position[2],cur_jtang.position[3],cur_jtang.position[4],cur_jtang.position[5])
			return move_goal

		# open the gripper
		def finger_request_cb(userdata, request):
			finger_request = SetFingers().Request
			finger_request.task = "OPEN"			
			return finger_request

   
		# Add states to the container

		smach.StateMachine.add('Move_jacoArm', smach_ros.SimpleActionState('move_jaco_arm', MoveArmAction, goal_cb =move_goal_cb), transitions = {'preempted':'pickPlace_aborted', 'succeeded':'Open_finger', 'aborted':'pickPlace_aborted'})

		smach.StateMachine.add('Open_finger', ServiceState('set_fingers', SetFingers, request = "CLOSE"), transitions={'preempted':'pickPlace_aborted','aborted':'pickPlace_aborted','succeeded':'pickPlace_succeeded'})

	    	# Execute SMACH plan
	    	outcome = sm.execute()

		# Wait for ctrl-c to stop the application
		rospy.spin()
	


if __name__ == '__main__':
	rospy.init_node('get_joint_state_machine',anonymous=False, log_level=rospy.INFO, disable_signals=True)	
	sub = rospy.Subscriber("joint_states",JointState, jtangCallback)	
	#rospy.spin()
	#rospy.sleep(2.)
	main()

