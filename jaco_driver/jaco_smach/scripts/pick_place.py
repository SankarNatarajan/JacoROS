#!/usr/bin/env python
"""
Description:
	Create a simple pick and place state state machine. Generate 4 different set of pose from a given start pose and move to those pose and also uses gripper.
	state similar to move_arm_new.py.

Usage:
    $> python pick_place_new.py

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

from kinematics_msgs.srv import *
from arm_navigation_msgs.msg import *

from jaco.msg import FingerMovementGoal
from jaco.msg import FingerMovementAction

from time import sleep

jtag = [0 for j in range(6)]
jaco_position = [0 for j in range(3)]
jaco_orientation = [0 for j in range(4)]

send_position = [0 for j in range(3)]
send_orientation = [0 for j in range(4)]

cur_jtang = JointState
counter = 0

jacoStartPose = Pose
jacoEndPose = Pose


def jtangCallback(msg):
	
	global cur_jtang	
	global counter
	counter += 1
	cur_jtang = msg
	rospy.loginfo(rospy.get_name()+"I heard %f %f %f %f %f %f",msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
	for i in range(6):
		jtag[i] = cur_jtang.position[i]
	if counter > 5:	
		counter = 0			
		sub.unregister()


def poseConstraintToPositionOrientationConstraints(pose_constraint):
	position_constraint = PositionConstraint()

	orientation_constraint = OrientationConstraint()
	position_constraint.header = pose_constraint.header
	position_constraint.link_name = pose_constraint.link_name
	position_constraint.position = pose_constraint.pose.position
	position_constraint.constraint_region_shape.type = 0
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

	position_constraint.constraint_region_orientation.x = 0.0
	position_constraint.constraint_region_orientation.y = 0.0
	position_constraint.constraint_region_orientation.z = 0.0
	position_constraint.constraint_region_orientation.w = 1.0

	position_constraint.weight = 1.0


	orientation_constraint.header = pose_constraint.header
	orientation_constraint.link_name = pose_constraint.link_name
	orientation_constraint.orientation = pose_constraint.pose.orientation
	orientation_constraint.type = pose_constraint.orientation_constraint_type

	orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
	orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
	orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
	orientation_constraint.weight = 1.0

	return (position_constraint, orientation_constraint)


def addGoalConstraintToMoveArmGoal(pose_constraint, move_arm_goal):
	position_constraint, orientation_constraint = poseConstraintToPositionOrientationConstraints(pose_constraint);
	move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
	move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)



# generate new points
class GeneratePose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['generated_Pose1', 'generated_Pose2', 'generated_Pose3', 'generated_Pose4','aborted'],
                             input_keys=['generatePose_Startinput', 'generatePose_Endinput', 'input_counter'],
                             output_keys=['output_counter','generatePose_Startoutput', 'generatePose_Endoutput'])
        
    def execute(self, userdata):

        
	if (userdata.input_counter == 0):
		generatePose = userdata.generatePose_Startinput
		rospy.loginfo("Position 1 %f %f %f", userdata.generatePose_Startinput.position.x, userdata.generatePose_Startinput.position.y, userdata.generatePose_Startinput.position.z)

	        generatePose.position.z = generatePose.position.z - 0.25
		userdata.output_counter = userdata.input_counter+1
		userdata.generatePose_Startoutput = generatePose
		return 'generated_Pose1'
	elif (userdata.input_counter == 1):
		generatePose = userdata.generatePose_Startinput
		rospy.loginfo("Position 1 %f %f %f", userdata.generatePose_Startinput.position.x, userdata.generatePose_Startinput.position.y, userdata.generatePose_Startinput.position.z)

	        generatePose.position.z = generatePose.position.z + 0.25
		userdata.output_counter = userdata.input_counter+1
		userdata.generatePose_Startoutput = generatePose
		return 'generated_Pose2'
	elif (userdata.input_counter == 2):
		generatePose = userdata.generatePose_Endinput
		rospy.loginfo("Position 1 %f %f %f", userdata.generatePose_Endinput.position.x, userdata.generatePose_Endinput.position.y, userdata.generatePose_Endinput.position.z)

	        generatePose.position.z = generatePose.position.z - 0.25
		userdata.output_counter = userdata.input_counter+1
		userdata.generatePose_Endoutput = generatePose
		return 'generated_Pose3'
	elif (userdata.input_counter == 3):
		generatePose = userdata.generatePose_Endinput
		rospy.loginfo("Position 1 %f %f %f", userdata.generatePose_Endinput.position.x, userdata.generatePose_Endinput.position.y, userdata.generatePose_Endinput.position.z)

	        generatePose.position.z = generatePose.position.z + 0.25
		userdata.output_counter = userdata.input_counter+1
		userdata.generatePose_Endoutput = generatePose
		return 'generated_Pose4'		 
	else :
      		return 'aborted'	
        
	
# define state MoveAction
class MovePoseAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['move_pose'], 
                             output_keys=[''])

    
    def execute(self, userdata):
        start_time = rospy.Time.now()
        self.ac = actionlib.SimpleActionClient('move_jaco_arm', MoveArmAction)
        self.ac.wait_for_server()

        # fill the goal pose
       

        goalpose = userdata.move_pose

	rospy.loginfo("Position 1 %f %f %f", userdata.move_pose.position.x, userdata.move_pose.position.y, userdata.move_pose.position.z)
        moveArm_goal = MoveArmGoal()
	

        moveArm_goal.motion_plan_request.group_name = "jaco_arm"
        moveArm_goal.motion_plan_request.num_planning_attempts = 15
        moveArm_goal.motion_plan_request.planner_id = ""
        moveArm_goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        moveArm_goal.motion_plan_request.allowed_planning_time = rospy.Duration(100.0)
        
        desired_pose = SimplePoseConstraint()

        desired_pose.header.frame_id = "jaco_base_link";		
        desired_pose.link_name = "jaco_link_6";
        
        
        # desired pose get value my userdata
        desired_pose.pose = goalpose   	
        
        
        desired_pose.absolute_position_tolerance.x = 0.03;
        desired_pose.absolute_position_tolerance.y = 0.03;
        desired_pose.absolute_position_tolerance.z = 0.03;
        
        desired_pose.absolute_roll_tolerance = 0.5;  # 5 degree
        desired_pose.absolute_pitch_tolerance = 0.5;
        desired_pose.absolute_yaw_tolerance = 0.5;
        
        addGoalConstraintToMoveArmGoal(desired_pose,moveArm_goal);
        
        send_position[0] = desired_pose.pose.position.x 
        send_position[1] = desired_pose.pose.position.y 
        send_position[2] = desired_pose.pose.position.z 
        send_orientation[0] = desired_pose.pose.orientation.x 
        send_orientation[1] = desired_pose.pose.orientation.y
        send_orientation[2] = desired_pose.pose.orientation.z
        send_orientation[3] = desired_pose.pose.orientation.w
        
        rospy.loginfo("Position 1 %f %f %f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z)

        
        self.ac.send_goal_and_wait(goal=moveArm_goal, execute_timeout=rospy.Duration(10))
        
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

# define state MoveAction
class MoveFingerAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=['set_finger'], 
                             output_keys=[''])
    
    def execute(self, userdata):
        start_time = rospy.Time.now()
        self.ac = actionlib.SimpleActionClient('finger_action', FingerMovementAction)
        self.ac.wait_for_server()


        # fill the goal pose       
        moveFinger_goal = FingerMovementGoal()
	
        moveFinger_goal.task = userdata.set_finger	
                
        self.ac.send_goal_and_wait(goal=moveFinger_goal, execute_timeout=rospy.Duration(10))
        
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

# define state Forward Kinematics
class GetFkService(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted', 'preempted'], 
                             input_keys=[''], output_keys=['resFk'])
    

    def execute(self, userdata):

        start_time = rospy.Time.now()
	self.ac = ServiceState('jaco_arm_kinematics/get_fk', GetPositionFK)
 	rospy.wait_for_service('jaco_arm_kinematics/get_fk')

	#try:
	getFK = rospy.ServiceProxy('jaco_arm_kinematics/get_fk', GetPositionFK)
	fk_request = GetPositionFKRequest()
	joint_names = ['jaco_joint_1' , 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6']
	fk_request.header.frame_id="jaco_base_link"
	fk_request.fk_link_names = [GetPositionFKRequest() for ii in range(1)]
	fk_request.fk_link_names[0]="jaco_link_6"

	fk_request.robot_state.joint_state.position = [GetPositionFKRequest() for j in range(6)]
	fk_request.robot_state.joint_state.name= [GetPositionFKRequest() for jj in range(6)]
	for i in range(len(joint_names)):
		fk_request.robot_state.joint_state.name[i] = joint_names[i]
	for i in range(6):
		fk_request.robot_state.joint_state.position[i] = jtag[i]	
		#rospy.loginfo(rospy.get_name()+"I heard %f",	jtag[i])

	actFK = getFK(fk_request)
	if(actFK.error_code.val == actFK.error_code.SUCCESS):
		rospy.loginfo(rospy.get_name()+"I heard  actual fk %f %f %f", actFK.pose_stamped[0].pose.position.x, actFK.pose_stamped[0].pose.position.y, actFK.pose_stamped[0].pose.position.z)
					
		goalFk = Pose()
		goalFk.position.x 	= actFK.pose_stamped[0].pose.position.x
		goalFk.position.y  	= actFK.pose_stamped[0].pose.position.y
		goalFk.position.z 	= actFK.pose_stamped[0].pose.position.z
		goalFk.orientation.x	= actFK.pose_stamped[0].pose.orientation.x
		goalFk.orientation.y    = actFK.pose_stamped[0].pose.orientation.y
		goalFk.orientation.z    = actFK.pose_stamped[0].pose.orientation.z
		goalFk.orientation.w    = actFK.pose_stamped[0].pose.orientation.w

		
		userdata.resFk = goalFk
		
		return 'succeeded'
	else:
		return 'aborted'

def main():
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['pickPlace_succeeded','pickPlace_aborted'])

	sm.userdata.setFingerCLOSE = "CLOSE"
	sm.userdata.setFingerOPEN = "OPEN"
	sm.userdata.counter = 0

# [1338382621.593038933]: Position: 0.493104,-0.164955,-0.303781
# [1338382621.593099885]: Orientation in quaterion: 0.725867 0.131365 -0.048703 0.673416
	sm.userdata.jacoStartPose= Pose()   
	sm.userdata.jacoStartPose.position.x = 0.380104
	sm.userdata.jacoStartPose.position.y = -0.564955
	sm.userdata.jacoStartPose.position.z = -0.025781 #-0.303781
	sm.userdata.jacoStartPose.orientation.x = 0.725867
	sm.userdata.jacoStartPose.orientation.y = 0.131365
	sm.userdata.jacoStartPose.orientation.z = -0.048703
	sm.userdata.jacoStartPose.orientation.w = 0.673416

	sm.userdata.jacoEndPose= Pose()   
	sm.userdata.jacoEndPose.position.x = 0.293104
	sm.userdata.jacoEndPose.position.y = -0.664955
	sm.userdata.jacoEndPose.position.z = 0.053781 #-0.303781
	sm.userdata.jacoEndPose.orientation.x = 0.725867
	sm.userdata.jacoEndPose.orientation.y = 0.131365
	sm.userdata.jacoEndPose.orientation.z =-0.048703
	sm.userdata.jacoEndPose.orientation.w = 0.673416

 
	sm.set_initial_state(['MoveStartPosition'])
	
	# Open the container
	with sm:	

		
   
		# Add states to the container

		smach.StateMachine.add('find_fk', GetFkService(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'generatePose'},
							remapping={'resFk':'jacoStartPose'})

		smach.StateMachine.add('MoveStartPosition', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'OpenFingerAction1'},
							remapping={'move_pose':'jacoStartPose'})

		smach.StateMachine.add('OpenFingerAction1', MoveFingerAction(), 
							transitions={'preempted':'pickPlace_aborted','aborted':'pickPlace_aborted','succeeded':'generatePose'},
							remapping={'set_finger':'setFingerOPEN'})

		smach.StateMachine.add('generatePose', GeneratePose(),
							transitions = {'generated_Pose1':'MovePosition1', 'generated_Pose2':'MovePosition2', 'generated_Pose3':'MovePosition3', 									'generated_Pose4':'MovePosition4', 'aborted':'pickPlace_aborted'},
							remapping={'generatePose_Startinput':'jacoStartPose','generatePose_Endinput':'jacoEndPose', 'input_counter':'counter',
								 	'generatePose_Startoutput':'jacoStartPose','generatePose_Endoutput':'jacoEndPose','output_counter':'counter'})

		smach.StateMachine.add('MovePosition1', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'CloseFingerAction'},
							remapping={'move_pose':'jacoStartPose'})

		smach.StateMachine.add('CloseFingerAction', MoveFingerAction(), 
							  transitions={'preempted':'pickPlace_aborted','aborted':'pickPlace_aborted','succeeded':'generatePose'},
							    remapping={'set_finger':'setFingerCLOSE'})

		smach.StateMachine.add('MovePosition2', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'MoveEndPosition'},
							remapping={'move_pose':'jacoStartPose'})
        	
		smach.StateMachine.add('MoveEndPosition', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'generatePose'},
							remapping={'move_pose':'jacoEndPose'})

		smach.StateMachine.add('MovePosition3', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'OpenFingerAction'},
							remapping={'move_pose':'jacoEndPose'})

   
		smach.StateMachine.add('OpenFingerAction', MoveFingerAction(), 
							transitions={'preempted':'pickPlace_aborted','aborted':'pickPlace_aborted','succeeded':'generatePose'},
							remapping={'set_finger':'setFingerOPEN'})


		smach.StateMachine.add('MovePosition4', MovePoseAction(),
							transitions = {'preempted':'pickPlace_aborted', 'aborted':'pickPlace_aborted', 'succeeded':'pickPlace_succeeded'},
							remapping={'move_pose':'jacoEndPose'})
        
		
		# Create and start the introspection server
		sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
		sis.start()


	    	# Execute SMACH plan
	    	outcome = sm.execute()

		# Wait for ctrl-c to stop the application
		rospy.spin()
		sis.stop()


if __name__ == '__main__':
	rospy.init_node('pick_place_state_machine',anonymous=False, log_level=rospy.INFO, disable_signals=True)	
	sub = rospy.Subscriber("joint_states",JointState, jtangCallback)	
	sleep(2.5)

	main()

