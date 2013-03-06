/*
 * Copyright (c) 2011  DFKI GmbH, Bremen, Germany
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 *  Author: Sankaranarayanan Natarajan / sankar.natarajan@dfki.de
 *
 *  FILE --- move_jaco_simple_pose_goal_fk.cpp
 *
 *  PURPOSE --- Move the arm -10 cm in Z axis. The current arm joint angles are read through joint state subscriber then
 *		FK is called and w.r.t the FK value the arm is moved relatively. 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <jaco/JacoPoseStamped.h>
#include <tf/transform_datatypes.h>
#include <LinearMath/btMatrix3x3.h>
#include <tf/transform_listener.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <Eigen/Core>
#define DTR 0.0174532925
#define RTD 57.295779513


float joints[6];


void setJointCB(const sensor_msgs::JointStateConstPtr & des_jtangles)
{
	
	for(int i = 0 ; i < 6; i++)
		joints[i] = des_jtangles->position[i];
	

	//for(int i = 0 ; i < 6; i++) {		
	//	ROS_INFO("  our joint      %d: %f", (int)i,joints[i]);
	//}

      
}



int main(int argc, char **argv){
	ros::init (argc, argv, "move_jaco_simple_pose_goal");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5); // 95Hz


	ros::Subscriber	sub_jointangle;

	sub_jointangle		= nh.subscribe<sensor_msgs::JointState>	("joint_states",10,setJointCB);

	for(int i = 0; i< 5; i++)
	{
		loop_rate.sleep();
		ros::spinOnce();

	}

	ros::service::waitForService("jaco_arm_kinematics/get_fk_solver_info");
	ros::service::waitForService("jaco_arm_kinematics/get_fk");
	
	ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("jaco_arm_kinematics/get_fk_solver_info");
	ros::ServiceClient fk_client = nh.serviceClient <kinematics_msgs::GetPositionFK>("jaco_arm_kinematics/get_fk");

	// define the service messages
	kinematics_msgs::GetKinematicSolverInfo::Request request;
	kinematics_msgs::GetKinematicSolverInfo::Response response;

	if(query_client.call(request,response))
	{
		for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
		{
			ROS_DEBUG("Joint: %d %s", i,
			response.kinematic_solver_info.joint_names[i].c_str());
		}
	}
	else
	{
		ROS_ERROR("Could not call query service");
		ros::shutdown();
		exit(1);
	}

	// define the service messages
	kinematics_msgs::GetPositionFK::Request  fk_request;
	kinematics_msgs::GetPositionFK::Response fk_response;
	fk_request.header.frame_id = "jaco_base_link";
	fk_request.fk_link_names.resize(1);
	fk_request.fk_link_names[0] = "jaco_link_6";
	
 	tf::Pose bt;
 
	fk_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
	fk_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;

	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	{
		fk_request.robot_state.joint_state.position[i] = joints[i];
	}


	if(fk_client.call(fk_request, fk_response))
	{
		if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
		{
			tf::poseMsgToTF(fk_response.pose_stamped[0].pose, bt);
			
			for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
			{
				ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
				ROS_INFO_STREAM("Position: " << 
				fk_response.pose_stamped[i].pose.position.x << "," <<  
				fk_response.pose_stamped[i].pose.position.y << "," << 
				fk_response.pose_stamped[i].pose.position.z);
				ROS_INFO("Orientation: %f %f %f %f",
				fk_response.pose_stamped[i].pose.orientation.x,
				fk_response.pose_stamped[i].pose.orientation.y,
				fk_response.pose_stamped[i].pose.orientation.z,
				fk_response.pose_stamped[i].pose.orientation.w);
			} 
		}
		else
		{
			ROS_ERROR("Forward kinematics failed");
		}
	}
	else
	{
		ROS_ERROR("Forward kinematics service call failed");
	}

	ROS_INFO("im here 1");		
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_jaco_arm",true);

	ROS_INFO("im here");
		

	move_arm.waitForServer();
	ROS_INFO("Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "jaco_arm";
	goalA.motion_plan_request.num_planning_attempts = 15;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(100.0);
  
	arm_navigation_msgs::SimplePoseConstraint desired_pose, target_pose;
	desired_pose.header.frame_id = "jaco_base_link";
	desired_pose.link_name = "jaco_link_6";

	

	desired_pose.pose.position = fk_response.pose_stamped[0].pose.position;
	desired_pose.pose.position.z= desired_pose.pose.position.z - 0.1 ;

	desired_pose.pose.orientation = fk_response.pose_stamped[0].pose.orientation;

	std::cout<< desired_pose.pose.position<<std::endl;
	std::cout<< "--------------------"<<std::endl;
	std::cout<< desired_pose.pose.orientation<<std::endl;

	

	desired_pose.absolute_position_tolerance.x = 0.05;
	desired_pose.absolute_position_tolerance.y = 0.05;
	desired_pose.absolute_position_tolerance.z = 0.05;

	desired_pose.absolute_roll_tolerance = 0.87;  // 50 degree
	desired_pose.absolute_pitch_tolerance = 0.87;
	desired_pose.absolute_yaw_tolerance = 0.87;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalA);
		finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
		if (!finished_within_time)
		{
			move_arm.cancelGoal();
			ROS_INFO("Timed out achieving goal A");
		}
		else
		{
			actionlib::SimpleClientGoalState state = move_arm.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success)
				ROS_INFO("Action finished: %s",state.toString().c_str());
			else
				ROS_INFO("Action failed: %s",state.toString().c_str());
		}
	}
	ros::shutdown();
}


