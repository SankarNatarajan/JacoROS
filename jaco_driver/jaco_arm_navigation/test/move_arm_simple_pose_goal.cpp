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
 *  FILE --- move_arm_simple_pose_goal.cpp
 *
 *  PURPOSE --- Move the arm +10cm in X axis and -10cm in Yaxis w.r.t jaco retract pose. 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_arm_pose_goal_test");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_jaco_arm",true);
	move_arm.waitForServer();
	ROS_INFO("Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = "jaco_arm";
	goalA.motion_plan_request.num_planning_attempts = 50;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(100.0);

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "jaco_base_link";
	desired_pose.link_name = "jaco_link_6";
	//
	//
	// this position is +10cm in X axis and -10cm in Yaxis w.r.t jaco retract pose
	//
	//
	desired_pose.pose.position.x = 0.313001;
	desired_pose.pose.position.y = -0.437679;
	desired_pose.pose.position.z = 0.495062;

	desired_pose.pose.orientation.x = 0.301508;
	desired_pose.pose.orientation.y = -0.624439;
	desired_pose.pose.orientation.z = 0.653030;
	desired_pose.pose.orientation.w = -0.304501;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.20;
	desired_pose.absolute_pitch_tolerance = 0.20;
	desired_pose.absolute_yaw_tolerance = 0.20;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalA);
		finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
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
