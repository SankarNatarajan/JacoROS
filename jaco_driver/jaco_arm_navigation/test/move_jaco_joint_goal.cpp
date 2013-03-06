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
 *  FILE --- move_jaco_joint_goal_fk.cpp
 *
 *  PURPOSE --- Move the arm in joint space relatively. 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

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
	ros::init (argc, argv, "move_jaco_joint_goal");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5); // 95Hz
	ros::Subscriber	sub_jointangle;

	sub_jointangle		= nh.subscribe<sensor_msgs::JointState>	("joint_states",10,setJointCB);

	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_jaco_arm",true);

	for(int i = 0; i< 5; i++)
	{
		loop_rate.sleep();
		ros::spinOnce();

	}

	move_arm.waitForServer();
	ROS_INFO("Connected to server");

	arm_navigation_msgs::MoveArmGoal goalB;
	std::vector<std::string> names(6);
	std::vector<float>       jtclient_jtangles;

		
	jtclient_jtangles.resize(6);

	names[0] = "jaco_joint_1";
	names[1] = "jaco_joint_2";
	names[2] = "jaco_joint_3";
	names[3] = "jaco_joint_4";
	names[4] = "jaco_joint_5";
	names[5] = "jaco_joint_6";

	for (size_t i = 0; i < 6; i++)
		jtclient_jtangles[i] = joints[i];

	std::cout<<"Please select which joint to move relatively: "<<std::endl;
	std::cout<<"Please type the following for desired joint: "<<std::endl;
	std::cout<<" 'J1' for joint 1"<<std::endl;
	std::cout<<" 'J2' for joint 2"<<std::endl;
	std::cout<<" 'J3' for joint 3"<<std::endl;
	std::cout<<" 'J4' for joint 4"<<std::endl;
	std::cout<<" 'J5' for joint 5"<<std::endl;
	std::cout<<" 'J6' for joint 6"<<std::endl;


	float des_value = 0.0;
	std::string des_joint;

	std::cout<<"Please enter desired joint : ";
	std::cin>> des_joint;

	std::cout<<"Please enter desired relative value for "<<des_joint<<" in degrees: ";	
	std::cin>>des_value;

	std::cerr<<"Joint value before moving "<<std::endl;
	for (size_t i = 0; i < 6; i++)
		std::cout<<jtclient_jtangles[i]*RTD<<std::endl;
	std::cerr<<" ---------------------------"<<std::endl;

	if (des_joint == "J1")
	{
		std::cout<< " changing the joint 1"<<std::endl;
		jtclient_jtangles[0] = joints[0] + (des_value*DTR);
	}
	else if(des_joint ==  "J2")
	{
		std::cout<< " changing the joint 2"<<std::endl;
		jtclient_jtangles[1] = joints[1] + (des_value*DTR);
	}
	else if(des_joint == "J3")
	{
		std::cout<< " changing the joint 3"<<std::endl;
		jtclient_jtangles[2] = joints[2] + (des_value*DTR);
	}
	else if(des_joint == "J4")
	{
		std::cout<< " changing the joint 4"<<std::endl;
		jtclient_jtangles[3] = joints[3] + (des_value*DTR);
	}
	else if(des_joint == "J5")
	{
		std::cout<< " changing the joint 5"<<std::endl;
		jtclient_jtangles[4] = joints[4] + (des_value*DTR);
	}
		

	std::cerr<<"Joint value after changing "<<std::endl;
	for (size_t i = 0; i < 6; i++)
		std::cout<<jtclient_jtangles[i]*RTD<<std::endl;
	std::cerr<<" ---------------------------"<<std::endl;

	goalB.motion_plan_request.group_name = "jaco_arm";
	goalB.motion_plan_request.num_planning_attempts = 1;
	goalB.motion_plan_request.allowed_planning_time = ros::Duration(50.0);

	goalB.motion_plan_request.planner_id= std::string("");
	goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

	for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
	{
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.02;
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.02;
	}

	for(int i = 0; i< 6; i++)
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = jtclient_jtangles.at(i);	
    
	//goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = 0.02;
	//goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = 0.0;
	//goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = 0.0;
   
	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalB);
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
