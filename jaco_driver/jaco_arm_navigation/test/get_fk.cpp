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
 *  FILE --- get_fk.cpp
 *
 *  PURPOSE --- Getting the forward kinematics using KDL 
 */

#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <tf/transform_datatypes.h>
#include <LinearMath/btMatrix3x3.h>

float joints[6];


void setJointCB(const sensor_msgs::JointStateConstPtr & des_jtangles)
{
	
	for(int i = 0 ; i < 6; i++)
		joints[i] = des_jtangles->position[i];
	

	//for(int i = 0 ; i < 6; i++) {		
	//	ROS_INFO("  our joint      %d: %f", (int)i,joints[i]);
	//}

      
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "get_fk");
	ros::NodeHandle rh;
	ros::Rate loop_rate(5); // 95Hz
	ros::Subscriber	sub_jointangle;

	sub_jointangle		= rh.subscribe<sensor_msgs::JointState>	("joint_states",10,setJointCB);

	for(int i = 0; i< 5; i++)
	{
		loop_rate.sleep();
		ros::spinOnce();

	}

	ros::service::waitForService("jaco_arm_kinematics/get_fk_solver_info");
	ros::service::waitForService("jaco_arm_kinematics/get_fk");

	ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("jaco_arm_kinematics/get_fk_solver_info");
	ros::ServiceClient fk_client = rh.serviceClient <kinematics_msgs::GetPositionFK>("jaco_arm_kinematics/get_fk");


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
	//fk_request.fk_link_names[0] = "jaco_handpose";
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
				ROS_INFO("Orientation in quaterion: %f %f %f %f",
				fk_response.pose_stamped[i].pose.orientation.x,
				fk_response.pose_stamped[i].pose.orientation.y,
				fk_response.pose_stamped[i].pose.orientation.z,
				fk_response.pose_stamped[i].pose.orientation.w);
			} 

			btQuaternion t(fk_response.pose_stamped[0].pose.orientation.x, fk_response.pose_stamped[0].pose.orientation.y, 
						fk_response.pose_stamped[0].pose.orientation.z, fk_response.pose_stamped[0].pose.orientation.w);

			btScalar roll, pitch,yaw;
			btMatrix3x3 rotmat;
			btMatrix3x3(t).getEulerYPR(yaw, pitch, roll);	
			ROS_INFO("Orientation in euler: %f %f %f ", roll, pitch, yaw);   		
		
			//checking the conversion
			btQuaternion t1;
			t1.setEulerZYX(0,1.57079,0);
			ROS_INFO("Orientation in quaterion from the converted euler: %f %f %f %f ", t1.x(), t1.y(), t1.z(), t1.w());  

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

	ros::shutdown();
}
