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
 *  FILE --- get_solver_info.cpp
 *
 *  PURPOSE --- Getting solver info 
 */
#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "get_kinematic_solver_info");
	ros::NodeHandle rh;
	ros::service::waitForService("jaco_arm_kinematics/get_fk_solver_info");
	ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("jaco_arm_kinematics/get_fk_solver_info");

	// define the service messages
	kinematics_msgs::GetKinematicSolverInfo::Request request;
	kinematics_msgs::GetKinematicSolverInfo::Response response;

	if(query_client.call(request,response))
	{
		for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
		{
			ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
		}
	}
	else
	{
		ROS_ERROR("Could not call query service");
	}
	ros::shutdown();
}

