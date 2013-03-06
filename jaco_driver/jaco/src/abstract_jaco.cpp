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
 *  FILE --- abstract_jaco.cpp
 *
 *  PURPOSE --- Source file for abstract class for Kinova's jaco arm
 */

#include <jaco/abstract_jaco.h>

namespace kinova
{
	AbstractJaco::AbstractJaco()
	{
		// joint names
                joints_name_.resize(NUM_JOINTS);
	  	// finger names
                fingers_jointname_.resize(NUM_FINGER_JOINTS);
		// link names
		link_names_.resize(NUM_JOINTS);
		
	  
		// Joints - angles and velocities and limits
		joint_angles_.resize(NUM_JOINTS, 0.0);
                joints_current_.resize(NUM_JOINTS, 0.0);
                fingers_jointangle_.resize(NUM_FINGER_JOINTS, 0.0);
                fingers_current_.resize(NUM_FINGER_JOINTS, 0.0);
		pose_.resize(6, 0.0);
		trajnum_ = 0;
		joint_velocities_.resize(NUM_JOINTS);
	 
		/* ********* get parameters ********* */	  	
	  	ros::NodeHandle n;	

                joints_name_.at(0) = "jaco_joint_1";
                joints_name_.at(1) = "jaco_joint_2";
                joints_name_.at(2) = "jaco_joint_3";
                joints_name_.at(3) = "jaco_joint_4";
                joints_name_.at(4) = "jaco_joint_5";
                joints_name_.at(5) = "jaco_joint_6";

                fingers_jointname_.at(0) = "jaco_finger_joint_1";
                fingers_jointname_.at(1) = "jaco_finger_joint_2";
                fingers_jointname_.at(2) = "jaco_finger_joint_3";

		link_names_.at(0) = "jaco_link_1";
		link_names_.at(1) = "jaco_link_2";
		link_names_.at(2) = "jaco_link_3";
		link_names_.at(3) = "jaco_link_4";
		link_names_.at(4) = "jaco_link_5";
		link_names_.at(5) = "jaco_link_6";

	  	

	  }

	AbstractJaco::~AbstractJaco()
	{
	}	

	std::vector<double> AbstractJaco::getJointAngles()
	{
		return joint_angles_;
	}
        std::vector<double> AbstractJaco::getJointsCurrent()
        {
                return joints_current_;
        }

	std::vector<double> AbstractJaco::getFingersJointAngle()
	{
                return fingers_jointangle_;
	}
	
	std::vector<double> AbstractJaco::getFingersCurrent()
	{
                return fingers_current_;
	}

	std::vector<double> AbstractJaco::getPose()
	{
		return pose_;
	}

	int AbstractJaco::getCurrentTrajectoryNumber()
	{
		return trajnum_;
	}

	std::vector<std::string> AbstractJaco::getJointNames()
	{
                return joints_name_;
	}

        std::vector<std::string> AbstractJaco::getFingersJointName()
	{
                return fingers_jointname_;
	}


	std::vector<std::string> AbstractJaco::getLinkNames()
	{
	  	return link_names_;
	}	
}

