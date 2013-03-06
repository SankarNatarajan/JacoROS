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
 *  FILE --- jaco_joint_publisher.cpp
 *
 *  PURPOSE ---  Read the value from jaco arm and publish the values
 */

#include <jaco/jaco_joint_publisher.h>

namespace kinova
{
        JacoJointPublisher::JacoJointPublisher(boost::shared_ptr<AbstractJaco> jaco) : jaco(jaco)
        {
                ros::NodeHandle nh;
                jtang_pub = nh.advertise<sensor_msgs::JointState>   ("joint_states", 100);                

                jointNames.resize(NUM_JOINTS);
                fingers_jointName.resize(NUM_FINGER_JOINTS);
                jointangles.resize(NUM_JOINTS);
                joints_current.resize(NUM_JOINTS);
                fingers_jointangle.resize(NUM_FINGER_JOINTS);
                fingers_current.resize(NUM_FINGER_JOINTS);

                for(size_t i = 0; i < NUM_JOINTS ; i++)
                {
                        jointangles.at(i) 		= 0.0;
                        joints_current.at(i) 		= 0.0;
                        jointNames.at(i) 		= "";
                }

                for(size_t i = 0; i < NUM_FINGER_JOINTS; i++)
                {
                        fingers_jointangle.at(i)	= 0.0;
                        fingers_current.at(i) 		= 0.0;
                        fingers_jointName.at(i) 	= "";
                }

        }

        JacoJointPublisher::~JacoJointPublisher()
        {
        }

	void JacoJointPublisher::update()
	{
		// publish joint angles 
		sensor_msgs::JointStatePtr jtang_msg = boost::make_shared<sensor_msgs::JointState>();                
		
		jointNames 		= jaco -> getJointNames();
		jointangles 		= jaco -> getJointAngles();
                joints_current          = jaco -> getJointsCurrent();
                fingers_jointName 	= jaco -> getFingersJointName();
                fingers_jointangle 	= jaco -> getFingersJointAngle();
                fingers_current 	= jaco -> getFingersCurrent();
		
		
	  	for (size_t i = 0; i < NUM_JOINTS; i++)
	  	{	
			jtang_msg->name.push_back(jointNames[i]);    		
	    		jtang_msg->position.push_back(jointangles[i]);	
                        jtang_msg->effort.push_back(joints_current[i]);   	
	  	}

		for (size_t i = 0; i < NUM_FINGER_JOINTS; i++)
	  	{	
                        jtang_msg->name.push_back(fingers_jointName[i]);
                        jtang_msg->position.push_back(fingers_jointangle[i]);
                        jtang_msg->effort.push_back(fingers_current[i]);
	  	}
	
		
	  	jtang_msg -> header.stamp = ros::Time::now();	                

                jtang_pub.publish (jtang_msg);

	}
}
