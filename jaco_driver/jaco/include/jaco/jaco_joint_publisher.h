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
 *  FILE --- jaco_joint_publisher.h
 *
 *  PURPOSE ---  Read the value from jaco arm and publish the values
 */

#ifndef JACO_JOINT_PUBLISHER_H_
#define JACO_JOINT_PUBLISHER_H_

#include <vector>
#include <jaco/abstract_jaco.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


namespace kinova
{
	class JacoJointPublisher
	{
		public:
			JacoJointPublisher(boost::shared_ptr<AbstractJaco>);
			virtual ~JacoJointPublisher();
		  	void update();			
		private:
                        std::vector<double> jointangles, joints_current, fingers_jointangle, fingers_current;
			std::vector<double> pose;
			boost::shared_ptr<AbstractJaco> jaco;
                        ros::Publisher jtang_pub;                        
                        std::vector<std::string> jointNames, fingers_jointName;
	};

}

#endif /* JACO_JOINT_PUBLISHER_H_ */
