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
 *  FILE --- jaco_node.h
 *
 *  PURPOSE ---  This is the node that start all publishers/services/actions related to the Jaco arm
 */

#ifndef JACO_NODE_H_
#define JACO_NODE_H_

#include <ros/ros.h>
#include <jaco/abstract_jaco.h>
#include <jaco/jaco_joint_publisher.h>
#include <jaco/jaco.h>
#include <jaco/jaco_action_controller.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

//#include <jaco/armpose.h>
namespace kinova
{
	class JacoNode
	{
		public:
			JacoNode(char *CSharpDLL_path);
			virtual ~JacoNode();
                        int loop();
                        bool apistate;  // to check whether jaco api is properly initialised

		private:
			boost::shared_ptr<kinova::AbstractJaco> jaco;


					
	};
}
#endif
