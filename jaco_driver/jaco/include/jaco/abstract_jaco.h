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
 *  FILE --- abstract_jaco.h
 *
 *  PURPOSE --- Header file for abstract class for Kinova's jaco arm
 */

#ifndef ABSTRACT_JACO_H_
#define ABSTRACT_JACO_H_

#include <vector>

#include <ros/ros.h>
#include <jaco/jaco_constants.h>
#include <jaco/JacoPoseTrajectory.h>


namespace kinova
{
	class AbstractJaco
	{
		public:
			AbstractJaco();
			virtual ~AbstractJaco();
                        virtual bool checkApiInitialised()=0;
			virtual void readJointStatus()=0;
			virtual void readJacoStatus()=0;
			virtual void setJointAngles(double jointangles[])=0;
			virtual bool setJointSpaceTrajectory(std::vector<double> jointtrajectory)=0;
			virtual bool setCartesianSpaceTrajectory(jaco::JacoPoseTrajectory cartesiantrajectory)=0;
			virtual bool setAbsPose(double pose[])=0;
			virtual bool setRelPosition(double position[])=0;
			virtual bool openFingers()=0;
			virtual bool closeFingers()=0;
            		virtual bool setFingersValues(double fingers[])=0;
			virtual bool startApiCtrl()=0;
			virtual bool stopApiCtrl()=0;
			virtual bool setAngularMode()=0;
			virtual bool setCartesianMode()=0;
                        virtual bool setActuatorPIDGain(int jointnum, float P, float I, float D)=0;
                        virtual bool restoreFactorySetting()=0;

                        std::vector<std::string> getJointNames();
                        std::vector<std::string> getFingersJointName();
			std::vector<std::string> getLinkNames();
			std::vector<double> getJointAngles();
                        std::vector<double> getJointsCurrent();
			std::vector<double> getFingersJointAngle();
			std::vector<double> getFingersCurrent();
			std::vector<double> getPose();
			int getCurrentTrajectoryNumber();

	
		protected:
                        std::vector<std::string> joints_name_;
                        std::vector<std::string> fingers_jointname_;
			std::vector<std::string> link_names_;	
			std::vector<double> joint_angles_;
                        std::vector<double> joints_current_;
                        std::vector<double> fingers_jointangle_;
                        std::vector<double> fingers_current_;
			std::vector<double> pose_;
			int trajnum_;
			std::vector<double> joint_velocities_;
	};
}
#endif	       /*ABSTRACTJACO_H_ */
