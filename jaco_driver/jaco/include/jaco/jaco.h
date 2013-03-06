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
 *  FILE --- jaco.h
 *
 *  PURPOSE --- Header file for C++ Wrapper for Kinova's jaco arm
 */

#ifndef JACO_H_
#define JACO_H_

#include <iostream>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <glib-2.0/glib.h>
#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>
#include <jaco/abstract_jaco.h>
#include <jaco/JacoPoseTrajectory.h>
#include <math.h>

#include <string>

namespace kinova
{
	/// \brief The state of a single Jaco joint.
	struct JacoJointState
	{
		double angle;
		double velocity;	
	};

	/// \brief The state of the arm.	
	/// See the C# wrapper code for fields organization.
	struct JacoArmState
	{
		JacoJointState joints[6];
                double joints_current[6];
		JacoJointState fingers[3];                
		double fingers_current[3];
		double hand_position[3];
		double hand_orientation[3]; // Euler angles XYZ
		int current_trajectory;

	};

	class Jaco : public AbstractJaco
	{   
		/**
		*  This the c++ wrapper class to access the c# dll which in turn a wrapper to access the dlls provided by the manufacture.
		*/
		public:
			/**
			* This constructor allow c++ wrapper class to access the c# dll wrapper
			*
			* @param c# dll wrapper path.
			* @param api password provided by the manufacture.
			*/
                        Jaco(const char* dll, const char* API_password );
                        Jaco();
                        virtual ~Jaco();
                        bool checkApiInitialised();
                        void readJacoStatus();
                        void readJointStatus();
                        void setJointAngles(double jointangles[]);
                        bool setAbsPose(double pose[]);
                        bool setRelPosition(double position[]);
                        bool setJointSpaceTrajectory(std::vector<double> jointtrajectory);
                        bool setCartesianSpaceTrajectory(jaco::JacoPoseTrajectory cartesiantrajectory);
                        bool eraseTrajectories();
                        bool openFingers();
                        bool closeFingers();
                        bool setFingersValues(double fingers[]);
                        bool startApiCtrl();
                        bool stopApiCtrl();
                        bool setAngularMode();
                        bool setCartesianMode();
                        bool setActuatorPIDGain(int jointnum, float P, float I, float D);
                        bool restoreFactorySetting();
		private:
                        /* Variables related to Mono */
                        // Domain that will contains our reference to the DLL
                        MonoDomain *jaco_domain;
                        // Reference to each DLL
                        MonoAssembly *jaco_assembly;
                        // Image for each DLL that will be used in the program
                        MonoImage *jaco_image;
                        // Class in the DLL
                        MonoClass *jaco_class;
                        // Object of the class in the DLL
                        MonoObject *jaco_classobject;
                        // MonoObject for the function
                        MonoObject *jaco_exc;
                        // MonoObject for the function get state
                        //MonoObject *jacostate_obj;
                        // Monoarray to store the manipulator actual joint angles
                        //MonoArray *jarray_act_jtang;
                        // Monoarray to store the manipulator desired joint angles
                        //MonoArray *jarray_des_jtang;
                        // Constructor of the DLL
                        MonoMethod *JacoConstructor;
                        // Check API control of the arm - DLL
                        MonoMethod *CheckAPI;
                        // Get the arm state  - DLL
                        MonoMethod *GetState;
                        // Set the joint angles of the arm - DLL
                        MonoMethod *SetJointAngles;
                        // Set the Pose-Absolute of the arm - DLL
                        MonoMethod *SetAbsPose;
                        // Set the Position-Relative of the arm - DLL
                        MonoMethod *SetRelPosition;
                        // Add the Trajectory in Joint space - DLL
                        MonoMethod *AddJointSpaceTrajectory;
                        // Set the Trajectory in Joint space - DLL
                        MonoMethod *SetJointSpaceTrajectory;
                        // Add the Trajectory in Cartesian space - DLL
                        MonoMethod *AddCartesianSpaceTrajectory;
                        // Set the Trajectory in Cartesian space - DLL
                        MonoMethod *SetCartesianSpaceTrajectory;
                        // Erase Trajectories
                        MonoMethod *EraseTrajectories;
                        // Open Fingers
                        MonoMethod *OpenFingers;
                        // Close Fingers
                        MonoMethod *CloseFingers;
                        // Add the Figner Position
                        MonoMethod *AddFingerPosition;
                        // set Fingers Joint Angles
                        MonoMethod *SetFingersPosition;
                        // Start API control of the arm - DLL
                        MonoMethod *StartAPI;
                        // Stop API control of the arm - DLL
                        MonoMethod *StopAPI;
                        // Set angular mode of the arm - DLL
                        MonoMethod *SetAngularMode;
                        // Set cartesian mode of the arm - DLL
                        MonoMethod *SetCartesianMode;
                        // Set actuator PID gain - DLL
                        MonoMethod *SetActuatorPIDGain;
                        // Restore factory setting of the arm - DLL
                        MonoMethod *RestoreFactorySetting;
		public:
			double ja[6];
			double *tja[6];
			void *get_params[6];
			void *set_params[6];
			void *set_position[3];
                        void *set_pid_gain[4];
                        void *set_fingers_params[3];

	                //boost::recursive_mutex jaco_mutex;	
			JacoArmState jacostate;
		
					
	};
};
#endif
