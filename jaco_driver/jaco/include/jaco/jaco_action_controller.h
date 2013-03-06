/*
 * Copyright (c) 2012  DFKI GmbH, Bremen, Germany
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
 *  FILE --- jaco_action_controller.h
 *
 *  PURPOSE ---  This file has 3 actionlib
 *                  - To move a trajectory in joint space
 *                  - To open or close the finger
 *                  - To move the jaco in relative cartesian space
 *                  - !!! jaco joints are controlled by P controller, so currently a "brutal/hack" controller is used on
 *                        top of jaco controller to rech a desired value !!!
 */

#ifndef JACO_ACTION_CONTROLLER_H_
#define JACO_ACTION_CONTROLLER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <jaco/abstract_jaco.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <jaco/FingerMovementAction.h>
#include <jaco/CartesianMovementAction.h>
#include <boost/thread/thread.hpp>


#define DTR 0.0174532925
#define RTD 57.295779513

const double DEFAULT_GOAL_THRESHOLD = 0.01;
namespace kinova
{

	class JacoActionController
	{
		typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
		typedef JTAS::GoalHandle JointGoalHandle;	

		typedef actionlib::ActionServer<jaco::CartesianMovementAction> CMAS;			
		typedef CMAS::GoalHandle CartesianGoalHandle;		

                typedef actionlib::ActionServer<jaco::FingerMovementAction> FAS;
                typedef FAS::GoalHandle FingerGoalHandle;


		public:			
			JacoActionController(boost::shared_ptr<AbstractJaco>);			
			virtual ~JacoActionController();
			bool suitableGoal(const std::vector<std::string> &goalNames);
			bool is_jointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue);
                        bool is_trajectory_finished();
                        void calculate_error_dervError(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue);                        
			bool is_cartesianSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue);
                        bool simplecontroller_finger(const std::vector<double> &currentvalue, const double targetvalue);                        
			void watchdog(const ros::TimerEvent &e);			
			void update();	
			bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b);			
				
		private:
			// joint trajectory actionlib variables
			boost::shared_ptr<kinova::AbstractJaco> JTAC_jaco;
                        control_msgs::FollowJointTrajectoryResult jtaction_res;
                        control_msgs::FollowJointTrajectoryFeedback jtaction_fb;
			ros::NodeHandle jtacn;
                        std::vector<std::string> joints_name; // joint state
                        std::vector<double> current_jtangles, desired_jtangles, final_jtangles, nextTraj_jtangles;
                        std::vector<double> error_jtangles, old_err_jtangles, dervErr_jtangles, old_dervErr_jtangles;
                        double old_time, current_time, time_diff;
			void joint_goalCB(JointGoalHandle gh);
 			void joint_cancelCB(JointGoalHandle gh);			
  			JTAS jt_actionserver;	

			bool move_joint;
			bool movejoint_done;
                        int num_jointTrajectory;
                        int num_activeTrajectory;       //active trajectory in jaco
			std::map<std::string,double> goal_constraints;			
			std::map<std::string,double> trajectory_constraints;
			double goal_time_constraint;
			double stopped_velocity_tolerance;
			ros::Publisher pub_controller_command;
			ros::Subscriber sub_controller_state;
			JointGoalHandle joint_active_goal;

			// cartesian actionlib variables
			boost::shared_ptr<kinova::AbstractJaco> CMAC_jaco;
			jaco::CartesianMovementResult cmaction_res;
			ros::NodeHandle cmacn;
			std::vector<double> current_pose, desired_pose;
			void cartesian_goalCB(CartesianGoalHandle gh);
 			void cartesian_cancelCB(CartesianGoalHandle gh);							
			CMAS cm_actionserver;
						
			bool move_pose;
			bool movepose_done;
			CartesianGoalHandle cartesian_active_goal;	

			// finger actionlib variables
			boost::shared_ptr<kinova::AbstractJaco> FAC_jaco;
			jaco::FingerMovementResult fingeraction_res;                      
                        ros::NodeHandle facn;
                        std::vector<std::string> fingers_name;	// finger state
                        std::vector<double> current_fingervalues;
                        void finger_goalCB(FingerGoalHandle gh);
                        void finger_cancelCB(FingerGoalHandle gh);
                        FAS finger_actionserver;

                        bool move_finger;
                        bool movefinger_done;
                        int fingerTrajNumber;
                        std::string finger_action;
                        bool finger_open, finger_close;
                        FingerGoalHandle finger_active_goal;

                        std::vector<double> fingers_current;
						
			// action lib variables			
			boost::shared_ptr<kinova::AbstractJaco> jaco_apictrl;
			ros::Timer watchdog_timer;					
			bool stop_jaco;			
			bool has_active_goal;	

			// temporary outer control loop			
			void outerloopcontroller_jointSpace(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue, const double error_factor);
			double error_factor;
			int control_counter;

			// graps			
			bool object_grasped_process;
			bool object_grasped;
                        bool objectgrasped();

			int derv_counter;
    

                        
	};	
}
 
#endif // JACO_ACTION_CONTROLLER_H_
