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
 *  FILE --- jaco_kinematics.h
 *
 *  PURPOSE ---  Header file for jaco arm kinematics. It uses a ik file generated by openrave ikfast.
 *               Based on pr2_arm_kinematics.h by Sachin Chitta and
 *	         Katana ros package by Henning Deeken
 */

/*************************************************************************/
/* Arm Kinematics for Jaco manipulator		                         */
/*                                                                       */
/* FILE --- jaco_arm_kinematics.h	                                 */
/*                                                                       */
/* PURPOSE --- Header file for jaco arm kinematics for Kinova's jaco arm */
/*             Based on pr2_arm_kinematics.h by Sachin Chitta and     	 */
/*	       Katana ros package by Henning Deeken 			 */
/*                                                                       */
/*  Sankaranarayanan Natarajan                                           */
/*  sankar.natarajan@dfki.de                                             */
/*  DFKI - BREMEN 2012                                                   */
/*************************************************************************/

#ifndef JACO_ARM_KINEMATICS_H
#define JACO_ARM_KINEMATICS_H

#include <ros/ros.h>

#include <arm_navigation_msgs/JointLimits.h>
#include <urdf/model.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/model_utils.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>

#include <boost/shared_ptr.hpp>

#include <../src/jaco_openrave_ikfast.cpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <angles/angles.h>


namespace jaco_arm_kinematics
{
	struct redundant_ik_solution
	{
	    std::vector<float> ik_sol;
	    float opt_iksol_index_value;
	};

	struct compare_result
	{
	    bool operator()(redundant_ik_solution const &a, redundant_ik_solution const &b) {
		 return a.opt_iksol_index_value < b.opt_iksol_index_value;
	     }

	};

	class JacoArmKinematics
	{
		public:
		

			/** @class
			*  @brief ROS/Openrave based interface for the inverse kinematics of the Jaco arm
			*
			*  This class provides a ROS/Openrave based interface to the inverse kinematics of the jaco arm
			*
			*  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server.
			*/
			JacoArmKinematics();
		
			virtual ~JacoArmKinematics()
			{
				if (collision_models_interface_)
      					delete collision_models_interface_;
			}

			/**
			*  @brief Specifies if the node is active or not
	     		*  @return True if the node is active, false otherwise.
	     		*/
	    		bool isActive();

	    		/**
	     		* @brief This is the basic IK service method that will compute and return an IK solution.
	     		* @param A request message. See service definition for GetPositionIK for more information on this message.
	     		* @param The response message. See service definition for GetPositionIK for more information on this message.
	     		*/
	    		bool getOpenRaveIK(kinematics_msgs::GetPositionIK::Request &request,
						kinematics_msgs::GetPositionIK::Response &response);

			/**
			* @brief This is the basic forward kinematics service that will return information about the kinematics node.
			* @param A request message. See service definition for GetPositionFK for more information on this message.
			* @param The response message. See service definition for GetPositionFK for more information on this message.
			*/
			bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
		               			kinematics_msgs::GetPositionFK::Response &response);
	    		/**
			* @brief This is the basic kinematics info service that will return information about the kinematics node.
			* @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
			* @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
			*/
			bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
						 kinematics_msgs::GetKinematicSolverInfo::Response &response);

			/**
			* @brief This is the basic kinematics info service that will return information about the kinematics node.
			* @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
			* @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
			*/
			bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
		                 		kinematics_msgs::GetKinematicSolverInfo::Response &response);

			bool get_constraint_aware_ik (kinematics_msgs::GetConstraintAwarePositionIK::Request &request, 
							kinematics_msgs::GetConstraintAwarePositionIK::Response &response);

			/**   
			*@brief This function pick the optimal solution from a vector of IK solution provided by openrave. It consider two main conditions
			*	First it get rid of solution which exceeds joint limits and second pick a solution which near to current joint angle(to avoid unneccasary joint movement)
			*@param input current joint angles
				*@param output optimised ik solution
			*/
			//bool pickOptimalIkSolution(const std::vector<float> &cur_jtang, const std::vector<IKSolution> &redundantSolutions, std::vector<float> &optSol);
			bool pickOptimalIkSolution(const std::vector<float> &cur_jtang, const std::vector<IKSolution> &redundantSolutions, std::vector<std::vector<float> > &optSol);

            		void collisionCheck(const std::vector<IKReal> &ik_solution, int &error_code);

		private:
			float jt_weight[6];
			std::vector<float>current_jointangles;
			int num_joints;
			planning_environment::CollisionModelsInterface *collision_models_interface_;	// used for transformation
			kinematics_msgs::KinematicSolverInfo kinematic_info;                            // kinematic chain info


			planning_models::KinematicState* state_;
			arm_navigation_msgs::Constraints constraints_;

		
		protected:
			bool active_;
			urdf::Model robot_model;
			XmlRpc::XmlRpcValue joint_names;
			std::string robot_desc_string;
			ros::NodeHandle node_handle;
			std::string root_name, tip_name, xml_string;
			kinematics_msgs::KinematicSolverInfo ik_solver_info, fk_solver_info;
			boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver;
			tf::TransformListener tf;
			KDL::Chain kdl_chain;
			btMatrix3x3 rotmat_4_IK;
			ros::ServiceServer ik_service, fk_service, constraint_aware_ik_service, ik_solver_info_service, fk_solver_info_service;
    		
	};
}

#endif


