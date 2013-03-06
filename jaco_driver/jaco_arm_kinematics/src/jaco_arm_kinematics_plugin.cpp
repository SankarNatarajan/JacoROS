/*************************************************************************/
/*  Jaco kinematics plugin for Jaco manipulator                          */
/*                                                                       */
/* FILE --- jaco_kinematics_plugin.cpp                                   */
/*                                                                       */
/* PURPOSE --- Source file for jaco kinematics plugin 			 */
/*             for Kinova's jaco arm      				 */
/*             Based on katana_kinematics_plugin.cpp and 		 */ 
/*                      pr2_arm_kinematics_plugin.cpp by Sachin Chitta   */
/*								         */
/*  Sankaranarayanan Natarajan                                           */
/*  sankar.natarajan@dfki.de                                             */
/*  DFKI - BREMEN 2011                                                   */
/*************************************************************************/

#include <jaco_arm_kinematics/jaco_arm_kinematics_plugin.h>
#include <pluginlib/class_list_macros.h>


using namespace tf;
using namespace kinematics;
using namespace std;
using namespace ros;

// register JacoArmKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(jaco_arm_kinematics, JacoArmKinematicsPlugin, jaco_arm_kinematics::JacoArmKinematicsPlugin, kinematics::KinematicsBase)


namespace jaco_arm_kinematics
{
	JacoArmKinematicsPlugin::JacoArmKinematicsPlugin() : active_(false)
	{}

	bool JacoArmKinematicsPlugin::isActive()
	{
		if (active_)
			return true;
		return false;
	}

	bool JacoArmKinematicsPlugin::initialize(const std::string& group_name,
					  const std::string& base_name,
					  const std::string& tip_name,
					  const double& search_discretization)
	{		
		urdf::Model robot_model;
		std::string tip_name_ ;//= tip_name;
		std::string xml_string;
		ros::NodeHandle private_handle("~/" + group_name);

		dimension_ = 6;

		while (!arm_kinematics_constraint_aware::loadRobotModel(private_handle, robot_model, root_name_, tip_name_, xml_string) && private_handle.ok())		
		{
			ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
			ros::Duration(0.5).sleep();
		}
		
		kinematics_msgs::KinematicSolverInfo kinematic_info;

		if (!arm_kinematics_constraint_aware::getChainInfoFromRobotModel(robot_model, root_name_, tip_name_, kinematic_info))
		{
			ROS_FATAL("Could not get chain info!");
		}


		std::cout<<"group_name = "<<group_name<<std::endl;
		std::cout<<"base_name = "<<base_name<<std::endl;
		std::cout<<"tip_name = "<<tip_name<<std::endl;
		std::cout<<"root_name = "<<root_name_<<std::endl;
		std::cout<<"tip_name = "<<tip_name_<<std::endl;

		// connecting to services
		std::string fk_service;
		private_handle.param<std::string> ("fk_service", fk_service, "get_fk");
		fk_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);

		std::string fk_info;
		private_handle.param<std::string> ("fk_info", fk_info, "get_fk_solver_info");
		fk_solver_info_service_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (fk_info);

		std::string ik_service;
		private_handle.param<std::string> ("ik_service", ik_service, "get_ik");
		ik_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (ik_service);


		fk_solver_info_ = kinematic_info;
		ik_solver_info_ = fk_solver_info_;

		for (unsigned int i = 0; i < fk_solver_info_.joint_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin joint name: %s",fk_solver_info_.joint_names[i].c_str());

		}
		for (unsigned int i = 0; i < ik_solver_info_.link_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin can solve IK for %s",ik_solver_info_.link_names[i].c_str());
		}
		for (unsigned int i = 0; i < fk_solver_info_.link_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin can solve FK for %s",fk_solver_info_.link_names[i].c_str());
		}
		ROS_INFO("JacoArmKinematics Plugin::active for %s",group_name.c_str());
		active_ = true;


		ROS_DEBUG("Initializing the JacoArmKinematicsPlugin was successful.");		

		return active_;
	}
	bool JacoArmKinematicsPlugin::initialize(std::string name)

	{		
		urdf::Model robot_model;
		std::string tip_name, xml_string;
		ros::NodeHandle private_handle("~/" + name);

		dimension_ = 6;

		while (!arm_kinematics_constraint_aware::loadRobotModel(private_handle, robot_model, root_name_, tip_name, xml_string) && private_handle.ok())
		{
			ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
			ros::Duration(0.5).sleep();
		}
		
		kinematics_msgs::KinematicSolverInfo kinematic_info;

		if (!arm_kinematics_constraint_aware::getChainInfoFromRobotModel(robot_model, root_name_, tip_name, kinematic_info))
		{
			ROS_FATAL("Could not get chain info!");
		}

		// connecting to services
		std::string fk_service;
		private_handle.param<std::string> ("fk_service", fk_service, "get_fk");
		fk_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);

		std::string fk_info;
		private_handle.param<std::string> ("fk_info", fk_info, "get_fk_solver_info");
		fk_solver_info_service_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (fk_info);

		std::string ik_service;
		private_handle.param<std::string> ("ik_service", ik_service, "get_ik");
		ik_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (ik_service);

		fk_solver_info_ = kinematic_info;
		ik_solver_info_ = fk_solver_info_;

		for (unsigned int i = 0; i < fk_solver_info_.joint_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin joint name: %s",fk_solver_info_.joint_names[i].c_str());

		}
		for (unsigned int i = 0; i < ik_solver_info_.link_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin can solve IK for %s",ik_solver_info_.link_names[i].c_str());
		}
		for (unsigned int i = 0; i < fk_solver_info_.link_names.size(); i++)
		{
			ROS_INFO("JacoArmKinematics Plugin can solve FK for %s",fk_solver_info_.link_names[i].c_str());
		}
		ROS_INFO("JacoArmKinematics Plugin::active for %s",name.c_str());
		active_ = true;
		

		ROS_DEBUG("Initializing the JacoArmKinematicsPlugin was successful.");		

		return active_;
	}

	bool JacoArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
		                                   const std::vector<double> &ik_seed_state, std::vector<double> &solution,
		                                   int &error_code)
	{
		if (!active_)
		{
			ROS_ERROR("kinematics not active");
			error_code = kinematics::SUCCESS;
			return false;
		}

 		ROS_DEBUG("Call getPositionIK()...");
		kinematics_msgs::GetPositionIK srv;

		srv.request.ik_request.pose_stamped.pose 		= ik_pose;
		srv.request.ik_request.pose_stamped.header.frame_id 	= root_name_;
		srv.request.ik_request.pose_stamped.header.stamp 	= ros::Time::now();
		ik_service_.call(srv);

		if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
		{
			solution.resize(dimension_);
			solution 	= srv.response.solution.joint_state.position;
			error_code 	= kinematics::SUCCESS;
			return true;
		}
		else
		{
			ROS_DEBUG("An IK solution could not be found");
			error_code 	= kinematics::NO_IK_SOLUTION;
			return false;
		}
	
	}


	bool JacoArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
		                                      const std::vector<double> &ik_seed_state, const double &timeout,
		                                      std::vector<double> &solution, int &error_code)
	{
		if (!active_)
		{
			ROS_ERROR("kinematics not active");
			error_code = -8; //kinematics::INACTIVE;
			return false;
		}

	  	ROS_DEBUG("Call ik() ");
		return true;		
	
	}

	bool JacoArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
							const std::vector<double> &ik_seed_state,
							const double &timeout,
							std::vector<double> &solution,
							const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution, int &error_code)> &desired_pose_callback,
							const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution, int &error_code)> &solution_callback,
							int &error_code_int)
	{
		if (!active_)
		{
			ROS_ERROR("kinematics not active");
			error_code_int = -8; // kinematics::INACTIVE;
			return false;
		}
		

		return false;
	}

	bool JacoArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				const double &timeout,
				const unsigned int& redundancy,         
				const double &consistency_limit,
				std::vector<double> &solution,
				int &error_code)
	{
		return false;
	}

	bool JacoArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				const double &timeout,
				const unsigned int& redundancy,
				const double &consistency_limit,
				std::vector<double> &solution,
				const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
				const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
				int &error_code)
	{
		return false;
	}

	void JacoArmKinematicsPlugin::desiredPoseCallback(const std::vector<double>& ik_seed_state,
		                                         const geometry_msgs::Pose& ik_pose,
		                                         arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
	{

		int int_error_code;

		desiredPoseCallback_(ik_pose, ik_seed_state, int_error_code);

		if (int_error_code)
			error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
		else
			error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
	}

	void JacoArmKinematicsPlugin::jointSolutionCallback(const std::vector<double>& solution,
		                                           const geometry_msgs::Pose& ik_pose,
		                                           arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
	{
		int int_error_code;

		solutionCallback_(ik_pose, solution, int_error_code);

		if (int_error_code > 0)
			error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
		else
			error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
	}


	bool JacoArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
		                                   const std::vector<double> &joint_angles,
		                                   std::vector<geometry_msgs::Pose> &poses)
	{
		if (!active_)
		{
			ROS_ERROR("kinematics not active");
			return false;
		}

		ROS_DEBUG("Plugin: Call getPositionFK()...");		

		kinematics_msgs::GetPositionFK srv;
	
		srv.request.header.frame_id 			= root_name_;
		srv.request.fk_link_names 			= link_names;
		srv.request.robot_state.joint_state.name 	= fk_solver_info_.joint_names;
		srv.request.robot_state.joint_state.position 	= joint_angles;

		fk_service_.call(srv);

	  	poses.resize(link_names.size());

		if (srv.response.error_code.val == srv.response.error_code.NO_FK_SOLUTION)
		{
			ROS_INFO("Plugin: Could not find a FK");
			return false;
		}

		if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
		{
			ROS_INFO("Successfully computed FK...");

			for (size_t i = 0; i < poses.size(); i++)
			{
				poses[i] = srv.response.pose_stamped[i].pose;
				ROS_DEBUG("PLUGIN Joint: %s Pose: %f %f %f // %f %f %f %f", link_names[i].c_str(),
				poses[i].position.x,
				poses[i].position.y,
				poses[i].position.z,
				poses[i].orientation.x,
				poses[i].orientation.y,
				poses[i].orientation.z,
				poses[i].orientation.w);
			}

			return true;
		}
		else
		{
			ROS_INFO("Plugin: Could not compute FK");
			return false;
		}
	}



	std::string JacoArmKinematicsPlugin::getBaseFrame()
	{
		if (!active_)
		{
			ROS_ERROR("kinematics not active");
			return std::string("");
		}
		return root_name_;
	}

	std::string JacoArmKinematicsPlugin::getToolFrame()
	{
		if (!active_ || ik_solver_info_.link_names.empty())
		{
			ROS_ERROR("kinematics not active");
			return std::string("");
		}

		return ik_solver_info_.link_names[0];
	}

	const std::vector<std::string>& JacoArmKinematicsPlugin::getJointNames() const
	{
		if (!active_)
		{            
			ROS_ERROR("kinematics not active");
		}
		return ik_solver_info_.joint_names;
	}

	const std::vector<std::string>& JacoArmKinematicsPlugin::getLinkNames() const
	{
		if (!active_)
		{
           
			ROS_ERROR("kinematics not active");
           
		}

		return fk_solver_info_.link_names;
	}

}// namespace



