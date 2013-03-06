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
 *  FILE --- jaco_action_controller.cpp
 *
 *  PURPOSE ---  This file has 3 actionlib
 *                  - To move a trajectory in joint space
 *                  - To open or close the finger
 *                  - To move the jaco in relative cartesian space
 *                  - !!! jaco joints are controlled by P controller, so currently a "brutal/hack" controller is used on
 *                        top of jaco controller to rech a desired value !!!
 */


#include <jaco/jaco_action_controller.h>



namespace kinova
{
        JacoActionController::JacoActionController(boost::shared_ptr<AbstractJaco> jaco) :  jaco_apictrl(jaco), JTAC_jaco(jaco), jt_actionserver(jtacn,"joint_trajectory_action",
                                                    boost::bind(&JacoActionController::joint_goalCB,  this, _1), boost::bind(&JacoActionController::joint_cancelCB, this, _1),false),
                                                    CMAC_jaco(jaco), cm_actionserver(cmacn,"cartesian_action",
                                                    boost::bind(&JacoActionController::cartesian_goalCB,  this, _1), boost::bind(&JacoActionController::cartesian_cancelCB, this, _1),false),
                                                    FAC_jaco(jaco), finger_actionserver(facn,"finger_action",
                                                    boost::bind(&JacoActionController::finger_goalCB,  this, _1), boost::bind(&JacoActionController::finger_cancelCB, this, _1),false),
                                                    has_active_goal(false)
        {
                ros::NodeHandle pn("~");
                joints_name.resize(NUM_JOINTS, "");
                current_jtangles.resize(NUM_JOINTS, 0.0); 
		final_jtangles.resize(NUM_JOINTS, 0.0);               
                nextTraj_jtangles.resize(NUM_JOINTS, 0.0);
                current_pose.resize(NUM_JOINTS, 0.0);
                desired_pose.resize(NUM_JOINTS, 0.0);
                error_jtangles.resize(NUM_JOINTS, 0.0);
                dervErr_jtangles.resize(NUM_JOINTS, 0.0);
                old_dervErr_jtangles.resize(NUM_JOINTS, 0.0);
                old_err_jtangles.resize(NUM_JOINTS, 0.0);
                fingers_name.resize(NUM_FINGER_JOINTS, "");
                current_fingervalues.resize(NUM_FINGER_JOINTS, 0.0);
                fingers_current.resize(NUM_FINGER_JOINTS, 0.0);
            

                finger_action = "";

                joints_name 	= jaco -> getJointNames();
                fingers_name    = jaco -> getFingersJointName();

                stop_jaco       = false;
                // used for joint trajectory action
                move_joint          = false;
                movejoint_done      = false;
                num_jointTrajectory = 0;
                num_activeTrajectory = 0;
                // used for cartesian action
                move_pose           = false;
                movepose_done 		= false;
                // used for finger action
                move_finger         = false;
                movefinger_done     = false;
                finger_open         = false;
                finger_close        = false;
                fingerTrajNumber    = 0;


                pn.param("constraints/goal_time", goal_time_constraint, 0.0);
                // Gets the constraints for each joint.
                for (size_t i = 0; i < joints_name.size(); ++i)
                {
                        std::string ns = std::string("constraints/") + joints_name[i];
                        double g, t;
                        pn.param(ns + "/goal", g, DEFAULT_GOAL_THRESHOLD);
                        pn.param(ns + "/trajectory", t, -1.0);
                        goal_constraints[joints_name.at(i)] = g;
                        trajectory_constraints[joints_name.at(i)] = t;
                }
                pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance, 0.01);


                pub_controller_command 	= jtacn.advertise<trajectory_msgs::JointTrajectory>("command", 1);
                //sub_controller_state   	= jtacn.subscribe("feedback_states", 1, &JacoActionController::controllerStateCB, this);

                // starting all the action server
                jt_actionserver.start();
                cm_actionserver.start();
                finger_actionserver.start();

                // temporary outer control loop
                error_factor = 1;
                control_counter = 0;

                // test thread
                object_grasped = false;
                object_grasped_process = false;
                //boost::thread grasp_thread(&objectgrasped(grasp_jaco));
                //boost::thread grasp_thread(boost::bind(&JacoActionController::objectgrasped,this));
                //boost::thread grasp_thread(&JacoActionController.objectgrasped);


                // feedback
                jtaction_fb.joint_names.resize(6,"");
                jtaction_fb.desired.positions.resize(6,0.0);
                jtaction_fb.actual.positions.resize(6,0.0);
                jtaction_fb.error.positions.resize(6,0.0);

                current_time = 0.0;
                old_time = 0.0;
                time_diff = 0.001;
                derv_counter = 0;           


        }

        JacoActionController::~JacoActionController()
        {
                pub_controller_command.shutdown();
                sub_controller_state.shutdown();
                watchdog_timer.stop();
        }

        void JacoActionController::update()
        {


                if (movejoint_done)
                {
                        current_jtangles = JTAC_jaco->getJointAngles();

                        num_activeTrajectory = JTAC_jaco->getCurrentTrajectoryNumber();
                        
                        if (num_jointTrajectory == num_activeTrajectory)
                        {   
                                for (int j = 0; j < 6; j++)                                
                                        nextTraj_jtangles.at(j) = joint_active_goal.getGoal()->trajectory.points.at((num_jointTrajectory-num_activeTrajectory)).positions.at(j);
                                
                        }
                        else
                        {   
                                for (int j = 0; j < 6; j++)                                
                                        nextTraj_jtangles.at(j) = joint_active_goal.getGoal()->trajectory.points.at((num_jointTrajectory-num_activeTrajectory)-1).positions.at(j);
                                        
                        }

			calculate_error_dervError(current_jtangles, nextTraj_jtangles);

                        // feedback
                        jtaction_fb.header.stamp = ros::Time::now();
                        jtaction_fb.joint_names = joints_name;


                        for(int i = 0; i< 6; i++)
                        {
                                jtaction_fb.desired.positions.at(i) = nextTraj_jtangles.at(i);
                                jtaction_fb.actual.positions.at(i)= current_jtangles.at(i);
                                jtaction_fb.error.positions.at(i) = fabs(error_jtangles.at(i));
                        }

                        joint_active_goal.publishFeedback(jtaction_fb);                     
                       

                        if ( num_activeTrajectory == 0)
                        {
                                
                                if (is_trajectory_finished())
                                {
                                        
                                        jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                                        movejoint_done = false;
                                        joint_active_goal.setSucceeded(jtaction_res);

                                        std::cout<<" Final angles in degree"<<std::endl;
                                        for(int i = 0; i< 6; i++)
                                        std::cout<<current_jtangles.at(i)*RTD<<"  ";
                                        std::cout<<"---------------"<<std::endl;
                                        std::cout<<" Final angles in Radian"<<std::endl;
                                        for(int i = 0; i< 6; i++)
                                        std::cout<<current_jtangles.at(i)<<"  ";
                                                                                
                                        std::cerr<<"!!!!!!!!  finished !!!!!!!!!!!!"<<std::endl;
                                        error_factor = 1;
                                        control_counter = 0;
                                        has_active_goal = false;                                        

                                }
				
				/*
				// uncommented bcoz now we can set PID gain.
                                if (simplecontroller_jointSpace(current_jtangles, final_jtangles))                                
                                {
                                        
                                        jtaction_res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                                        movejoint_done = false;
                                        joint_active_goal.setSucceeded(jtaction_res);

                                        std::cout<<" Final angles in degree"<<std::endl;
                                        for(int i = 0; i< 6; i++)
                                        std::cout<<current_jtangles.at(i)*RTD<<"  ";
                                        std::cout<<"---------------"<<std::endl;
                                        std::cout<<" Final angles in Radian"<<std::endl;
                                        for(int i = 0; i< 6; i++)
                                        std::cout<<current_jtangles.at(i)<<"  ";

                                                                                
                                        std::cerr<<"!!!!!!!!  finished !!!!!!!!!!!!"<<std::endl;
                                        error_factor = 1;
                                        control_counter = 0;
                                        has_active_goal = false;


                                        //stop_jaco = true;

                                        if(control_counter > 10)
                                        {
                                                outerloopcontroller_jointSpace(current_jtangles, final_jtangles, error_factor);
                                                error_factor = error_factor + 0.1;
                                                control_counter = 0;
                                        }

                                }
				*/
                               
                        }
                }

                // reason for putting this code here instead of placing above the movejoint_done is bcoz of traj num
                // i.e. firt i need to send the traj to jaco and i need to update the status and then check for traj num.
                if (move_joint)
                {

                        JTAC_jaco->setJointSpaceTrajectory(desired_jtangles);
                        
                        ROS_INFO("Joint trajectory sent to Jaco arm");

                        old_time = ros::Time::now().toSec();

                        movejoint_done = true;
                        move_joint = false;
                        
                        current_jtangles = JTAC_jaco->getJointAngles();

                        num_activeTrajectory = JTAC_jaco->getCurrentTrajectoryNumber();                      

                        
                        for (int j = 0; j < 6; j++)                        
                                nextTraj_jtangles.at(j) = joint_active_goal.getGoal()->trajectory.points.at(0).positions.at(j);
                                

			calculate_error_dervError(current_jtangles, nextTraj_jtangles);

                        // feedback
                        jtaction_fb.header.stamp = ros::Time::now();
                        jtaction_fb.joint_names = joints_name;
                        
                        for(int i = 0; i< 6; i++)
                        {
                                jtaction_fb.desired.positions.at(i) = nextTraj_jtangles.at(i);
                                jtaction_fb.actual.positions.at(i)= current_jtangles.at(i);
                                jtaction_fb.error.positions.at(i) = fabs(error_jtangles.at(i));
                        }

                        joint_active_goal.publishFeedback(jtaction_fb); 

			/*
			// uncommented bcoz now we can set PID gain.
			for (int i = 0; i < 6; i++)
			{
				final_jtangles.at(i) = gh.getGoal()->trajectory.points.at((num_jointTrajectory -1)).positions.at(i);
			}*/                       
                       
                }

                // pose
                if (move_pose)
                {
                        double ps[6];

                        for ( int i = 0; i < 6; i++)
                        	ps[i] = desired_pose.at(i);

                        ROS_INFO("Sending movement to Jaco arm...");
                        CMAC_jaco->setAbsPose(ps);

                        movepose_done = true;
                        move_pose = false;
                }
                if (movepose_done)
                {
                        
                        if (CMAC_jaco->getCurrentTrajectoryNumber() == 0)
                        {

                                current_pose = CMAC_jaco->getPose();
                                if (is_cartesianSpaceTrajectory_finished(current_pose, desired_pose))
                                {
                                        cmaction_res.error_code = jaco::CartesianMovementResult::SUCCESSFUL;
                                        movepose_done = false;
                                        cartesian_active_goal.setSucceeded(cmaction_res);
                                        std::cerr<<"!!!!!!!!  finished !!!!!!!!!!!!"<<std::endl;
                                        //stop_jaco = true;
                                }
                        }
                }

                // finger
                if (move_finger)
                {                        
                        ROS_INFO("Sending finger trajectory movement to Jaco arm...");

                        if (finger_open == true)
                        {
                                FAC_jaco->openFingers();                                
                        }
                        else if (finger_close == true)
                        {
                                FAC_jaco->closeFingers();
                                
                                // grapsing thread started
                                boost::thread grasp_thread(boost::bind(&JacoActionController::objectgrasped,this));

                        }
                        else
                                ROS_ERROR("something went wrong");

                        movefinger_done = true;
                        move_finger = false;
                        
                        object_grasped_process = false;

                }

                if (movefinger_done)
                {
                        fingerTrajNumber = FAC_jaco->getCurrentTrajectoryNumber();
                        current_fingervalues = FAC_jaco->getFingersJointAngle();
                       
                        if (finger_open == true)
                        {
                                if ( (fingerTrajNumber == 0) && (simplecontroller_finger(current_fingervalues,0.001745329)  ) )
                                {                                        
                                        fingeraction_res.result_code = jaco::FingerMovementResult::GRASPED ;
                                        movefinger_done = false;
                                        finger_active_goal.setSucceeded(fingeraction_res);
                                        finger_open = false;
                                }

                        }
                        else if (finger_close == true)
                        {
                                if ( (fingerTrajNumber == 0) && (object_grasped_process)  ) //&& (simplecontroller_finger(current_fingervalues,0.6981317)  )
                                {
                                        
                                        std::cerr<< std::endl<<" object Grasped result " << object_grasped <<std::endl;
                                        
                                        fingeraction_res.result_code = jaco::FingerMovementResult::GRASPED ;
                                        movefinger_done = false;
                                        finger_active_goal.setSucceeded(fingeraction_res);                                        
                                        std::cerr<<"!!!!!!!!  finished !!!!!!!!!!!!"<<std::endl;
                                        //stop_jaco = true;
                                        finger_close = false;

                                }
                        }
                }


                if (stop_jaco)
                {
                        ROS_INFO(" Stopping the api control of Jaco arm...");
                        //jaco_apictrl->stopApiCtrl();
                        stop_jaco = false;
                        //jaco_apictrl->startApiCtrl();
                }

        }

        void JacoActionController::watchdog(const ros::TimerEvent &e)
        {
                // todo
        }

        void JacoActionController::joint_goalCB(JointGoalHandle gh)
        {

                // Ensures that the joints in the goal match the joints we are commanding.
                ROS_INFO("Received goal: goalCB");
                if (!setsEqual(joints_name, gh.getGoal()->trajectory.joint_names))
                {
                        ROS_ERROR("Joints on incoming goal don't match our joints");
                        gh.setRejected();
                        return;
                }

                // Cancels the currently active goal.
                if (has_active_goal)
                {
                        ROS_DEBUG("Received new goal, canceling current goal");

                        // Marks the current goal as canceled.
                        joint_active_goal.setCanceled();
                        has_active_goal = false;
                }

                gh.setAccepted();
                joint_active_goal = gh;
                has_active_goal = true;

                // Sends the trajectory along to the controller
                ROS_DEBUG("Publishing trajectory");                

                int ct = 0;
                num_jointTrajectory = 0;
                num_jointTrajectory = gh.getGoal()->trajectory.points.size();


                desired_jtangles.resize(num_jointTrajectory * 6);

                std::cerr<<"num_jointTrajectory  "<<num_jointTrajectory<<std::endl;

                for(unsigned int i = 0 ; i < gh.getGoal()->trajectory.points.size(); i++)
                {
                        for (int j = 0; j < 6; j++)
                        {
                                desired_jtangles.at(ct) = gh.getGoal()->trajectory.points.at(i).positions.at(j);
                                ct = ct+1;                                
                        }
                }

                move_joint = true;  
        }

        void JacoActionController::joint_cancelCB(JointGoalHandle gh)
        {
                ROS_DEBUG("Received action cancel request");
                if (joint_active_goal == gh)
                {
                        // Stops the controller.
                        stop_jaco = true;

                        // Marks the current goal as canceled.
                        joint_active_goal.setCanceled();
                        has_active_goal = false;
                }
        }

        void JacoActionController::cartesian_goalCB(CartesianGoalHandle gh)
        {
                // Ensures that the joints in the goal match the joints we are commanding.
                ROS_INFO("Received goal: goalCB");

                if(gh.getGoal()->poseGoal.header.frame_id != "base_jaco")
                {
                        ROS_ERROR("Pose.header on incoming goal don't match our Pose.header");
                        gh.setRejected();
                        return;
                }

                // Cancels the currently active goal.
                if (has_active_goal)
                {
                        ROS_DEBUG("Received new goal, canceling current goal");

                        // Marks the current goal as canceled.
                        cartesian_active_goal.setCanceled();
                        has_active_goal = false;
                }

                gh.setAccepted();
                cartesian_active_goal = gh;
                has_active_goal = true;


                desired_pose.at(0) = gh.getGoal()->poseGoal.position.x;
                desired_pose.at(1) = gh.getGoal()->poseGoal.position.y;
                desired_pose.at(2) = gh.getGoal()->poseGoal.position.z ;
                desired_pose.at(3) = gh.getGoal()->poseGoal.orientation.x ;
                desired_pose.at(4) = gh.getGoal()->poseGoal.orientation.y;
                desired_pose.at(5) = gh.getGoal()->poseGoal.orientation.z;

                move_pose = true;

        }

        void JacoActionController::cartesian_cancelCB(CartesianGoalHandle gh)
        {
                ROS_DEBUG("Received action cancel request");
                if (cartesian_active_goal == gh)
                {
                        // Stops the controller.
                        stop_jaco = true;

                        // Marks the current goal as canceled.
                        cartesian_active_goal.setCanceled();
                        has_active_goal = false;
                }
        }

        void JacoActionController::finger_goalCB(FingerGoalHandle gh)
        {

                ROS_INFO("Received goal: goalCB");               

                // Cancels the currently active goal.
                if (has_active_goal)
                {
                        ROS_DEBUG("Received new goal, canceling current goal");

                        // Marks the current goal as canceled.
                        finger_active_goal.setCanceled();
                        has_active_goal = false;
                }

                gh.setAccepted();
                finger_active_goal = gh;
                has_active_goal = true;

                move_finger = true;
                finger_action = gh.getGoal()->task;

                if (finger_action == jaco::FingerMovementGoal::OPEN)
                        finger_open = true;
                else if (finger_action == jaco::FingerMovementGoal::CLOSE)
                        finger_close = true;
                else
                {
                        ROS_ERROR("incoming goal don't match our figner goal");
                        gh.setRejected();
                        return;
                }
        }

        void JacoActionController::finger_cancelCB(FingerGoalHandle gh)
        {
                ROS_DEBUG("Received action cancel request");
                if (finger_active_goal == gh)
                {
                        // Stops the controller.
                        stop_jaco = true;

                        // Marks the current goal as canceled.
                        finger_active_goal.setCanceled();
                        has_active_goal = false;
                }
        }

        bool JacoActionController::objectgrasped()
        {
                int counter  = 0;
                int counter1 = 0;
                int counter2 = 0;
                int counter3 = 0;
                object_grasped = false;
                std::vector<double> finger_current_areas(3,0.0);
                std::vector<double> finger_current_angle(3,0.0);

                ros::Rate r(50); //20 hz

                while (ros::ok())
                {
                        fingers_current = jaco_apictrl->getFingersCurrent();


                        if (fingers_current.at(0) > 0.2)
                        {
                                finger_current_areas.at(0) = finger_current_areas.at(0) + fingers_current.at(0);
                                counter1++;
                        }

                        if (fingers_current.at(1) > 0.2)
                        {
                                finger_current_areas.at(1) = finger_current_areas.at(1) + fingers_current.at(1);
                                counter2++;
                        }

                        if (fingers_current.at(2) > 0.2)
                        {
                                finger_current_areas.at(2) = finger_current_areas.at(2) + fingers_current.at(2);
                                counter3++;
                        }                       


                        counter = counter + 1;
                        r.sleep();
                        if(counter > 150 ) // 150 =  50hz * 3minutes ->
                        {
                                if ( (counter1 > 10) || (counter2 > 10)|| (counter3 > 10) || (finger_current_areas.at(0) > 10.0) ||  /*(finger_current_areas.at(1) > 6.0) && */
                                (finger_current_areas.at(2) > 10.0) )
                                {
                                         //std::cerr<< "OBJECT GRASPPPPPPPPPPPPED !!!!!!!!!!!!!!!!!!!!!" <<std::endl;
                                        object_grasped = true;
                                 }

                                finger_current_angle  = jaco_apictrl->getFingersJointAngle();

                                //std::cerr<< " Chance of current gripping ="<<(finger_current_areas.at(0) +finger_current_areas.at(1) + finger_current_areas.at(2)) / 135.0<<std::endl;
                                //std::cerr<< " Chance of current gripping ="<<(finger_current_areas.at(0) +finger_current_areas.at(1) + finger_current_areas.at(2)) <<std::endl;
                                //std::cerr<< " Chance of current gripping ="<<fingers_current.at(0) <<"  "<<fingers_current.at(1)<<"  "<<fingers_current.at(2)<<std::endl;

                                break;
                        }
                }

                object_grasped_process = true;

                return object_grasped;

        }



        bool JacoActionController::is_jointSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
        {
                double tol_jtag = 2.0 * DTR; 	// tolerance = 2 degree

                for(int i = 0; i< 6; i++)
                        std::cout<<fabs(currentvalue.at(i) - targetvalue.at(i))<<"  ";
                std::cout<<"---------------"<<std::endl;

                if (	(fabs(currentvalue.at(0) - targetvalue.at(0)) < tol_jtag) &&
                (fabs(currentvalue.at(1) - targetvalue.at(1)) < tol_jtag) &&
                (fabs(currentvalue.at(2) - targetvalue.at(2)) < tol_jtag) &&
                (fabs(currentvalue.at(3) - targetvalue.at(3)) < tol_jtag) &&
                (fabs(currentvalue.at(4) - targetvalue.at(4)) < tol_jtag) &&
                (fabs(currentvalue.at(5) - targetvalue.at(5)) < tol_jtag) )
                        return true;

                return false;

        }


        bool JacoActionController::is_cartesianSpaceTrajectory_finished(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
        {

                double tol_pose = 0.01; 			// tolerance = 0.1 m
                double tol_jtag = 0.1 * DTR; 			// tolerance = 0.5 degree

                if (	( fabs(currentvalue.at(0)   	- targetvalue.at(0) ) < tol_pose) &&
                ( fabs(currentvalue.at(1)    	- targetvalue.at(1) ) < tol_pose) &&
                ( fabs(currentvalue.at(2)    	- targetvalue.at(2) ) < tol_pose) &&
                ( fabs(currentvalue.at(3) 	- targetvalue.at(3) ) < tol_jtag) &&
                ( fabs(currentvalue.at(4) 	- targetvalue.at(4) ) < tol_jtag) &&
                ( fabs(currentvalue.at(5) 	- targetvalue.at(5) ) < tol_jtag) )
                        return true;


                return false;

        }

        void JacoActionController::outerloopcontroller_jointSpace(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue, const double error_factor)
        {

                std::vector<double> new_jtang(6,0.0);

                for(int i = 0; i< 6; i++)
                        new_jtang.at(i) = targetvalue.at(i) + (error_factor *( targetvalue.at(i) - currentvalue.at(i)) );

                std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!  Doing the brutal control !!!!!!!!!!!!!!!!!! "<<std::endl;

                JTAC_jaco->setJointSpaceTrajectory(new_jtang);  
        }

        bool JacoActionController::is_trajectory_finished()
        {

                if (    (( fabs(dervErr_jtangles.at(0))< 0.000001 ) && ( fabs(old_dervErr_jtangles.at(0))< 0.000001 ) ) &&
                        (( fabs(dervErr_jtangles.at(1))< 0.000001 ) && ( fabs(old_dervErr_jtangles.at(1))< 0.000001 ) ) &&
                        (( fabs(dervErr_jtangles.at(2))< 0.000001 ) && ( fabs(old_dervErr_jtangles.at(2))< 0.000001 ) ) )
                {
                        derv_counter = derv_counter+1;
                }
                else
                        derv_counter = 0;


                if(derv_counter > 100)
                {
                        derv_counter = 0;
                        //printf("%2.9f, %2.9f, %2.9f, %2.9f \n",error_jtangles.at(0), old_err_jtangles.at(0), time_diff, dervErr_jtangles.at(0));
                        //printf("%2.9f, %2.9f, %2.9f, %2.9f \n",error_jtangles.at(1), old_err_jtangles.at(1), time_diff, dervErr_jtangles.at(1));
                        //printf("%2.9f, %2.9f, %2.9f, %2.9f \n",error_jtangles.at(2), old_err_jtangles.at(2), time_diff, dervErr_jtangles.at(2));
                        return true;
                }

                return false;

        }

        void JacoActionController::calculate_error_dervError(const std::vector<double> &currentvalue, const std::vector<double> &targetvalue)
        {

                current_time = ros::Time::now().toSec();

                time_diff =  current_time - old_time;


                for(int i = 0; i< 6; i++)
                {
                         error_jtangles.at(i) = targetvalue.at(i) - currentvalue.at(i);
                         dervErr_jtangles.at(i) = (error_jtangles.at(i) - old_err_jtangles.at(i)) / time_diff;
                }


                old_time = current_time;

                //printf("\n %2.9f, %2.9f, %2.9f   ",error_jtangles.at(0)*RTD, time_diff, dervErr_jtangles.at(0));

                for(int i = 0; i < 6; i++)
                {
                        old_err_jtangles.at(i) = error_jtangles.at(i);
                        old_dervErr_jtangles.at(i) = dervErr_jtangles.at(i);
                }

        }

        bool JacoActionController::simplecontroller_finger(const std::vector<double> &currentvalue, const double targetvalue)
        {
                double tol_jtag = 3.0 * DTR; 	// tolerance = 2 degree

		// for what ever reason sometime finger 1 and 2 works fine , not fionger 3
		if (    (fabs(currentvalue.at(0) - targetvalue) < tol_jtag) || (fabs(currentvalue.at(1) - targetvalue) < tol_jtag) 
			|| (fabs(currentvalue.at(2) - targetvalue) < tol_jtag) )
                        return true;
		

                return false;

        }


        bool JacoActionController::setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
        {
                if (a.size() != b.size())
                return false;

                for (size_t i = 0; i < a.size(); ++i)
                {
                        if (count(b.begin(), b.end(), a[i]) != 1)
                                return false;
                }
                for (size_t i = 0; i < b.size(); ++i)
                {
                        if (count(a.begin(), a.end(), b[i]) != 1)
                                return false;
                }

                return true;
        }
        
}

