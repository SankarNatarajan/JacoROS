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
 *  FILE --- jaco.cpp                                                    
 *                                                                       
 *  PURPOSE --- Source file for C++ Wrapper for Kinova's jaco arm        
 */

#include <jaco/jaco.h>

namespace kinova
{
	Jaco::Jaco(const char* dll, const char* API_password) : kinova::AbstractJaco()
	{
		jaco_domain = mono_jit_init_version ("C++Wrapper","v2.0.50727");
		jaco_assembly = mono_domain_assembly_open (jaco_domain, dll);
		
		if (!jaco_assembly)
        		std::cout << dll <<" assembly NOT found!" << std::endl;
		else
         	        std::cout << dll <<" assembly found!" << std::endl;

		//Initialize an image for each DLL that we will use in the program.
		jaco_image = mono_assembly_get_image (jaco_assembly);
	
		//Initialize every classes we need to use.
	        jaco_class = mono_class_from_name (jaco_image, "CSharpWrapper", "MyJacoArm");

                if (!jaco_class)
                {
                        std::cout << "Cannot create JacoArm class!" << std::endl;
                }

         	//We declare an object for every instance of each classes we need to use.
         	jaco_classobject = mono_object_new (jaco_domain, jaco_class);
	
		if (!jaco_classobject)
         	{
                	std::cout << "Cannot create JacoArm instance object!" << std::endl;
         	}

		MonoMethod* tempMethod = 0;
         	void* iterator = 0;


        	//We loop all method from CSharpWrapper to get those we need.
         	//In this case, the constructor and the methods.
		while ((tempMethod = mono_class_get_methods(jaco_class, &iterator)))
         	{
                        //std::cout << "Method: " << mono_method_get_name(tempMethod) << std::endl;
                        //Checking whether the below function is in charpwrapper
                        if (strcmp(mono_method_get_name(tempMethod), ".ctor") == 0)
                                JacoConstructor = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoCheckAPIEnabled") == 0)
                                CheckAPI  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoGetState") == 0)
                                GetState  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetJointAngles") == 0)
                                SetJointAngles  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetAbsPose") == 0)
                                SetAbsPose  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetRelPosition") == 0)
                                SetRelPosition  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoAddJointSpaceTrajectory") == 0)
                                AddJointSpaceTrajectory  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetJointSpaceTrajectory") == 0)
                                SetJointSpaceTrajectory  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoAddCartesianSpaceTrajectory") == 0)
                                AddCartesianSpaceTrajectory  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetCartesianSpaceTrajectory") == 0)
                                SetCartesianSpaceTrajectory  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoEraseTrajectories") == 0)
                                EraseTrajectories  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoOpenFingers") == 0)
                                OpenFingers  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoCloseFingers") == 0)
                                CloseFingers  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoAddFingerPosition") == 0)
                                AddFingerPosition  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetFingersPosition") == 0)
                                SetFingersPosition  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoStartAPICtrl") == 0)
                                StartAPI  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoStopAPICtrl") == 0)
                                StopAPI  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetAngularMode") == 0)
                                SetAngularMode  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetCartesianMode") == 0)
                                SetCartesianMode  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoSetActuatorPIDGain") == 0)
                                SetActuatorPIDGain  = tempMethod;
                        else if (strcmp(mono_method_get_name(tempMethod), "JacoFactoryRestore") == 0)
                                RestoreFactorySetting  = tempMethod;
			
		}
	
                if (!JacoConstructor)
                        std::cout << "Cannot find JacoArm constructor!" << std::endl;
                if (!CheckAPI)
                        std::cout << "Cannot find method CheckAPI!" << std::endl;
                if (!GetState)
                        std::cout << "Cannot find method JacoGetState!" << std::endl;
                if (!SetJointAngles)
                        std::cout << "Cannot find method JacoSetJointAngles!" << std::endl;
                if (!SetAbsPose)
                        std::cout << "Cannot find method JacoSetAbsPose!" << std::endl;
                if (!SetRelPosition)
                        std::cout << "Cannot find method JacoSetRelPosition!" << std::endl;
                if (!AddJointSpaceTrajectory)
                        std::cout << "Cannot find method JacoAddJointSpaceTrajectory!" << std::endl;
                if (!SetJointSpaceTrajectory)
                        std::cout << "Cannot find method JacoSetJointSpaceTrajectory!" << std::endl;
                if (!AddCartesianSpaceTrajectory)
                        std::cout << "Cannot find method JacoAddCartesianSpaceTrajectory!" << std::endl;
                if (!SetCartesianSpaceTrajectory)
                        std::cout << "Cannot find method JacoSetCartesianSpaceTrajectory!" << std::endl;
                if (!EraseTrajectories)
                        std::cout << "Cannot find method JacoEraseTrajectories!" << std::endl;
                if (!OpenFingers)
                        std::cout << "Cannot find method JacoOpenFingers!" << std::endl;
                if (!CloseFingers)
                        std::cout << "Cannot find method JacoCloseFingers!" << std::endl;
                if (!AddFingerPosition)
                        std::cout << "Cannot find method JacoAddFingerPosition" << std::endl;
                if (!SetFingersPosition)
                        std::cout << "Cannot find method JacoSetFingersPosition!" << std::endl;
                if (!StartAPI)
                        std::cout << "Cannot find method JacoStartAPICtrl!" << std::endl;
                if (!StopAPI)
                        std::cout << "Cannot find method JacoStopAPICtrl!" << std::endl;
                if (!SetAngularMode)
                        std::cout << "Cannot find method JacoSetAngularMode!" << std::endl;
                if (!SetCartesianMode)
                        std::cout << "Cannot find method JacoSetCartesianMode!" << std::endl;
                if (!SetActuatorPIDGain)
                        std::cout << "Cannot find method JacoSetActuatorPIDGain!" << std::endl;
                if (!RestoreFactorySetting)
                        std::cout << "Cannot find method JacoFactoryRestore!" << std::endl;



 
         	void* args[1];
         	args[0] = mono_string_new(jaco_domain, API_password);
 
                mono_runtime_invoke(JacoConstructor, jaco_classobject, args, &jaco_exc);

		if (jaco_exc != NULL)	
		{
			std::cout<< "!!!!!!!  Error while calling the C#wrapper constructor" <<std::endl;
			exit(1);
		}	
		
	
	}

	Jaco::Jaco(){}

	Jaco::~Jaco()
	{
		mono_jit_cleanup(jaco_domain);
	}

        bool Jaco::checkApiInitialised()
        {
                bool apistate = false;
                MonoObject *jacoapistate_obj = mono_runtime_invoke(CheckAPI, jaco_classobject, NULL, NULL);
                apistate = *((MonoBoolean*)mono_object_unbox(jacoapistate_obj));

                return apistate;
        }
	
	void Jaco::readJacoStatus()
	{	

		jaco_exc = NULL;
		
            	MonoObject *jacostate_obj = mono_runtime_invoke(GetState, jaco_classobject, NULL, &jaco_exc);
            	jacostate = *((JacoArmState*)mono_object_unbox(jacostate_obj));

		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper readJacoStatus" <<std::endl;
                	exit(1);
			//return false;
                }
		
		// joint angles		
		for(int i = 0; i< 6; i++)
                {
			joint_angles_.at(i) = jacostate.joints[i].angle;
                        joints_current_.at(i) = jacostate.joints_current[i];
                }


		for(int j = 0; j< 3; j++)
		{
                        fingers_jointangle_.at(j)       = jacostate.fingers[j].angle;		// finger joint angles
                        fingers_current_.at(j)          = jacostate.fingers_current[j];		// finger current
		}
		
			
                // pose
		// !!!  CAUTION !!!!
		// Jaco arm API is calculating the forward kinematics not fast enough or its not
		// calculating at all...
		//
		// todo: need to replace by jaco_arm_kinematics FK

		pose_.at(0) = jacostate.hand_position[0];		
		pose_.at(1) = jacostate.hand_position[1];
		pose_.at(2) = jacostate.hand_position[2];
		pose_.at(3) = jacostate.hand_orientation[0];
		pose_.at(4) = jacostate.hand_orientation[1];
		pose_.at(5) = jacostate.hand_orientation[2];

		// current number of trajectory
		trajnum_ = jacostate.current_trajectory;	
		
	}

	void Jaco::readJointStatus()
	{	

		//getJointAngles(ja);		
            	
		// mapping the read value from robot to "normal" configuration

		// joint angles
		joint_angles_.at(0) = ja[0];		
		joint_angles_.at(1) = ja[1];
		joint_angles_.at(2) = ja[2];
		joint_angles_.at(3) = ja[3];
		joint_angles_.at(4) = ja[4];
		joint_angles_.at(5) = ja[5]; 
				
	}
	
	void Jaco::setJointAngles(double jointangles[])
	{	

		setAngularMode();
		
		jaco_exc = NULL;
		
		for(int i = 0; i< 6; i++)					
			set_params[i] = &jointangles[i];


		mono_runtime_invoke(SetJointAngles, jaco_classobject, set_params, &jaco_exc);


		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper setJointangle" <<std::endl;
                	//return false;
                }
		//else
			//return true;	
	}

	

	bool Jaco::setAbsPose(double pose[])
	{
		if(!setCartesianMode())
			return false;

		jaco_exc = NULL;
		
		for(int i = 0; i< 6; i++)					
			set_params[i] = &pose[i];

	
		mono_runtime_invoke(SetAbsPose, jaco_classobject, set_params, &jaco_exc);
				
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper setAbsPose" <<std::endl;
                	return false;
                }
		else
			return true;

	}

	bool Jaco::setRelPosition(double position[])
	{
		if(!setCartesianMode())
			return false;

		jaco_exc = NULL;
		
		for(int i = 0; i< 3; i++)					
			set_position[i] = &position[i];
		
		mono_runtime_invoke(SetRelPosition, jaco_classobject, set_position, &jaco_exc);
				
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper setRelPosition" <<std::endl;
                	return false;
                }
		else
			return true;

	}

	bool Jaco::setJointSpaceTrajectory(std::vector<double> jointtrajectory)
	{	

		if(!setAngularMode())
			return false;

		// erasing any previous trajectory
		if(!eraseTrajectories())
			return false;
		
		if ((jointtrajectory.size() % 6) != 0)
		{
			std::cout<< "!!!!!!!  Trajectory value is woring" <<std::endl;
			return false;
		}
		else
		{
			int number_trajectory = jointtrajectory.size() / 6;

			std::cerr<< "num of tra ="<<number_trajectory <<std::endl;			

			for(int i = 0; i< number_trajectory; i++)
			{
				for(int j = 0; j< 6; j++)					
					set_params[j] = &jointtrajectory.at(j+(i*6));


				mono_runtime_invoke(AddJointSpaceTrajectory, jaco_classobject, set_params, &jaco_exc);


				if (jaco_exc != NULL)	
				{
					std::cout<< "!!!!!!!  Error while calling the C#wrapper AddJointSpaceTrajectory" <<std::endl;
					return false;
				}				
				
			}

			mono_runtime_invoke(SetJointSpaceTrajectory, jaco_classobject, NULL, &jaco_exc);


			if (jaco_exc != NULL)	
			{
				std::cout<< "!!!!!!!  Error while calling the C#wrapper SetJointSpaceTrajectory" <<std::endl;
				return false;
			}
			return true;			
			
		}
	}

	bool Jaco::setCartesianSpaceTrajectory(jaco::JacoPoseTrajectory cartesiantrajectory)
	{	
		if(!setCartesianMode())
			return false;
		
		jaco_exc = NULL;
		std::cerr<< "num of tra ="<<cartesiantrajectory.points.size() <<std::endl;
		
		for(int i = 0; i< cartesiantrajectory.points.size(); i++)
		{
			set_params[0] = &cartesiantrajectory.points.at(i).position.x;
			set_params[1] = &cartesiantrajectory.points.at(i).position.y;
			set_params[2] = &cartesiantrajectory.points.at(i).position.z;
			set_params[3] = &cartesiantrajectory.points.at(i).orientation.x;
			set_params[4] = &cartesiantrajectory.points.at(i).orientation.y;
			set_params[5] = &cartesiantrajectory.points.at(i).orientation.z;

			mono_runtime_invoke(AddCartesianSpaceTrajectory, jaco_classobject, set_params, &jaco_exc);


			if (jaco_exc != NULL)	
			{
				std::cout<< "!!!!!!!  Error while calling the C#wrapper AddCartesianSpaceTrajectory" <<std::endl;
				return false;
			}				
			
		}

		mono_runtime_invoke(SetCartesianSpaceTrajectory, jaco_classobject, NULL, &jaco_exc);


		if (jaco_exc != NULL)	
		{
			std::cout<< "!!!!!!!  Error while calling the C#wrapper SetCartesianSpaceTrajectory" <<std::endl;
			return false;
		}
		return true;			
		
	}

	bool Jaco::eraseTrajectories()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(EraseTrajectories, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper eraseTrajectories" <<std::endl;
                	return false;
                }
		else
			return true;

	}
		


	
	bool Jaco::openFingers()
	{
		jaco_exc = NULL;

		if(!setAngularMode())
			return false;

		// erasing any previous trajectory
                if(!eraseTrajectories())
                        return false;
		
                mono_runtime_invoke(OpenFingers, jaco_classobject, NULL, &jaco_exc);
		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper openFingers" <<std::endl;
                	return false;
                }
		else
			return true;

	}

	bool Jaco::closeFingers()
	{
		jaco_exc = NULL;

		if(!setAngularMode())
			return false;

		// erasing any previous trajectory
                if(!eraseTrajectories())
                        return false;

                mono_runtime_invoke(CloseFingers, jaco_classobject, NULL, &jaco_exc);

		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper closeFingers" <<std::endl;
                	return false;
                }
		else
			return true;

	}


        bool Jaco::setFingersValues(double fingers[])
        {

                jaco_exc = NULL;

                if(!setAngularMode())
                        return false;

                // erasing any previous trajectory
                if(!eraseTrajectories())
                        return false;

                for(int i = 0; i< 3; i++)
                        set_fingers_params[i] = &fingers[i];

                mono_runtime_invoke(AddFingerPosition, jaco_classobject, set_fingers_params, &jaco_exc);


                if (jaco_exc != NULL)
                {
                        std::cout<< "!!!!!!!  Error while calling the C#wrapper AddFingerPosition" <<std::endl;
                        return false;
                }

                mono_runtime_invoke(SetFingersPosition, jaco_classobject, NULL, &jaco_exc);

                if (jaco_exc != NULL)
                {
                        std::cout<< "!!!!!!!  Error while calling the C#wrapper fingersvalues" <<std::endl;
                        return false;
                }
                else
                        return true;

        }

	bool Jaco::startApiCtrl()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(StartAPI, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper startApiCtrl" <<std::endl;
                	return false;
                }
		else
			return true;

	}

	bool Jaco::setAngularMode()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(SetAngularMode, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper SetAngularMode" <<std::endl;
                	return false;
                }
		else
			return true;

	}

	bool Jaco::setCartesianMode()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(SetCartesianMode, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper SetCartesianMode" <<std::endl;
                	return false;
                }
		else
			return true;

	}


	bool Jaco::stopApiCtrl()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(StopAPI, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
	               	std::cout<< "!!!!!!!  Error while calling the C#wrapper stopApiCtrl" <<std::endl;
                	return false;
                }
		else
			return true;
	}

        bool Jaco::setActuatorPIDGain(int jointnum, float P, float I, float D)
        {
                jaco_exc = NULL;

                set_pid_gain[0] = &jointnum;
                set_pid_gain[1] = &P;
                set_pid_gain[2] = &I;
                set_pid_gain[3] = &D;

                mono_runtime_invoke(SetActuatorPIDGain, jaco_classobject, set_pid_gain, &jaco_exc);

                if (jaco_exc != NULL)
                {
                        std::cout<< "!!!!!!!  Error while calling the C#wrapper setActuatorPIDGain" <<std::endl;
                        return false;
                }
                else
                        return true;
        }



	bool Jaco::restoreFactorySetting()
	{
		jaco_exc = NULL;
		
		mono_runtime_invoke(RestoreFactorySetting, jaco_classobject, NULL, &jaco_exc);		
		
		if (jaco_exc != NULL)	
                {
	               	std::cout<< "!!!!!!!  Error while calling the C#wrapper restoreFactorySetting" <<std::endl;
                	return false;
                }
		else
			return true;
	}

}




/*	
	!!!!! This method works, but everytime u call u need to declare a array and fill the stuff  !!!!
	std::vector<double> Jaco::getJointAngles()
	{	
		std::vector<double> joint_angles(6,0.0);
		jaco_exc = NULL;
		
		jarray_act_jtang = (MonoArray*) mono_runtime_invoke(GetJointAngles, jaco_classobject, NULL, &jaco_exc);

		if(jarray_act_jtang)
		{
			for(int i = 0; i< 6; i++)
				ja[i] = (double *)mono_array_addr (jarray_act_jtang, double, i);

			// Mapping the read value from robot to "normal" configuration
			joint_angles.at(0) =  (*ja[0]);
			joint_angles.at(1) =  (*ja[1]) - M_PI;
			joint_angles.at(2) =  M_PI - (*ja[2]);
			joint_angles.at(3) = (*ja[3]);
			joint_angles.at(4) = (*ja[4]);
			joint_angles.at(5) = (*ja[5]);   
     
         	}         
	
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper getjointangle" <<std::endl;
                	exit(1);
                }

		return joint_angles;
	}*/

	/*
	!!!!! For what ever reason, if you used a reference array to get joint angles.. once in a while you get zero joint angles.
	For this reason we are not using this function  !!!!
	void Jaco::getJointAngles(std::vector<double> &jointangles)
	{		
		jaco_exc = NULL;

		mono_runtime_invoke(GetJointAngles, jaco_classobject, get_params, &jaco_exc);
		//jarray_act_jtang = (MonoArray*) mono_runtime_invoke(GetJointAngles, jaco_classobject, NULL, &jaco_exc);

		if(jarray_act_jtang)
		{
			for(int i = 0; i< 6; i++)
			{
				ja[i] = (double *)mono_array_addr (jarray_act_jtang, double, i);
				jointangles.at(i) = *ja[i];
			}
   
     
         	}         
	
		if (jaco_exc != NULL)	
                {
                	std::cout<< "!!!!!!!  Error while calling the C#wrapper getjointangle" <<std::endl;
                	exit(1);
                }
	}*/
	
