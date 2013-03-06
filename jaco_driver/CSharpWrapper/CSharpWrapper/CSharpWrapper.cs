/*
 * Copyright (c) 2011  DFKI GmbH, Bremen, Germany
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 2 of the License, or (at your
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
 *  FILE --- CSharpWrapper.cs
 *
 *  PURPOSE ---  This is a wrapper to communicate with other dll's provided by kinova. So it will be easier to communicate with
 *               one dll from C++.
 */

using System;
using System.Collections.Generic;
using System.Threading;
using Kinova.API.Jaco;
using Kinova.API.Jaco.Configurations;
using Kinova.API.Jaco.Control.Trajectory;
using Kinova.DLL.SafeGate;
using Kinova.DLL.Data;
using Kinova.DLL.Data.Jaco;
using Kinova.DLL.Data.Jaco.Control;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.Data.Util;
using Kinova.API.Jaco.Diagnostic;
using Kinova.DLL.Data.Jaco.Diagnostic;


namespace CSharpWrapper
{	
		
	static class Constants
	{
		public const double DTR = 0.0174532925;  // Degree to Radian
		public const double RTD = 57.295779513;  // Radian to Degree
	}
	/// \brief A Jaco joint state structure.
    public struct JacoJointState
    {
        public double angle;
        public double velocity;       
    }

  	/// this JacoArmState concept is taken from ros pakage provided by Kinova
    /// NOTE: we're not using arrays for unboxing issues on the native side.
    /// fixed buffers are an option, but implies unsafe sections.
    /// CAUTION: this variable order should be similiar to the struct in jaco.cpp
    public struct JacoArmState
    {
		// joints information
        public JacoJointState shoulder_yaw;
        public JacoJointState shoulder_pitch;
        public JacoJointState elbow_pitch;
        public JacoJointState elbow_roll;
        public JacoJointState wrist_roll;
        public JacoJointState hand_roll;
		public double current_joint_1;
		public double current_joint_2;
		public double current_joint_3;
		public double current_joint_4;
		public double current_joint_5;
		public double current_joint_6;
		
		// finger information
		public JacoJointState finger_1;
		public JacoJointState finger_2;
		public JacoJointState finger_3;			
		public double current_finger_1;
		public double current_finger_2;
		public double current_finger_3;		
		
		// hand information
        public double hand_position_x;
        public double hand_position_y;
        public double hand_position_z;
        public double hand_orientation_x;
        public double hand_orientation_y;
        public double hand_orientation_z;
		
		// trajectory info
		public int current_traj;
		
	
    }
	
	public class MyJacoArm
	{		
		private CJacoArm m_Arm;
		private JacoArmState m_State;
		private CVectorAngle setjointvalue = new CVectorAngle();
		private CVectorEuler setposevalue  = new CVectorEuler();
		private float []addposevalue = new float[6];
		private float []addjointvalue = new float[6];
		private CTrajectoryInfo jointvaluetrajectory = new CTrajectoryInfo();
		private CTrajectoryInfo posevaluetrajectory  = new CTrajectoryInfo();
		private CPointsTrajectory m_JointsTrajectory = new CPointsTrajectory();
		private CPointsTrajectory m_PoseTrajectory = new CPointsTrajectory();
		private CPointsTrajectory m_fingerTrajectory = new CPointsTrajectory();
		private CJoystickValue m_Cmd;
		private CPosition positionLive;	
		//private CPosition positionError = new CPosition();
		private bool m_IsEnabled = false;
		private bool m_IsRetracting;       
        private TimeSpan m_RetractDelay;
		private DateTime m_LastCmd; 
		private const double CMD_EPSILON = 1e-6;
		
		// jaco state
		private CAngularInfo 		joint_info;
		private CAngularInfo 		current_info;
		private CCartesianInfo 		pose_info;
		private CInfoFIFOTrajectory trajectory_info;
		
		
		public MyJacoArm(string key)
        {
            try 
            {
                CCypherMessage pass = Crypto.GetInstance().Encrypt("C6H12O6h2so4");
				
                m_Arm = new CJacoArm(pass);					
				
                System.Console.WriteLine("C# Wrapper initialized.");				
                System.Console.Write("JACO version: ");
                System.Console.WriteLine(m_Arm.ConfigurationsManager.GetCodeVersion()[0].ToString("X1"));
				
				m_RetractDelay = new TimeSpan(0,0,0,24,0); // 8 secs.	
				
				for(int i = 0; i < 6; i++)
				{
					addposevalue[i] = 0.0f;
					addjointvalue[i] = 0.0f;
				}
				
				// erasing trajectories
				m_Arm.ControlManager.EraseTrajectories();
				// deleting the previous error log
				m_Arm.DiagnosticManager.DataManager.DeleteErrorLog();
	
				// restore factory setting
				JacoFactoryRestore();
				// set the arm flag to true 
				m_IsEnabled = true;
								
            } 
			catch (Exception e)
            {
                System.Console.WriteLine("JACO API initialisation failed. Reason: ");
                System.Console.WriteLine(e.ToString());
				// set the arm flag to false 
				m_IsEnabled = false;
            }
        }	
		
		public bool JacoCheckAPIEnabled()
		{
			return m_IsEnabled;
		}
       
		public void JacoStartAPICtrl()
		{
			try
			{
				if (m_Arm.JacoIsReady())
	    		{				
	        		//From now on, API can move the arm.	        		
					m_Arm.ControlManager.StartControlAPI();
								
	    		}
			}
			catch (Exception ex)
			{
    			System.Console.WriteLine("EXCEPTION in JacoStartAPICtrl");
				System.Console.WriteLine(ex.ToString());
			}		
			
		}
		
		public void JacoStopAPICtrl()
		{
			try
			{
				if (m_Arm.JacoIsReady())
				{
					m_Arm.ControlManager.StopControlAPI();
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoStopAPICtrl");
				System.Console.WriteLine(ex.ToString());
			}
			
		}	

		
		public void JacoSetAngularMode()
		{
			try
			{
				if (m_Arm.JacoIsReady())
	    		{				
	        		// Setting the angular mode
					m_Arm.ControlManager.SetAngularControl();									
	    		}
			}
			catch (Exception ex)
			{
    			System.Console.WriteLine("EXCEPTION in JacoSetAngularMode");
				System.Console.WriteLine(ex.ToString());
			}		
		}		
		
		public void JacoSetCartesianMode()
		{
			try
			{
				if (m_Arm.JacoIsReady())
	    		{				
	        		// Setting the angular mode
					m_Arm.ControlManager.SetCartesianControl();
									
	    		}
			}
			catch (Exception ex)
			{
    			System.Console.WriteLine("EXCEPTION in JacoSetCartesianMode");
				System.Console.WriteLine(ex.ToString());
			}		
		}	
		
		// get the status of the arm only if its ready
		public JacoArmState JacoGetState()
		{
			JacoRefreshEncoder();
			return m_State;
		}
		
		public void JacoRefreshEncoder()
		{
			try
			{
					if (m_Arm.JacoIsReady())
    				{				
						joint_info 			= m_Arm.ControlManager.GetPositioningAngularInfo();
						current_info 		= m_Arm.ControlManager.GetCurrentAngularInfo();
						pose_info 			= m_Arm.ControlManager.GetCommandCartesianInfo();
						trajectory_info		= m_Arm.ControlManager.GetInfoFIFOTrajectory();					     
											
											    
						// Based on Observation of actual model	
						// getting joints angles
						// Normailising the joint angle (-180 to +180)
						m_State.shoulder_yaw.angle 		= Normalize( ((joint_info.Joint1) - 180.0) * Constants.DTR );
						m_State.shoulder_pitch.angle 	= Normalize( ((joint_info.Joint2) - 270.0) * Constants.DTR );
						m_State.elbow_pitch.angle 		= Normalize( ((joint_info.Joint3) - 90.0 ) * Constants.DTR );	
						m_State.elbow_roll.angle 		= Normalize( ((joint_info.Joint4) - 180.0) * Constants.DTR );
						m_State.wrist_roll.angle 		= Normalize( ((joint_info.Joint5) - 180.0) * Constants.DTR );
						m_State.hand_roll.angle 		= Normalize( ((joint_info.Joint6) - 260.0) * Constants.DTR );					
					
						// based on DH model provided by Kinova
						/*m_State.shoulder_yaw.angle 		= (180.0 - (joint_info.Joint1)) * Constants.DTR;
						m_State.shoulder_pitch.angle 	= ((joint_info.Joint2) - 270.0) * Constants.DTR;
						m_State.elbow_pitch.angle 		= (90.0 - (joint_info.Joint3))  * Constants.DTR;
						m_State.elbow_roll.angle 		= (180.0 - (joint_info.Joint4)) * Constants.DTR;
						m_State.wrist_roll.angle 		= (180.0 - (joint_info.Joint5)) * Constants.DTR;
						m_State.hand_roll.angle 		= (260.0 - (joint_info.Joint6)) * Constants.DTR;*/
					
						// getting joint current	
						current_info = m_Arm.ControlManager.GetCurrentAngularInfo();				
					
						m_State.current_joint_1 = current_info.Joint1;
						m_State.current_joint_2 = current_info.Joint2;
						m_State.current_joint_3 = current_info.Joint3;
						m_State.current_joint_4 = current_info.Joint4;
						m_State.current_joint_5 = current_info.Joint5;
						m_State.current_joint_6 = current_info.Joint6;
			
					
						// getting fingers angles
						m_State.finger_1.angle = (joint_info.Finger1) * Constants.DTR;
						m_State.finger_2.angle = (joint_info.Finger2) * Constants.DTR;
						m_State.finger_3.angle = (joint_info.Finger3) * Constants.DTR;
					
						// getting finger current					
						m_State.current_finger_1 = current_info.Finger1;
						m_State.current_finger_2 = current_info.Finger2;	
						m_State.current_finger_3 = current_info.Finger3;					
					
						// getting the pose
						// !!!  CAUTION !!!!
						// Jaco arm API is calculating the forward kinematics not fast enough or its not
                		// calculating at all...
						//
						// todo: need to replace by jaco_arm_kinematics FK
						m_State.hand_position_x 	= pose_info.X ;
						m_State.hand_position_y  	= pose_info.Y ;
						m_State.hand_position_z  	= pose_info.Z ;
						m_State.hand_orientation_x 	= pose_info.ThetaX ;
						m_State.hand_orientation_y 	= pose_info.ThetaY ;
						m_State.hand_orientation_z 	= pose_info.ThetaZ ;
											
						// getting the trajectory info
						m_State.current_traj = trajectory_info.StillInFIFO;							
    				}					
					
				}
				catch (Exception ex)
				{
	    			System.Console.WriteLine("EXCEPTION in JacoRefreshEncoder");
					System.Console.WriteLine(ex.ToString());
				}		
				
		}
		
		public double Normalize( double angle )
		{
			//Console.Write("angle_old : " + angle);
			if( angle > Math.PI )
			{
				while( angle > Math.PI )
				{
					angle -= 2*Math.PI;
				}
			}
			else if( angle < -Math.PI )
			{
				while( angle < -Math.PI )
				{
					angle += 2*Math.PI;
				}
			}			
			return angle;	
		}
		
		// The "JacoSetJointAngles" function receive all Joint angles in Radians, But robot need joint angles in degree
		public void JacoSetJointAngles(double j1, double j2, double j3, double j4, double j5, double j6)
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{	
					
					// Based on Observation of actual model
					setjointvalue.Angle[0] = (float)(( j1 * Constants.RTD) + 180.0 );
					setjointvalue.Angle[1] = (float)(( j2 * Constants.RTD) + 270.0 );
					setjointvalue.Angle[2] = (float)(( j3 * Constants.RTD) + 90.0 ) ;
					setjointvalue.Angle[3] = (float)(( j4 * Constants.RTD) + 180.0 );
					setjointvalue.Angle[4] = (float)(( j5 * Constants.RTD) + 180.0 );
					setjointvalue.Angle[5] = (float)(( j6 * Constants.RTD) + 260.0 ) ;
					
					// Based on DH model provided by Kinova
					/*setjointvalue.Angle[0] = (float)(-j1 * Constants.RTD) + 180.0f ;
					setjointvalue.Angle[1] = (float)( j2 * Constants.RTD) + 270.0f ;
					setjointvalue.Angle[2] = (float)(-j3 * Constants.RTD) + 90.0f  ;
					setjointvalue.Angle[3] = (float)(-j4 * Constants.RTD) + 180.0f ;
					setjointvalue.Angle[4] = (float)(-j5 * Constants.RTD) + 180.0f ;
					setjointvalue.Angle[5] = (float)(-j6 * Constants.RTD) + 260.0f ;*/		
					
										
			        jointvaluetrajectory.UserPosition.AnglesJoints = setjointvalue;
			        jointvaluetrajectory.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;				       
			
			        m_JointsTrajectory.Add(jointvaluetrajectory);
			
			        m_Arm.ControlManager.SendTrajectoryFunctionnality(m_JointsTrajectory); 					
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoSetJointAngles");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}
		
	
		
		public void JacoSetAbsPose(double X, double Y, double Z, double Rx, double Ry, double Rz)
		{				  
				try
				{
					if (m_Arm.JacoIsReady())
    				{   						
						setposevalue.Position[0] = (float)(X);
						setposevalue.Position[1] = (float)(Y);
						setposevalue.Position[2] = (float)(Z);
						setposevalue.Rotation[0] = (float)(Rx);
						setposevalue.Rotation[1] = (float)(Ry);
						setposevalue.Rotation[2] = (float)(Rz);
											
						posevaluetrajectory.UserPosition.Position     = setposevalue;
						posevaluetrajectory.UserPosition.PositionType = CJacoStructures.PositionType.CartesianPosition;
			        
						m_PoseTrajectory.Add(posevaluetrajectory);
			
			        	m_Arm.ControlManager.SendTrajectoryFunctionnality(m_PoseTrajectory);  
					
    				}					
				}
				catch (Exception ex)
				{
	    			System.Console.WriteLine("EXCEPTION in JacoSetPose");
					System.Console.WriteLine(ex.ToString());
				}					        	
		}
				
		public void JacoSetRelPosition(double X, double Y, double Z)
		{				
				try
				{
					if (m_Arm.JacoIsReady())
    				{   						
						
            			m_Arm.ControlManager.SetCartesianControl();

			            if (X < -CMD_EPSILON)
			                m_Cmd.InclineLR = -1;
			            else if (X > CMD_EPSILON)
			                m_Cmd.InclineLR = 1;
			            else
			                m_Cmd.InclineLR = 0;
			
			            if (Y < -CMD_EPSILON)
			                m_Cmd.InclineFB = -1;
			            else if (Y > CMD_EPSILON)
			                m_Cmd.InclineFB = 1;
			            else
			                m_Cmd.InclineFB = 0;
			
			            if (Z < -CMD_EPSILON)
			                m_Cmd.Rotate = -1;
			            else if (Z > CMD_EPSILON)
			                m_Cmd.Rotate = 1;
			            else
			                m_Cmd.Rotate = 0;								
						
					
    				}					
				}
				catch (Exception ex)
				{
	    			System.Console.WriteLine("EXCEPTION in JacoSetRelPosition");
					System.Console.WriteLine(ex.ToString());
				}					        	
		}
		
		public void JacoSetActuatorPIDGain(int jointnum, float P, float I, float D)
		{				
				try
				{
					if (m_Arm.JacoIsReady())
    				{   						
						//System.Console.WriteLine("Setting PID value");					
						switch(jointnum)
						{
							case 1:
								m_Arm.ControlManager.SetActuatorPID(16, P, I, D);								
								break;
							case 2:
								m_Arm.ControlManager.SetActuatorPID(17, P, I, D);
								break;
							case 3:
								m_Arm.ControlManager.SetActuatorPID(18, P, I, D);
								break;
							case 4:
								m_Arm.ControlManager.SetActuatorPID(19, P, I, D);
								break;
							case 5:
								m_Arm.ControlManager.SetActuatorPID(20, P, I, D);
								break;
							case 6:
								m_Arm.ControlManager.SetActuatorPID(21, P, I, D);
								break;
							default:
								System.Console.WriteLine("Joint number is wrong");
								break;
						
						}	
					
    				}					
				}
				catch (Exception ex)
				{
	    			System.Console.WriteLine("EXCEPTION in JacoSetPIDGain");
					System.Console.WriteLine(ex.ToString());
				}					        	
		}	
		
		
		public void JacoRetract()
		{			
			try
			{
				if (m_Arm.JacoIsReady())
				{
					
					
					System.Console.WriteLine("Jaco arm API Retract()");
					m_Cmd = new CJoystickValue();
					m_Cmd.ButtonValue[2] = 1;
					
					m_Arm.ControlManager.SendJoystickFunctionality(m_Cmd);					
					
					
					while(m_IsRetracting)
					{
						try
		            	{   
							
							//Getting position Live Only when retracting and wait for 
							positionLive = new CPosition();
							positionLive = m_Arm.DiagnosticManager.DataManager.GetPositionLogLiveFromJaco();
							
							System.Console.WriteLine("Retract status ");
							System.Console.WriteLine(positionLive.SystemStatus.RetractStatus);
							
							
						
							//Are we done retracting 
							if ( positionLive.SystemStatus.RetractStatus == 1 || positionLive.SystemStatus.RetractStatus == 3)
							{								
								m_IsRetracting = false;
								// Stop the retract/reset command.
		               			m_Cmd.ButtonValue[2] = 0;
		                		m_Arm.ControlManager.SendJoystickFunctionality(m_Cmd); 
								m_LastCmd = DateTime.Now;
							}
							
						
							else
							{
								//Look for timeout
								if ((DateTime.Now - m_LastCmd) > m_RetractDelay)
								{
									System.Console.WriteLine("Retract delay expired");
									m_IsRetracting = false;
									// Stop the retract/reset command.
			               			m_Cmd.ButtonValue[2] = 0;
			                		m_Arm.ControlManager.SendJoystickFunctionality(m_Cmd); 
									m_LastCmd = DateTime.Now;
								}
							}						
	                
			            }
			            catch (Exception e)
			            {
			                System.Console.WriteLine("JACO retract diagnostic API Failed : ");
			                System.Console.WriteLine(e.ToString());
							m_IsRetracting = false;
							// Stop the retract/reset command.
		           			m_Cmd.ButtonValue[2] = 0;
		            		m_Arm.ControlManager.SendJoystickFunctionality(m_Cmd); 
							m_LastCmd = DateTime.Now;
			            }						
					}	
										
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in ");
				System.Console.WriteLine(ex.ToString());
			}		 		
		
		}
		
		public void JacoAddJointSpaceTrajectory(double j1, double j2, double j3, double j4, double j5, double j6)
		{	
			try
			{
				if (m_Arm.JacoIsReady())
					
				{					
					
					// Based on Observation of actual model
					addjointvalue[0] = (float)(( j1 * Constants.RTD) + 180.0 );
					addjointvalue[1] = (float)(( j2 * Constants.RTD) + 270.0 );
					addjointvalue[2] = (float)(( j3 * Constants.RTD) + 90.0 ) ;
					addjointvalue[3] = (float)(( j4 * Constants.RTD) + 180.0 );
					addjointvalue[4] = (float)(( j5 * Constants.RTD) + 180.0 );
					addjointvalue[5] = (float)(( j6 * Constants.RTD) + 260.0 ) ;
					
					m_JointsTrajectory.Add(GenerateJointTrajectory(addjointvalue));
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoAddJointSpaceTrajectory");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}
		public void JacoSetJointSpaceTrajectory()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{					
					m_Arm.ControlManager.SendBasicTrajectory(m_JointsTrajectory);
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoSetJointSpaceTrajectory");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}	
		
		private CTrajectoryInfo GenerateJointTrajectory(float []jointAngles)
		{
					CTrajectoryInfo jointTrajectory = new CTrajectoryInfo();  
                  
                    jointTrajectory.LimitationActive = false;
                    jointTrajectory.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;		        	
                    jointTrajectory.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;
									
                    jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_1] = jointAngles[0];
					jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_2] = jointAngles[1];
					jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_3] = jointAngles[2];
					jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_4] = jointAngles[3];
					jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_5] = jointAngles[4];
					jointTrajectory.UserPosition.AnglesJoints.Angle[CVectorAngle.JOINT_6] = jointAngles[5];	
						
					
					jointTrajectory.UserPosition.FingerPosition[0] = joint_info.Finger1;
					jointTrajectory.UserPosition.FingerPosition[1] = joint_info.Finger2;
					jointTrajectory.UserPosition.FingerPosition[2] = joint_info.Finger3;
						
                   
					return jointTrajectory;
		}
		
		public void JacoAddCartesianSpaceTrajectory(double X, double Y, double Z, double Rx, double Ry, double Rz)
		{	
			
				try
				{
					if (m_Arm.JacoIsReady())
    				{   						
						addposevalue[0] = (float)(X);
						addposevalue[1] = (float)(Y);
						addposevalue[2] = (float)(Z);
						addposevalue[3] = (float)(Rx);
						addposevalue[4] = (float)(Ry);
						addposevalue[5] = (float)(Rz);	
									        
						m_PoseTrajectory.Add(GeneratePoseTrajectory(addposevalue));	
											
    				}					
				}
				catch (Exception ex)
				{
	    			System.Console.WriteLine("EXCEPTION in JacoAddCartesianSpaceTrajectory");
					System.Console.WriteLine(ex.ToString());
				}			
			
		}
		public void JacoSetCartesianSpaceTrajectory()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{
			        m_Arm.ControlManager.SendTrajectoryFunctionnality(m_PoseTrajectory);	
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoSetCartesianSpaceTrajectory");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}	

		
		private CTrajectoryInfo GeneratePoseTrajectory(float []pose)
		{
					CTrajectoryInfo poseTrajectory = new CTrajectoryInfo();  
                  
                    poseTrajectory.LimitationActive = false;
                    poseTrajectory.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;
                    poseTrajectory.UserPosition.PositionType = CJacoStructures.PositionType.CartesianPosition;
                    
					poseTrajectory.UserPosition.Position.Position[CVectorEuler.COORDINATE_X] = pose[0];
                    poseTrajectory.UserPosition.Position.Position[CVectorEuler.COORDINATE_Y] = pose[1];
                    poseTrajectory.UserPosition.Position.Position[CVectorEuler.COORDINATE_Z] = pose[2];
                    poseTrajectory.UserPosition.Position.Rotation[CVectorEuler.THETA_X] = pose[3];
                    poseTrajectory.UserPosition.Position.Rotation[CVectorEuler.THETA_Y] = pose[4];
                    poseTrajectory.UserPosition.Position.Rotation[CVectorEuler.THETA_Z] = pose[5];
                   
					return poseTrajectory;
		}
		
		private CTrajectoryInfo GenerateFingerTrajectory(float finger_1, float finger_2, float finger_3)
		{
					CTrajectoryInfo fingerTrajectory = new CTrajectoryInfo();  
                  
                    fingerTrajectory.LimitationActive = false;
                    fingerTrajectory.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;		        	
                    fingerTrajectory.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;
									
                    fingerTrajectory.UserPosition.AnglesJoints = m_Arm.ConfigurationsManager.GetJointPositions();					
					
					fingerTrajectory.UserPosition.FingerPosition[0] = finger_1;
					fingerTrajectory.UserPosition.FingerPosition[1] = finger_2;
					fingerTrajectory.UserPosition.FingerPosition[2] = finger_3;						
                   
					return fingerTrajectory;
		}
		
		public void JacoAddFingerPosition(double finger_1, double finger_2, double finger_3)
		{	
				try
				{
					if (m_Arm.JacoIsReady())					
					{
						m_fingerTrajectory.Add(GenerateFingerTrajectory((float)finger_1, (float)finger_2, (float)finger_3));						
					}
				}
				catch (Exception ex)
				{
					System.Console.WriteLine("EXCEPTION in JacoSetFingerPosition");
					System.Console.WriteLine(ex.ToString());
				} 					
			
		}
		
		public void JacoSetFingersPosition()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{					
			        m_Arm.ControlManager.SendTrajectoryFunctionnality(m_fingerTrajectory);
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoSetJointSpaceTrajectory");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}	
		
		public void JacoOpenFingers()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{		 
					
					JacoEraseTrajectories();
					double []kk= new double [3];
					
					kk[0] = 0.1;
					kk[1] = 0.1;
					kk[2] = 0.1;
					
					JacoAddFingerPosition(kk[0], kk[1], kk[2]);
					JacoSetFingersPosition();
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoOpenFinger");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}
		
		// arbitrary chosen 40.0  
		public void JacoCloseFingers()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{	
					JacoEraseTrajectories();
					
					double []finger_angle= new double [3];
					
					finger_angle[0] = 40.0;
					finger_angle[1] = 40.0;
					finger_angle[2] = 40.0;
					
					JacoAddFingerPosition(finger_angle[0], finger_angle[1], finger_angle[2]);
					JacoSetFingersPosition();	
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoCloseFinger");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}		
		
		
		public void JacoEraseTrajectories()
		{	
			try
			{
				if (m_Arm.JacoIsReady())
				{	
					m_JointsTrajectory.Trajectory.Clear();
					m_fingerTrajectory.Trajectory.Clear ();
					// erasing any previous trajectories
					m_Arm.ControlManager.EraseTrajectories();					
					
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in JacoEraseTrajectories");
				System.Console.WriteLine(ex.ToString());
			} 				
			
		}	
		
		public void JacoFactoryRestore()
		{
			
			try
			{
				if (m_Arm.JacoIsReady())
				{
					System.Console.WriteLine("Restoring Factory default settings ...");
					m_Arm.DiagnosticManager.ToolManager.RestoreFactorySettings();				
				}
			}
			catch (Exception ex)
			{
				System.Console.WriteLine("EXCEPTION in ");
				System.Console.WriteLine(ex.ToString());
			}		 
		
		}	
		
	}
}

/*
!!!!! For what ever reason, if you used a reference array to get joint angles.. once in a while you get zero joint angles.
      For this reason we are not using this function  !!!!
public void JacoGetJointAngles(ref double []jointangles)	     
{				
	try
	{
		if (m_Arm.JacoIsReady())
		{    
			for (int i = 0; i < 6; i++)
			{						
				jointangles[i] = m_Arm.ConfigurationsManager.GetJointPositions().Angle[i];
			}	
			
		}
		else
		{
			for (int i = 0; i < 6; i++)
			{
				jointangles[i] = 0;
			}		
		}
	}
	catch (Exception ex)
	{
		System.Console.WriteLine("EXCEPTION in JacoGetJointAngles");
		System.Console.WriteLine(ex.ToString());
	}	        	
}*/

/*
!!!! My impression is, mono array is not working properly.. sometimes its not passing the array !!!

public void JacoSetJointAngles(double[] desiredjointangle)		
{	
	try
	{
		if (m_Arm.JacoIsReady())
		{	
	
			for(int i = 0; i < 6; i ++)
	        	setjointvalue.Angle[i] = desiredjointangle[i];		
			
			for(int i = 0; i < 6; i++)
				System.Console.WriteLine(desiredjointangle[i]);
			
			System.Console.WriteLine("--------------");    
			
	        jointvaluetrajectory.UserPosition.AnglesJoints = setjointvalue;
	        jointvaluetrajectory.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;
	
	        jointvaluetrajectory.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;
	
	        jointvaluetrajectory.UserPosition.FingerPosition[0] = 10;
	        jointvaluetrajectory.UserPosition.FingerPosition[1] = 30;
	        jointvaluetrajectory.UserPosition.FingerPosition[2] = 40;
	
	        m_PointsTrajectory.Add(jointvaluetrajectory);
	
	        m_Arm.ControlManager.SendTrajectoryFunctionnality(m_PointsTrajectory);  
		}
	}
	catch (Exception ex)
	{
		System.Console.WriteLine("EXCEPTION in JacoSetJointAngles");
		System.Console.WriteLine(ex.ToString());
	} 					
	
}*/
