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
 *  FILE --- jaco_node.cpp
 *
 *  PURPOSE ---  This is the node that start all publishers/services/actions related to the Jaco arm
 */

#include <jaco/jaco_node.h>
#define DTR 0.0174532925

namespace kinova
{
	
	JacoNode::JacoNode(char *CSharpDLL_path)
	{
		
		ros::NodeHandle pn("~");
		
		jaco.reset(new Jaco(CSharpDLL_path, "C6H12O6h2so4")); 

                apistate = jaco->checkApiInitialised();

                if(apistate)
                {
                        jaco->startApiCtrl();
                        //jaco->setActuatorPIDGain(1, 1.0, 0.5, 0.0);
                        //jaco->setActuatorPIDGain(2, 1.0, 0.5, 0.0);
                        //jaco->setActuatorPIDGain(3, 0.7, 0.2, 0.0);
                }
                else
                        std::cout<< "Error : Jaconode intialising failed"<<std::endl;
		
	}
	

	JacoNode::~JacoNode()
	{
                if(apistate)
                {
                        jaco->restoreFactorySetting();
                        jaco->stopApiCtrl();
                }
	}	
	
	int JacoNode::loop()
	{
                ros::Rate loop_rate(100);
		JacoJointPublisher jacoJointPublisher(jaco);	
		JacoActionController jacoActionController(jaco);
                		
		while (ros::ok())
	  	{
			jaco -> readJacoStatus();
			jacoJointPublisher.update();
			jacoActionController.update();
					
			ros::spinOnce();
	    		loop_rate.sleep();
	  	}
	  	return 0;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "jaco");

	if (argc == 2)
	{
		kinova::JacoNode jaco_node(argv[1]);	
	        
                if(jaco_node.apistate)
                        jaco_node.loop();
		
	}
	else
		std::cout<< "Error : Jaconode need C# dll path as a argument"<<std::endl;

  	return 0;
}
