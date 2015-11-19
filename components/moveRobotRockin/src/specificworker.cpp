/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    //published
    pubROS = nh.advertise<std_msgs::String> (nh.resolveName("testPublication"),1);
    // --> If you look this. write in the terminal  ------>> rostopic echo /testPublication
    
    // subcribe
    subROS = nh.subscribe("chatter", 1000, &SpecificWorker::chatterCallback, this);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}

    // Only publish with the next lines:
    str->data = "If you are seeing this...";
    pubROS.publish(str);
    ROS_INFO_STREAM("hello!!!");
    ros::spinOnce();
    /////////////////

	

	for(auto target : targetList)
	{
		bool goOk = false;
		bool errorGo = false;
		int cont = 0;
		while (!goOk && !errorGo)
		{
			try
			{
				trajectoryrobot2d_proxy->go(target);
				printf("Go to ( %f , %f , %f )",target.x,target.y,target.z);
				goOk = true;
			}
			catch(const Ice::Exception &ex)
			{
				std::cout <<"ERROR trajectoryrobot2d->go "<< ex << std::endl;
				this->sleep(5000)
				cont++;
				if (cont == 4)
				{
					errorGo = true;
				}
			}
			if (!errorGo)
			{
				this->sleep(5000)
			}
		}
		
		if (!errorGo)
		{
			bool stateOk = false;
			bool errorState = false;
			int cont = 0;
			string state;

			while (true) // wait to idle state
			{
				while (!stateOk)
				{
					try
					{
						state = trajectoryrobot2d_proxy->getState().state;
						stateOk = true;
					}
					catch(const Ice::Exception &ex)
					{
						std::cout <<"ERROR trajectoryrobot2d->getState "<< ex << std::endl;
						this->sleep(5000)
						cont++;
						if (cont == 4)
						{
							errorState = true;
						}
					}
				}
				
				if (!errorState)
				{
					if (state == "IDLE")
					{
						break;
					}
				}
				else
				{
					std::cout <<"ERROR in get state!!!"<< std::endl;
					break;
				}
			}			
		}
	}
}


void SpecificWorker::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	///////////// Aqui tiene que venir las n poses
	
	// asignar la lista de target de ROS a la de robocomp
	// 	target.x = pose2D.x;
	// 	target.z = pose2D.y;
	// 	target.y = 0; // no cambiar! siempre 0
	// 	target.ry = pose2D.teta;
	// añadir la modificacion de nueva lista
	targetList = poses2D;
	
}
