/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "controller.h"

Controller::Controller(int delay)
{
	time = QTime::currentTime();
	this->delay = delay;
}

Controller::~Controller()
{
}

bool Controller::update(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy,  WayPoints &road)
{	
	
	if((road.isBlocked == true) or (road.finish == true ) or (road.requiresReplanning== true))
	{
		qDebug() << "Controller::update - Robot blocked of target reached";
		stopTheRobot(differentialrobot_proxy);
		return false;
	}
		
	if (road.distanceToLastVisible < 400 and road.distanceToLastVisible < road.distanceToTarget ) 
	{
		qDebug() << "Controller::update - Robot stopped to avoid collision";
		road.requiresReplanning = true;
		stopTheRobot(differentialrobot_proxy);
		return false;
	}
	
	if ( time.elapsed() > delay*1000 and (road.finish==false))   //ojo desbordamientos
	{
		const float MAX_ADV_SPEED = 200;
	
		vrot = -road.angleWithTangent + atan( road.distanceToRoad/350.) + 0.8 * road.roadCurvature ;
	
		//speed control when approaching the end of the road
		float teta;
		if( road.distanceToTarget < 500)
			teta = exponentialFunction(1./road.distanceToTarget,1./200,0.7, 0.1);
		else
			teta= 1;
 		vadvance = MAX_ADV_SPEED * exp(-fabs(2.1* road.roadCurvature)) * exp(-fabs(vrot*1.3)) * teta;
 	
// 		//Limiting filter
 		if( vadvance > MAX_ADV_SPEED ) 
 			vadvance = MAX_ADV_SPEED;
 		
 		//qDebug() << "Controller::update - VAdv = " << vadvance << " VRot = " << vrot;
 		
  		try {	differentialrobot_proxy->setSpeedBase( vadvance, vrot);	} 
  		catch (const Ice::Exception &e) { std::cout << e << std::endl;		}	
	}
	else
		try {	differentialrobot_proxy->setSpeedBase( 0, 0);	} 
		catch (const Ice::Exception &e) { std::cout << e << std::endl;		}	
	
	return false;
		
}

void Controller::stopTheRobot(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy)
{
	try {	differentialrobot_proxy->setSpeedBase( 0.f, 0.f);	} 
	catch (const Ice::Exception &e) { std::cout << e << std::endl;}	
}

float Controller::exponentialFunction(float value, float xValue, float yValue, float min)
{
	Q_ASSERT( yValue>0 );
	
	float landa = -fabs(xValue) / log(yValue);
	qDebug() << landa << value << value/landa << exp(-fabs(value)/landa);
	float res = exp(-fabs(value)/landa);
	if( res < min )
		return min;
	else
		return res;
}