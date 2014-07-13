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
	qDebug() << __FILE__ << __FUNCTION__ << "entering update";
	
	if((road.isBlocked == true) or (road.isFinished() == true ) or (road.requiresReplanning== true) or (road.isLost == true))
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Robot blocked,  target reached or road lost";
		stopTheRobot(differentialrobot_proxy);
		return false;
	}
		
		///ELIMINADO MOEMENTANME
		
// 	if (road.distanceToLastVisible < 400 /*and road.distanceToLastVisible < road.distanceToTarget*/ ) 
// 	{
// 		qDebug() << __FILE__ << __FUNCTION__<< "Robot stopped to avoid collision because distance to last point visible is" << road.distanceToLastVisible << "less than 400";
// 		road.requiresReplanning = true;
// 		stopTheRobot(differentialrobot_proxy);
// 		return false;
// 	}
	
	if ( time.elapsed() > delay*1000 )   //ojo desbordamientos
	{
		const float MAX_ADV_SPEED = 400;
		const float MAX_ROT_SPEED = 0.7;
		
	
		// VRot is computed as the sum of three terms: angle with tangent to road + atan(perp. distance to road) + road curvature
		// as descirbed in Thrun's paper on DARPA challenge
		
		vrot = road.getAngleWithTangentAtClosestPoint() + atan( road.getRobotPerpendicularDistanceToRoad()/350.) + 0.8 * road.getRoadCurvatureAtClosestPoint() ;
		//vrot = road.getAngleWithTangentAtClosestPoint() + atan( road.getRobotPerpendicularDistanceToRoad()/350. * 0.8);
	
		// Limiting filter
 		if( vrot > MAX_ROT_SPEED ) 
 			vrot = MAX_ROT_SPEED;
 		if( vrot < -MAX_ROT_SPEED ) 
 			vrot = -MAX_ROT_SPEED;
		
		
		// Factors to be used in speed control when approaching the end of the road
		float teta;
		if( road.getRobotDistanceToTarget() < 500)
			teta = exponentialFunction(1./road.getRobotDistanceToTarget(),1./200,0.7, 0.1);
		else
			teta= 1;
		
		//VAdv is computed as a reduction of MAX_ADV_SPEED by two exponential functions: road curvature and VRot, and teta that applies when arriving at target
 		vadvance = MAX_ADV_SPEED * exp(-fabs(2.1* road.getRoadCurvatureAtClosestPoint())) * exp(-fabs(vrot*1.3)) * teta;
		//vadvance = MAX_ADV_SPEED * teta;
		
 		// Limiting filter
 		if( vadvance > MAX_ADV_SPEED ) 
 			vadvance = MAX_ADV_SPEED;
 		
 		qDebug() << "Controller::update - VAdv = " << vadvance << " VRot = " << vrot << "teta" << teta << "atan term" << atan( road.getRobotPerpendicularDistanceToRoad() )*0.2;
 
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
	//qDebug() << landa << value << value/landa << exp(-fabs(value)/landa);
	float res = exp(-fabs(value)/landa);
	if( res < min )
		return min;
	else
		return res;
}