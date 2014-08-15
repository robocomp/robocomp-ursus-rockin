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

Controller::Controller(int delay)  //in secs
{
	time = QTime::currentTime();
	this->delay = delay*1000;
}

Controller::~Controller()
{
}

bool Controller::update(InnerModel &innerModel, const RoboCompLaser::TLaserData &laserData,
						RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy,  WayPoints &road)
{	
	static QTime reloj = QTime::currentTime();   //TO be used for a more accurate control (predictive). 
	static long epoch = 100;
	
	//Estimate the space that will be blindly covered and reduce Adv speed to remain within some boundaries
	
	//qDebug() << __FILE__ << __FUNCTION__ << "entering update with" << road.at(road.getIndexOfClosestPointToRobot()).pos;
	
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
	
	if ( time.elapsed() > delay )   //Initial wait in secs
	{
		//qDebug() << "epoch" << epoch;
		float MAX_ADV_SPEED = 600.f;
		float MAX_ROT_SPEED = 0.7;
		if( (epoch-100) > 0 )				//Damp max speeds if elapsed time is too long
		{
			MAX_ADV_SPEED = 600 * exponentialFunction(epoch-100, 200, 0.2);
			MAX_ROT_SPEED = 0.7 * exponentialFunction(epoch-100, 200, 0.2);
		}	
		
		float vadvance = 0;
		float vrot = 0;
		
		/////////////////////////////////////////////////
		//////   ROTATION SPEED
		////////////////////////////////////////////////
	
		// VRot is computed as the sum of three terms: angle with tangent to road + atan(perp. distance to road) + road curvature
		// as descirbed in Thrun's paper on DARPA challenge
		
		vrot = road.getAngleWithTangentAtClosestPoint() + atan( road.getRobotPerpendicularDistanceToRoad()/350.) + 0.8 * road.getRoadCurvatureAtClosestPoint() ;
	
	// Limiting filter
 		if( vrot > MAX_ROT_SPEED ) 
 			vrot = MAX_ROT_SPEED;
 		if( vrot < -MAX_ROT_SPEED ) 
 			vrot = -MAX_ROT_SPEED;
		
		/////////////////////////////////////////////////
		//////   ADVANCE SPEED
		////////////////////////////////////////////////
		
		// Factor to be used in speed control when approaching the end of the road
		float teta;
		if( road.getRobotDistanceToTarget() < 1000)
			teta = exponentialFunction(1./road.getRobotDistanceToTarget(),1./500,0.5, 0.1);
		else
			teta= 1;
		
// 		if( (road.getRobotDistanceToTarget() < 1000) and (derRobotDistanceToTarget < 0) and (vadvance > 0))
// 		{
// 			sunk = 0;
// 			road.
// 		}
		
		//VAdv is computed as a reduction of MAX_ADV_SPEED by three computed functions: 
		//				* road curvature reduces forward speed
		//				* VRot reduces forward speed
		//				* teta that applies when getting close to the target (1/roadGetCurvature)
		//				* a Delta that takes 1 if approaching the target is true, 0 otherwise. It applies only if at less than 1000m to the target
		
		vadvance = MAX_ADV_SPEED * exp(-fabs(1.6 * road.getRoadCurvatureAtClosestPoint())) 
								 * exponentialFunction(vrot, 0.8, 0.1)
								 * teta;
								 //* exponentialFunction(1./road.getRobotDistanceToTarget(),1./500,0.5, 0.1) 
								 //* sunk;
		
		//Pre-limiting filter to avoid displacements in very closed turns
		if( fabs(vrot) > 0.8)
			vadvance = 0;
	
 		// Limiting filter
 		if( vadvance > MAX_ADV_SPEED ) 
 			vadvance = MAX_ADV_SPEED;
 		
		/////////////////////////////////////////////////
		//////  LOWEST-LEVEL COLLISION AVOIDANCE CONTROL
		////////////////////////////////////////////////
		
		avoidanceControl(innerModel, road, laserData, vadvance, vrot);
		
		/////////////////////////////////////////////////
		//////   EXECUTION
		////////////////////////////////////////////////
		
		qDebug() << "------------------Controller Report ---------------;";
 		qDebug() << "	VAdv: " << vadvance << " VRot: " << vrot;
		qDebug() << "---------------------------------------------------;";
 		
 
   		try {	differentialrobot_proxy->setSpeedBase( vadvance, vrot);	} 
   		catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl;		}	
	}
	else
		try {	differentialrobot_proxy->setSpeedBase( 0, 0);	} 
		catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl;		}	
	
	epoch = reloj.restart();  //epcoh time in ms
	return false;
		
}


/**
 * @brief Lowest level of movement control
 * 
 * @param innerModel ...
 * @param road ...
 * @param laserData ...
 * @param vadvance ...
 * @param vrot ...
 * @return void
 */
void Controller::avoidanceControl(InnerModel& innerModel, WayPoints& road, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot)
{
	//compute repulsive forces from laser
	QVec res = QVec::zeros(3);
	float distN, laserMin;
	for(auto i : laserData)
	{
		//non-linear (exponential) transformation of the magnitude
		distN = exponentialFunction(i.dist, 500, 0.1, 0);
		//qDebug() << __FUNCTION__ << i.dist << distN;
		QVec p = innerModel.laserTo("laser", "laser" , distN, i.angle);
		p[0]=-p[0]; p[1]=0; p[2]=-p[2];
		if (laserMin < i.dist)
			laserMin = i.dist;
		//p.print("p");
		res = res + p;
	}
	qDebug() << __FUNCTION__ << "Resultant:" << res;
	
	//Combine p with vadvance, vrot to obtain the final control
	if( laserMin < 25 )
	{ 
		vadvance = 0; vrot = 0;
		qDebug() << __FUNCTION__ << "STOP due to inmminent risk of collision";
	//	return true;
	}
	
	{
			
	}
	
}


void Controller::stopTheRobot(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy)
{
	///CHECK IF ROBOT IS MOVING BEFORE
	try {	differentialrobot_proxy->setSpeedBase( 0.f, 0.f);	} 
	catch (const Ice::Exception &e) { std::cout << e << std::endl;}	
}

/**
 * @brief ...
 * 
 * @param value quantity to be tranformed
 * @param xValue for a point with xValue in X axis
 * @param yValue we want an yValue in Y axis
 * @param min ad if the result is less than min then the result is min
 * @return float
 */
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