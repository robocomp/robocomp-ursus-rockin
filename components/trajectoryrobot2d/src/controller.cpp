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

Controller::Controller(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, int delay)  //in secs
{
	this->time = QTime::currentTime();
	this->delay = delay*1000;	//msecs

	//compute offsets from laser center to the border of the robot base
	baseOffsets = computeRobotOffsets(innerModel, laserData);
}

Controller::~Controller()
{
}

bool Controller::update(InnerModel *innerModel, RoboCompLaser::TLaserData &laserData, RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy, WayPoints &road)
{
	static QTime reloj = QTime::currentTime();   //TO be used for a more accurate control (predictive).
	long epoch = 100;
 	static float lastVadvance = 0.f;
 	static float lastVrot = 0.f;
// 	const float umbral = 25.f;	//salto maximo de velocidad
// 	const float umbralrot = 0.08f;	//salto maximo de rotación

	//Estimate the space that will be blindly covered and reduce Adv speed to remain within some boundaries
	//qDebug() << __FILE__ << __FUNCTION__ << "entering update with" << road.at(road.getIndexOfClosestPointToRobot()).pos;

	//////////////////////////////////////////////	
	// Check if robot goal is achieved already
	//////////////////////////////////////////////	
	
	if(road.isFinished() == true ) 
	{		
		qDebug() << __FUNCTION__ << "road finished. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
	if(road.requiresReplanning == true ) 
	{		
		qDebug() << __FUNCTION__ << "requiresReplanning. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
	if(road.isLost == true ) 
	{		
		qDebug() << __FUNCTION__ << "robot is lost. Returning to main";
		stopTheRobot(omnirobot_proxy);
		return false;
	}
		
	//////////////////////////////////////////////	
	///CHECK ROBOT FOR INMINENT COLLISION
	///////////////////////////////////////////////
	float vside = 0;
	int j=0;
	road.setBlocked(false);
	for(auto i : laserData)
	{
		//printf("laser dist %f || baseOffsets %f \n",i.dist,baseOffsets[j]);
		if(i.dist < 10) i.dist = 30000;
		if( i.dist < baseOffsets[j] + 50 )
		{
			if(i.angle>-1.30 && i.angle<1.30){
			qDebug() << __FUNCTION__<< "Robot stopped to avoid collision because distance to obstacle is less than " << baseOffsets[j] << " "<<i.dist << " " << i.angle;
			stopTheRobot(omnirobot_proxy);
			road.setBlocked(true);		//AQUI SE BLOQUEA PARA REPLANIFICAR
			qDebug() << __FUNCTION__ << "Inminent collision, REPLANNING";
 			break;
			}
		}
		else
		{
			if (i.dist < baseOffsets[j] + 150) 
			{
				if (i.angle > 0)
				{
					vside  = -80;
				}
				else
				{
					vside = 80;
				}
			}
		}
		j++;
	}

	/////////////////////////////////////////////////
	//////  CHECK CPU AVAILABILITY. If lagging reduce speed
	/////////////////////////////////////////////////
	if ( this->time.elapsed() > this->delay )   //Initial wait in secs so the robot waits for everything is setup. Maybe it could be moved upwards
	{
		if( epoch  > MAX_LAG )				//Damp max speed if elapsed time is too long  TAKE CONSTANT OUT!
		{
			MAX_ADV_SPEED = 200 * exponentialFunction(epoch-100, 200, 0.2);
			MAX_ROT_SPEED = 0.3 * exponentialFunction(epoch-100, 200, 0.2);
		}
		//float vadvance = 0;
		//float vrot = 0;
	}
	
	/////////////////////////////////////////////////
	//////   ROTATION SPEED
	////////////////////////////////////////////////

	// VRot is computed as the sum of three terms: 
	// --   angle between robot nose and tangent to road at its closest point
	// --   atan(perp. distance to road), to force an inwards turn when the robot is off the center line
	// --   road curvature, to reduce speed inside curves
	//
	// as descirbed in Thrun's paper on DARPA challenge
	
	float vrot = 0.5 * road.getAngleWithTangentAtClosestPoint() + 
							 1.0 * atan( road.getRobotPerpendicularDistanceToRoad()/1000.) + 
							 0.8 * road.getRoadCurvatureAtClosestPoint() ;  //350->800.
	
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
	
	//VAdv is computed as a factored reduction of MAX_ADV_SPEED by three computed functions:
	//				* road curvature reduces forward speed
	//				* VRot reduces forward speed
	//				* reduction is 1 if there are not obstacle.
	//				* teta that applies when getting close to the target (1/roadGetCurvature)
	//				* a Delta that takes 1 if approaching the target is true, 0 otherwise. It applies only if at less than 1000m to the target
	float vadvance = MAX_ADV_SPEED 
								* exp(-fabs(1.6 * road.getRoadCurvatureAtClosestPoint()))
								* exponentialFunction(vrot, 0.8, 0.01)
								* teta;
								//* exponentialFunction(1./road.getRobotDistanceToTarget(),1./500,0.5, 0.1)
								//* sunk;

// 	if(fabs(vrot - lastVrot) > umbralrot)
// 	{
// 		//qDebug()<<"lastrot "<<lastVrot << "\n vrot "<< vrot;
// 		if(vrot > lastVrot)
// 			vrot = lastVrot + umbralrot;
// 		else vrot = lastVrot - umbralrot;
// 	}
	lastVrot=vrot;

	//Pre-limiting filter to avoid displacements in very closed turns
	if( fabs(vrot) == 0.3)
		vadvance = 0;
		vside = 0;
	
	//stopping speed jump
// 	if(fabs(vadvance - lastVadvance) > umbral)
// 	{
// 		//qDebug()<<"lastadvanced "<<lastVadvance << "\n vadvance "<< vadvance;
// 		if(vadvance > lastVadvance)
// 			vadvance = lastVadvance + umbral;
// 		else vadvance = lastVadvance - umbral;
// 	}
	lastVadvance=vadvance;

	// Limiting filter
	if( vadvance > MAX_ADV_SPEED )
		vadvance = MAX_ADV_SPEED;
	

	//vside = vrot*MAX_ADV_SPEED;
	
	/////////////////////////////////////////////////
	//////  LOWEST-LEVEL COLLISION AVOIDANCE CONTROL
	////////////////////////////////////////////////

	//bool collision = avoidanceControl(innerModel, laserData, vadvance, vrot);
//  		if( collision )
//  			road.setBlocked(true);

	/////////////////////////////////////////////////
	///  SIDEWAYS LASTMINUTE AVOIDING WITH THE OMNI BASE
	/////////////////////////////////////////////////
	//TODO: PROBAR EN URSUS A VER COMO QUEDA..
	
// 		std::sort(laserData.begin(), laserData.end(), [](auto a, auto b){ return a.dist < b.dist;});
// 		if(laserData.front().dist > 300 && vside == 0)// and fabs(laserData.front().angle)>0.3)
// 		{
// 			if( laserData.front().angle > 0) vside  = -30;
// 			else vside = 30;
// 		}
	
	/////////////////////////////////////////////////
	//////   EXECUTION
	////////////////////////////////////////////////

// 		qDebug() << "------------------Controller Report ---------------;";
// 		qDebug() <<"	VAdv: " << vadvance << "|\nVRot: " << vrot << "\nVSide: " << vside;
// 		qDebug() << "---------------------------------------------------;";
							
		try { omnirobot_proxy->setSpeedBase(vside, vadvance, vrot);}
		catch (const Ice::Exception &e) { std::cout << e << "Omni robot not responding" << std::endl; }
	
	epoch = reloj.restart();  //epoch time in ms
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
bool Controller::avoidanceControl(InnerModel *innerModel, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot)
{
	QVec res = QVec::zeros(3);
	float distN, distNorm;
	int k=0;
	bool collision = false;
	float minD = 999999999;
	for(auto i : laserData)
	{
		//non-linear (exponential) transformation of the magnitude
		distN = std::max<float>(i.dist - baseOffsets[k], 0);
		k++;
		distNorm = exponentialFunction(distN, 300, 0.1, 0);
		//qDebug() << distNorm;
		QVec p = innerModel->laserTo("laser", "laser" , distNorm, i.angle);  //Watch the laser to tobot offset to compute final corrections
		res += (p * (T)(-1));
		if (i.dist<minD)
			minD = i.dist;
		if( i.dist < 400.f)
		{
			collision = true;
			vadvance = 0;
			vrot = 0;
		}
		qDebug() << __FUNCTION__ << "min distance = " << minD;
	}
	return collision;
}

/**
 * @brief Offset computation of each laser beam to account for the geometry of the getBaseState
 *
 * @param innerModel ...
 * @param laserData ...
 * @return std::vector< float, std::allocator >
 */
std::vector<float> Controller::computeRobotOffsets(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData)
{
	//Base geometry GET FROM IM!!!
	//QRectF base( QPointF(-200, 200), QPointF(200, -200));
	std::vector<float> baseOffsets;
	QVec p(3,0.f);
	int k;

	for(auto i : laserData)
	{
		for(k = 10; k < 4000; k++)
		{
			p = innerModel->laserTo("robot","laser",k,i.angle);
			if( sqrt(p.x()*p.x() + p.z()*p.z()) - 250 >= 0) 
			//if( base.contains( QPointF( p.x(), p.z() ) ) == false )
				break;
		}
		baseOffsets.push_back(k);
	}
	return baseOffsets;
}



void Controller::stopTheRobot(RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy)
{
	///CHECK IF ROBOT IS MOVING BEFORE
	qDebug() << __FUNCTION__ << "Stopping the robot";
	try {	omnirobot_proxy->setSpeedBase( 0.f, 0.f, 0.f);	}
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
