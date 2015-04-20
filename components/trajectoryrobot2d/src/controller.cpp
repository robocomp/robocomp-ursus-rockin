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
	static long epoch = 100;

	//Estimate the space that will be blindly covered and reduce Adv speed to remain within some boundaries
	//qDebug() << __FILE__ << __FUNCTION__ << "entering update with" << road.at(road.getIndexOfClosestPointToRobot()).pos;

	//Check robot state
	if( (road.isFinished() == true ) or (road.requiresReplanning== true) or (road.isLost == true))
	{
		// 		if( road.isFinished() ) qDebug() << "road finished";
		// 		if( road.requiresReplanning ) qDebug() << "requiresReplanning";
		// 		if( road.isLost ) qDebug() << "robot is lost";
		stopTheRobot(omnirobot_proxy);
		return false;
	}

	///CHECK ROBOT INMINENT COLLISION
	int j=0;
	for(auto i : laserData)
	{
		if( i.dist < baseOffsets[j++] )
		{
			qDebug() << __FILE__ << __FUNCTION__<< "Robot stopped to avoid collision because distance to obstacle is less than " << baseOffsets[j];
			//road.requiresReplanning = true;
			stopTheRobot(omnirobot_proxy);
			road.setBlocked(true);
			return false;
		}
	}

	/////////////////////////////////////////////////
	//////  CHECK CPU AVAILABILITY
	////////////////////////////////////////////////

	if ( time.elapsed() > delay )   //Initial wait in secs so the robot waits for everything is setup. Maybe it could be moved upwards
	{
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

		//VAdv is computed as a reduction of MAX_ADV_SPEED by three computed functions:
		//				* road curvature reduces forward speed
		//				* VRot reduces forward speed
		//				* teta that applies when getting close to the target (1/roadGetCurvature)
		//				* a Delta that takes 1 if approaching the target is true, 0 otherwise. It applies only if at less than 1000m to the target

		vadvance = MAX_ADV_SPEED * exp(-fabs(1.6 * road.getRoadCurvatureAtClosestPoint()))
								 * exponentialFunction(vrot, 0.8, 0.01)
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

		bool collision = avoidanceControl(innerModel, laserData, vadvance, vrot);
//  		if( collision )
//  			road.setBlocked(true);

		/////////////////////////////////////////////////
		///  SIDEWAYS LASTMINUTE AVOIDING WITH THE OMNI BASE
		///
		/////////////////////////////////////////////////
		float vside = 0;
		std::sort(laserData.begin(), laserData.end(), [](auto a, auto b){ return a.dist < b.dist;});
		if(laserData.front().dist < 100)// and fabs(laserData.front().angle)>0.3)
		{
			if( laserData.front().angle > 0) vside  = -300;
			else vside = 300;
		}
		else
			vside = 0;
		
		/////////////////////////////////////////////////
		//////   EXECUTION
		////////////////////////////////////////////////

 		qDebug() << "------------------Controller Report ---------------;";
  		qDebug() << "	VAdv: " << vadvance << " VRot: " << vrot;
 		qDebug() << "---------------------------------------------------;";


   		try { omnirobot_proxy->setSpeedBase(vside, vadvance, vrot); }
   		catch (const Ice::Exception &e) { std::cout << e << "Omni robot not responding" << std::endl; }
	}
	else		//Too long delay. Stopping robot.
	{	
		qDebug() << __FILE__ << __FUNCTION__ << "Processing delay" << epoch << "ms. too high. Stopping the robot for safety";
		try { omnirobot_proxy->setSpeedBase( 0, 0, 0);	}
		catch (const Ice::Exception &e) { std::cout << e << "Omni robot not responding" << std::endl; }
	}

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
bool Controller::avoidanceControl(InnerModel *innerModel, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot)
{
	QVec res = QVec::zeros(3);
	float distN, distNorm;
	int k=0;
	bool collision = false;
	for(auto i : laserData)
	{
		//non-linear (exponential) transformation of the magnitude
		distN = std::max<float>(i.dist - baseOffsets[k], 0);
		k++;
		distNorm = exponentialFunction(distN, 300, 0.1, 0);
		//qDebug() << distNorm;
		QVec p = innerModel->laserTo("laser", "laser" , distNorm, i.angle);  //Watch the laser to tobot offset to compute final corrections
		res += (p * (T)(-1));
		if( i.dist < 100.f)
		{
			collision = true;
			vadvance = 0;
			vrot = 0;
		}
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
			if( p.x()*p.x() + p.z()*p.z() - 2*200*200 >= 0) 
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