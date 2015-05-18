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


#include "pointstoroad.h"

PointsToRoad::PointsToRoad(InnerModel *innermodel): 
	innerModel(innermodel), 
{
}

PointsToRoad::~PointsToRoad()
{
}

/**
 * @brief Loads the trajectory as a list of QVec (3D points)
 * 
 * @param _road List of points 3D as QVec
 * @return void
 */
void PointsToRoad::setRoad(WayPoints &road)
{
	Q_ASSERT(_road.size() > 1);
	//Initialize current robot position wrt to frontier line
	currentSign = getRobotFrontierCondition(road);
	qDebug() << __FILE__ << __FUNCTION__ << "currentSign" << currentSign;

}

bool PointsToRoad::update(WayPoints &road)
{
	//First we check if currentPoint has to be updated. The robot is always between currentPoint and nextPoint. In order to 
	//cross nextPoint it has to cross the line perpendicular to its Z axis and passing through nextPoint.
	
	if((road.finish == true) or (road.requiresReplanning == true)) 
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Nothing to do in PointsToRoad::update";
		return false;
	}
		
	//qDebug() << "Entering update";
	bool frontier = getRobotFrontierCondition(road);
	
	qDebug() << __FUNCTION__ << "PointsToRoad::update: frontier" << frontier << ". Current sign" << currentSign << ". Cur point" << road.currentPointIndex 
			 << ". Next point " << road.nextPointIndex << ". Road size" << road.size() 
			 << "Distance to next point" << road.robotDistanceToNextPoint(innerModel) << "Dist to current " << road.robotDistanceToCurrentPoint(innerModel);

//  	if( road.robotDistanceToNextPoint(innerModel) < road.robotDistanceToCurrentPoint(innerModel) )
// 	{
// 		road.isLost = true;
// 		qDebug() << __FILE__ << __FUNCTION__ << "Loosing the road";
// 	}
// 	else 
// 		road.isLost = false;
			 
	//Check if crossed the line!
	if ( (frontier != currentSign) /*and (road.robotDistanceToNextPoint(innerModel) < road.robotDistanceToCurrentPoint(innerModel))*/ ) 	
	{
		qDebug() << "--------------------------linecrossed--------------------------";
		if( road.nextPointIndex == road.size()-1 ) //The final goal is reached
		{
			road.finish = true; 
			qDebug() <<__FILE__ << __FUNCTION__  << "PointsToRoad::update-Road finished!";
			return false;
		}
		//road to go
		road.currentPointIndex++;
		road.nextPointIndex++;
	
		currentSign = getRobotFrontierCondition(road);
	
		//Remove point just passed
		for(int i=0; i<road.currentPointIndex; i++)
		{
			qDebug() << "removing point " << i;
		}
		road.currentPointIndex = 0;
		road.nextPointIndex = 1;
	}
	else if (frontier != currentSign)
	{
		currentSign = getRobotFrontierCondition(road);
	}
	

	QVec robot2DPos = QVec::vec2( innerModel->getBaseX(), innerModel->getBaseZ());
// 	road.angleWithTangent = getAngleWithTangent(road, robot2DPos);
// 	road.distanceToRoad = distanceToRoad(road, robot2DPos);
// 	road.distanceToTarget = distanceToTarget(road, innerModel->getBaseCoordinates());
// 	road.roadCurvature = roadCurvature(road, innerModel->getBaseCoordinates());
// 	road.distanceToLastVisible = distanceToNextVisible(road, innerModel->getBaseCoordinates() );

	return true;
}

/////////////////////////////////////////////////////////////////////////

/**
 * @brief Returns true if the perpendicular distance to the Frontier line es greater than 0
 * 
 * @param road ...
 * @return bool
 */
bool PointsToRoad::getRobotFrontierCondition(WayPoints &road)
{	
	//Build a line perpendicular to the robot Z axis and passing through road[nextPoint]
//	QLine2D frontier( getRobotZAxis().getPerpendicularVector(), road[road.nextPointIndex].pos.x(), road[road.nextPointIndex].pos.z() );
	QLine2D frontier =  getRobotZAxis().getPerpendicularLineThroughPoint( QVec::vec2(road.getNextPoint().pos.x(), road.getNextPoint().pos.z()));
	
	//Check if the point where RobotZAxis and Frontier intersect is ahead of the robot by 150mm.
	//QVec inter = frontier.intersectionPoint(getRobotZAxis());
	//QVec interRobot = innerModel->transform("base", QVec::vec3(inter.x(), 0, inter.y()), "world");
	//interRobot.print("intersection of Robot Axis with frontier");

//	if( interRobot.z() > -100)
	{
			// To draw perpendicular lines to robot through next point
// 		QString item = "p_" + QString::number(road.nextPoint);
// 		QVec n = frontier.getIntersectionPointOfNormalThroughOrigin();
		//Get the sign of the signed distance of the robot to the line
		QVec robot2DPos = QVec::vec2( innerModel->getBaseX(), innerModel->getBaseZ());
		float dist = frontier.perpendicularDistanceToPoint( robot2DPos );
		//frontier.print("frontier");
		//robot2DPos.print("Robot Position");
		//getRobotZAxis().print("Robot Axis");
		
	  road.currentDistanceToFrontier = dist;
	  return (dist>=0); 
	}
//  	else 
// 	{
// 		qDebug()<< "frontier anomaly";
//  		return currentSign;
// 	}
}

QLine2D PointsToRoad::getCurrentLineSegment(WayPoints &road)
{
	Q_ASSERT (currentPoint < road.size() -1 and road.size()>0);

	QVec p1 = QVec::vec2( road[road.currentPointIndex].pos.x(), road[road.currentPointIndex].pos.z());
	QVec p2 = QVec::vec2( road[road.currentPointIndex+1].pos.x(), road[road.currentPointIndex+1].pos.z());	
	QLine2D line( p1, p2);
	return line;
}

QLine2D PointsToRoad::getNextLineSegment(WayPoints &road)
{
	Q_ASSERT (nextPoint < road.size() -1 and road.size()>0);

	QVec p1 = QVec::vec2( road[road.nextPointIndex].pos.x(), road[road.nextPointIndex].pos.z());
	QVec p2 = QVec::vec2( road[road.nextPointIndex+1].pos.x(), road[road.nextPointIndex+1].pos.z());	
	QLine2D line( p1, p2);
	return line;
}

/**
 * @brief Compute QLine2D corresponding to the robot Z axis in World reference frame
 * 
 * @return QLine2D
 */
QLine2D PointsToRoad::getRobotZAxis()
{
	Q_ASSERT(currentPoint<road.size()-1 and road.size()>0);
	
	QVec nose = innerModel->transform("world", QVec::vec3(0,0,1000), "base");
	return QLine2D( nose, innerModel->getBaseCoordinates());
}

/**
 * @brief Compute the perpendicular distance of the robot to the road tangent
 * The signed distance is computed subtituting bState intor the general equation of the current segment and dividing by sqrt(A²+B²)	 
 * @param bState current pose of the robot (x,y,alfa)
 * @return float distance
 */
float PointsToRoad::distanceToRoad(WayPoints &road, const QVec& robot2DPos)
{	
		return getCurrentLineSegment(road).perpendicularDistanceToPoint(robot2DPos);
}

float PointsToRoad::distanceToTarget(WayPoints &road, const QVec& robotPos)
{
	Q_ASSERT(nextPointIndex<road.size() and road.size()>0);
	
	float dist = (robotPos - road.getNextPoint().pos).norm2();
	for(int i=road.nextPointIndex; i<road.size()-1; i++)
	{
		dist += (road[i].pos - road[i+1].pos).norm2();
	}
	return dist;
}

/**
 * @brief Compute the line of the robot Z axis in World coordinate frame
 * 
 * @param robot2DPos ...
 * @return float
 */
float PointsToRoad::getAngleWithTangent(WayPoints &road, const QVec& robot2DPos)
{			
	return getRobotZAxis().signedAngleWithLine2D( getCurrentLineSegment(road) );
}
/**
 * @brief Compute angle between current and next segment using the cross product
 * @param robot2DPos (X,Y) positionof the robot
 * @return float value between 0 curvature for straight lines and PI for U turns.
 */

float PointsToRoad::roadCurvature(WayPoints &road, const QVec& robotPos)
{
	if( road.currentPointIndex < road.size()-2 and road.size()>2 )
	{	
		float curv = getCurrentLineSegment(road).signedAngleWithLine2D(getNextLineSegment(road));
		return curv * exponential((robotPos-road.getNextPoint().pos).norm2(), 250, 0.5);
	}
	else 
		return 0.f;
}

// float PointsToRoad::maxRoadCurvatureAhead(WayPoints &road, const QVec& robotPos)
// {
// 	int index = road.currentPointIndex;
// 	float maxC = -999999;
// 	while( index < road.size()-2 )
// 	{
// 		current = getLineSegmentBetweenPoints(road, index, index+1).signedAngleWithLine2D(getLineSegmentBetweenPoints(road, index+1, index+2));
// 		if (current > maxC)
// 			maxC = current;
// 	}
// }

float PointsToRoad::distanceToNextVisible(const WayPoints &road, const QVec& robotPos)
{
	float dist = (robotPos - road.getNextPoint().pos).norm2();
	for(int i=road.nextPointIndex; i<road.size()-1; i++)
	{
		if( road[i].isVisible == true )
			dist += (road[i].pos - road[i+1].pos).norm2();
		else 
			break;
	}
	return dist;
}


float PointsToRoad::exponential(float value, float xValue, float yValue)
{
	Q_ASSERT( yValue>0 );
	
	float landa = -fabs(xValue) / log(yValue);
	return exp(-fabs(value)/landa);
}
