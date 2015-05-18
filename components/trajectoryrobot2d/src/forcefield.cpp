/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "forcefield.h"
#include <boost/graph/graph_concepts.hpp>

ForceField::ForceField(InnerModel *innermodel, InnerModelViewer *innerViewer_): 
	innerModel(innermodel), 
	innerViewer = innerViewer_;
{

}

/**
 * @brief Recomputes the force field that affects the robot caused by the elastic band. So far assumes the robot is called "robot" in the XML
 * 
 * @param road Elastic Band
 * @return bool
 */
bool ForceField::update(WayPoints &road)
{
	//First we check if currentPoint has to be updated. The robot is always between currentPoint and nextPoint. In order to 
	//cross nextPoint it has to cross the line perpendicular to its Z axis and passing through nextPoint.
	
	qDebug() << __FILE__ << __FUNCTION__ << "Road size " << road.size();
	
	if((road.isFinished() == true) or (road.requiresReplanning == true)) 
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Nothing to do in PointsToRoad::update";
		return false;
	}
	
	//Get robot's position in world and create robot's nose
	QVec robot3DPos = innerModel->transform("world", "robot");
	QVec noseInRobot = innerModel->transform("world", QVec::vec3(0,0,1000), "robot");
	QLine2D nose =  QLine2D(  QVec::vec2(robot3DPos.x(),robot3DPos.z()), QVec::vec2(noseInRobot.x(), noseInRobot.z()));
	
	//Compute closest existing trajectory point to robot
	WayPoints::iterator closestPoint = road.computeClosestPointToRobot(robot3DPos);
	//Compute roadTangent at closestPoint;
	qDebug() << __FUNCTION__ << "just here" << road.getCurrentPointIndex() << road.getRobotDistanceToClosestPoint();
	if(closestPoint == road.end())
		qFatal("fary");
	QLine2D tangent = road.computeTangentAt( closestPoint );
	road.setTangentAtClosestPoint(tangent);
	//Compute signed perpenduicular distance from robot to tangent at closest point
	road.setRobotPerpendicularDistanceToRoad( tangent.perpendicularDistanceToPoint(robot3DPos) );
 	road.setAngleWithTangentAtClosestPoint( nose.signedAngleWithLine2D( tangent ));
	//compute distanceToTarget along trajectory
  	road.setRobotDistanceToTarget( distanceToTarget(road, closestPoint, robot3DPos) );
	
	//Check for arrival to target  TOO SIMPLE 
	if(	(int)road.getCurrentPointIndex()==(int)road.size()-1 and (int)road.getRobotDistanceToTarget()<100) 
		road.setFinished(true);
	
	//compute curvature of trajectory at closest point to robot
  	road.setRoadCurvatureAtClosestPoint( roadCurvature(road, closestPoint, 3) );
	road.setRobotDistanceToLastVisible( distanceToLastVisible(road, closestPoint, robot3DPos ) );
	return true;
}

/**
 * @brief Computes the distance from the robot to the last visible point in the road
 * 
 * @param road ...
 * @param closestPoint ...
 * @param robotPos ...
 * @return float
 */
float ForceField::distanceToLastVisible(const WayPoints &road, WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::const_iterator it;
	for(it = closestPoint; it != road.end()-1; ++it)
	{
		if(it->isVisible == true )
			dist += (it->pos - (it+1)->pos).norm2();
		else 
			break;
	}
	return dist;
}

/**
 * @brief Computes the distance to the Target along the road
 * 
 * @param road ...
 * @param robotPos ...
 * @return float Distance to target in the units of InnerModel
 */
float ForceField::distanceToTarget(WayPoints &road, WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for(it = closestPoint; it != road.end()-1; ++it)
	{
		dist += (it->pos - (it+1)->pos).norm2();
	}
	return dist;
}

/**
 * @brief Compute angle between current and next segment using the cross product
 * @param robot2DPos (X,Y) positionof the robot
 * @return float value between 0 curvature for straight lines and PI for U turns.
 */

float ForceField::roadCurvature(const WayPoints &road, WayPoints::iterator closestPoint, uint pointsAhead)
{
	WayPoints::iterator it, final;
	uint count = 0;
	int achieved = 0;
	float sumAng = 0.f;
	for(it = closestPoint; it != road.end()-1 and count < pointsAhead; ++it, ++count, ++achieved)
	{
		float ang = road.computeTangentAt( it ).signedAngleWithLine2D( road.computeTangentAt( it + 1));
		//qDebug() << achieved << road.size() << ang;
		sumAng += ang;
	}
	if( achieved > 0 and isnan(sumAng)==false) 
		return sumAng/achieved;
	else 
		return 0;
}

/**
 * @brief Substitutes value in an gaussian with a sigma such that when substituting xValue it returns yValue
 * 
 * @param value vlaue to be substituted int the gaussian
 * @param xValue X axis value that when substituted in the Gaussian
 * @param yValue gives y Value
 * @return float Computed vlaue
 */
// float ForceField::exponential(float value, float xValue, float yValue)
// {
// 	Q_ASSERT( yValue>0 );
// 	
// 	float landa = -fabs(xValue) / log(yValue);
// 	return exp(-fabs(value)/landa);
// }
