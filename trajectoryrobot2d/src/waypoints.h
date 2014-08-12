/*
 * Copyright 2013 <copyright holder> <email>
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

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <QObject>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <iostream>
#include <fstream>
#include "rcisdraw.h"
#include <float.h>
#include "qline2d.h"

#define ROBOT_RADIUS 250

class WayPoint
{
	public:
		WayPoint()			{};
		WayPoint(QVec p) 	{ pos = p; minDist = ROBOT_RADIUS; minDistAnt = 0.f; isVisible = false; minDistPoint = QVec::zeros(3);};
		~WayPoint()			{};
	
		//For ElasticBand
		QVec pos;								// 3D point (x,y,z)
		QVec rot;								// Euler angles (x,y,z)
		float minDist, minDistAnt;
		QVec minDistPoint; //In world ref system
		float bMinusY, bPlusY, bMinusX, bPlusX;
		bool minDistHasChanged;
		QString centerTransformName, centerMeshName, centerLineName, centerPointName, ballTransformName, ballMeshName;
		bool isVisible;
		float initialDistanceToNext;
		float visibleLaserAngle;
		float visibleLaserDist;
		QVec posInRobotFrame;
		
}; 

class WayPoints : public QList< WayPoint >
{
	public:
		WayPoints();
		~WayPoints();
		void reset();
		void startRoad();
		void endRoad();
		void setInnerModel( InnerModel *inner) 												{ innerModel = inner;};
		void readRoadFromFile(InnerModel *innerModel, std::string name);
		void readRoadFromList(QList<QVec> list);
		void printRobotState(InnerModel* innerModel);
		void print();
		bool draw(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo = -1);  //Default in upTo means all list
		bool draw2(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo = -1);  //Default in upTo means all list
		void clearDraw(InnerModelManagerPrx innermodelmanager_proxy);
		void removeFirst(InnerModelManagerPrx innermodelmanager_proxy);
		float robotDistanceToCurrentPoint(InnerModel *innerModel);
		float robotDistanceToNextPoint(InnerModel *innerModel);
		WayPoint const getCurrentPoint() const 												{return (*this)[currentPointIndex];};
		WayPoint const getNextPoint() const 												{return (*this)[nextPointIndex];};
		QLine2D getRobotZAxis(InnerModel* innerModel);
		void computeDistancesToNext();
		QList<QVec> backList;
		QMutex mutex;
		
		//GOOD ONES
		QLine2D getTangentToCurrentPoint();
		void setIndexOfClosestPointToRobot(WayPoints::iterator it) 							{ indexOfClosestPointToRobot = it;};
		WayPoints::iterator getIndexOfClosestPointToRobot() const 							{ return indexOfClosestPointToRobot;};
		void setTangentAtClosestPoint(const QLine2D &tangent) 								{ roadTangentAtClosestPoint = tangent;};
		QLine2D getTangentAtClosestPoint() const											{ return roadTangentAtClosestPoint;};
		void setRobotDistanceToClosestPoint(float dist) 									{ robotDistanceToClosestPoint = dist;};
		float getRobotDistanceToClosestPoint() 	const										{ return robotDistanceToClosestPoint;};
		void setRobotPerpendicularDistanceToRoad(float dist) 								{ robotPerpendicularDistanceToRoad = dist;};
		float getRobotPerpendicularDistanceToRoad()	const									{ return robotPerpendicularDistanceToRoad;};
		void setAngleWithTangentAtClosestPoint( float ang)									{ angleWithTangentAtClosestPoint = ang;};
		float getAngleWithTangentAtClosestPoint() const										{ return angleWithTangentAtClosestPoint;};
		uint getCurrentPointIndex() const													{ return currentPointIndex;};  //DEPRECATED
		float getRoadCurvatureAtClosestPoint() const										{ return roadCurvatureAtClosestPoint;};
		void setRoadCurvatureAtClosestPoint( float c)										{ roadCurvatureAtClosestPoint = c;};
		float getRobotDistanceToTarget() const												{ return robotDistanceToTarget;};
		void setRobotDistanceToTarget( float dist)											{ robotDistanceToTarget = dist;};
		float getRobotDistanceToLastVisible() const											{ return robotDistanceToLastVisible;};
		void setRobotDistanceToLastVisible( float dist)										{ robotDistanceToLastVisible = dist;};
		void setFinished( bool b)															{ QMutexLocker ml(&mutex); finish = b; }
		bool isFinished() const 															{ return finish;};
		void setRobotDistanceVariationToTarget(float dist)									{ robotDistanceVariationToTarget = dist;};
		float getRobotDistanceVariationToTarget() const 									{ return robotDistanceVariationToTarget;};
		ulong getETA() const 																{ return estimatedTimeOfArrival;};
		WayPoints::iterator getIndexOfLastVisiblePoint() const								{ return indexOfLastVisiblePoint;};
		uint32_t getOrderOfClosestPointToRobot() const										{ return orderOfClosestPointToRobot;};
		
		int nextPointIndex;
	//	float distanceToLastVisible;
		bool isBlocked;
		bool isLost;
		int currentCollisionIndex;
		float currentDistanceToFrontier;
		bool requiresReplanning;
		
		
		bool computeForces();
		float computeRoadCurvature(WayPoints::iterator closestPoint, uint pointsAhead);
		float computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos);
		float computeDistanceToLastVisible(WayPoints::iterator closestPoint, const QVec &robotPos);
		QLine2D computeTangentAt(WayPoints::iterator w) const;
		void setETA();
		WayPoints::iterator computeClosestPointToRobot(const QVec& robot);
		
		
	private:
		
		float robotDistanceToClosestPoint;
		float robotPerpendicularDistanceToRoad;
		QLine2D roadTangentAtClosestPoint;
		WayPoints::iterator indexOfClosestPointToRobot, indexOfLastVisiblePoint;
		uint32_t orderOfClosestPointToRobot;
		uint currentPointIndex;
		float angleWithTangentAtClosestPoint;
		float roadCurvatureAtClosestPoint;
		float robotDistanceToTarget;
		float robotDistanceVariationToTarget;
		float robotDistanceToLastVisible;
		bool finish;
		ulong estimatedTimeOfArrival;
		InnerModel *innerModel;
		QTime reloj;
		float meanSpeed;  
		long elapsedTime;
		int initialDurationEstimation;
};

#endif // WAYPOINTS_H
