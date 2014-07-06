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

#define ROBOT_RADIUS 250

class WayPoint
{
	public:
		WayPoint(){};
		WayPoint(QVec p) { pos = p; minDist = ROBOT_RADIUS; minDistAnt = 0.f; isVisible = true; minDistPoint = QVec::zeros(3);};
		~WayPoint(){};
	
		//For ElasticBand
		QVec pos;
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
		void readRoadFromFile(InnerModel *innerModel, std::string name);
		void readRoadFromList(QList<QVec> list);
		
		void printRobotState(InnerModel* innerModel);
		void print();
		bool draw(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo = -1);  //Default in upTo means all list
		void removeFirst(InnerModelManagerPrx innermodelmanager_proxy);
		float robotDistanceToCurrentPoint(InnerModel *innerModel);
		float robotDistanceToNextPoint(InnerModel *innerModel);
		WayPoint const getCurrentPoint() const {return (*this)[currentPointIndex];};
		WayPoint const getNextPoint() const {return (*this)[nextPointIndex];};
		QLine2D getRobotZAxis(InnerModel* innerModel);
		void computeDistancesToNext();
		QLine2D getTangentToCurrentPoint();
		
		int currentPointIndex, nextPointIndex;
		
		//For PointsToRoad
		float distanceToRoad;
		float angleWithTangent;
		float distanceToTarget;
		float roadCurvature;
		float distanceToLastVisible;
		bool finish;
		bool isBlocked;
		bool isLost;
		int currentCollisionIndex;
		float currentDistanceToFrontier;
		bool requiresReplanning;
	
		
	private:
			

};

#endif // WAYPOINTS_H
