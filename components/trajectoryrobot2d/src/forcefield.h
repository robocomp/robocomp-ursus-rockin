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

#ifndef FORCEFIELD_H
#define FORCEFIELD_H

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include "qline2d.h"
#include <InnerModelManager.h>
#include "rcisdraw.h"
#include "waypoints.h"

class ForceField : public QObject
{
    Q_OBJECT
	public:
		ForceField(InnerModel *innermodel, InnerModelManagerPrx _innermodelmanager_proxy);
		bool update(WayPoints& road);
		
		struct State
		{
			float distanceToRoad;
			float angleWithTangent;
			float distanceToTarget;
			float roadCurvature;
			bool finish;
		};
		
//		QLine2D getRobotZAxis();
	private:
		InnerModel *innerModel;
		InnerModelManagerPrx innermodelmanager_proxy;
	
 		float computeAngleWithTangent(WayPoints &road, const QVec &robot2DPos);
 		float distanceToTarget(WayPoints& road, WayPoints::iterator closestPoint, const QVec& robotPos);
 		float roadCurvature(const WayPoints &road, WayPoints::iterator closestPoint, uint pointsAhead);
 		float distanceToLastVisible(const WayPoints &road, WayPoints::iterator closestPoint, const QVec &robotPos);	
 		//float exponential(float value, float xValue, float yValue);
};

#endif // FORCEFIELD_H
