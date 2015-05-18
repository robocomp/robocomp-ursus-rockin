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


#ifndef POINTSTOROAD_H
#define POINTSTOROAD_H

#include <QObject>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include "qline2d.h"
#include <InnerModelManager.h>
#include "waypoints.h"

using namespace RMat;
using namespace RoboCompInnerModelManager;

class PointsToRoad : public QObject
{
	Q_OBJECT
	
	public:
		PointsToRoad( InnerModel *innermodel, InnerModelManagerPrx _innermodelmanager_proxy);
		~PointsToRoad();
	
		bool update(WayPoints &road);
		void setRoad(WayPoints &road);
		
		struct State
		{
			float distanceToRoad;
			float angleWithTangent;
			float distanceToTarget;
			float roadCurvature;
			bool finish;
		};
		
		QLine2D getRobotZAxis();
		
	private:		
	
		InnerModel *innerModel;
		//int currentPoint, nextPoint;
		bool currentSign;
		InnerModelManagerPrx innermodelmanager_proxy;
	
		float getAngleWithTangent(WayPoints &road, const QVec &robot2DPos);
		float distanceToRoad(WayPoints &road, const QVec &robot2DPos);
		float distanceToTarget(WayPoints &road, const QVec &robotPos);
		float roadCurvature(WayPoints &road, const QVec &robotPos);
		float distanceToNextVisible(const WayPoints &road, const QVec& robotPos);
		
		QLine2D getCurrentLineSegment(WayPoints &road);
		QLine2D getNextLineSegment(WayPoints &road);
		bool getRobotFrontierCondition(WayPoints &road);
		float exponential(float value, float xValue, float yValue);
};

#endif // POINTSTOROAD_H
