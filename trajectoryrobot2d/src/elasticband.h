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

#ifndef ELASTICBAND_H
#define ELASTICBAND_H

#include <QtCore>
#include <qmat/QMatAll>
#include "rcisdraw.h"
#include <innermodel/innermodel.h>
#include <Laser.h>
#include <limits>       // std::numeric_limits
#include "waypoints.h"
#include <assert.h>
#include "currenttarget.h"

#define FORCE_DISTANCE_LIMIT 500  //mm
#define ROBOT_STEP 50
#define DELTA_H 50

class ElasticBand
{
	public:
		ElasticBand(InnerModel *_innermodel);
		~ElasticBand();
		bool update(WayPoints &road, const RoboCompLaser::TLaserData &laserData, const CurrentTarget &currentTarget, uint iter = 1);
		void addPoints(WayPoints &road, const CurrentTarget &currentTarget);

	private:		
		InnerModel *innermodel;
		float computeIntersectionChord( const WayPoint b1, const WayPoint b2);
		void computeDistanceField(WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit);
		bool computeFreePath(const WayPoint &w1, const WayPoint &w2, const RoboCompLaser::TLaserData &laserData );
		void checkBlocked(WayPoints &road, const RoboCompLaser::TLaserData &laserData);
		bool checkCollision(WayPoints &road, const RoboCompLaser::TLaserData &laserData,float robotRadius);
		float computeForces(WayPoints &road, const RoboCompLaser::TLaserData& laserData);
		void cleanPoints(WayPoints &road);
		bool checkVisiblePoints(WayPoints &road, const RoboCompLaser::TLaserData &laserData);
		bool shortCut(WayPoints& road);
};

#endif // ELASTICBAND_H
