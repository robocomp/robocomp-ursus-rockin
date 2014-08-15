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


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QtCore>
#include "pointstoroad.h"
#include <DifferentialRobot.h>
#include "waypoints.h"
#include <innermodel/innermodel.h>
#include <Laser.h>

class Controller
{
	public:
		Controller(int delay); //in secs
		~Controller();
	
		bool update(InnerModel& innerModel, const RoboCompLaser::TLaserData &laserData, RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy, WayPoints& road);
		void stopTheRobot(RoboCompDifferentialRobot::DifferentialRobotPrx differentialrobot_proxy);
float exponentialFunction(float value, float xValue, float yValue, float min = 0.f);
private:
		QTime time;
		int delay;
		
		void avoidanceControl(InnerModel &innerModel, WayPoints &road, const RoboCompLaser::TLaserData &laserData, float &vadvance, float &vrot);
};

#endif // CONTROLLER_H
