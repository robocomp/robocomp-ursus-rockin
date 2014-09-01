/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "trajectoryrobot2dI.h"

TrajectoryRobot2DI::TrajectoryRobot2DI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


TrajectoryRobot2DI::~TrajectoryRobot2DI()
{
	// Free component resources here
}

// Component functions, implementation
void TrajectoryRobot2DI::go(const TargetPose& target, const Ice::Current&){
	worker->go(target);
}

void TrajectoryRobot2DI::changeTarget(const TargetPose& target, const Ice::Current&){
	worker->changeTarget(target);
}

NavState TrajectoryRobot2DI::getState(const Ice::Current&){
	return worker->getState();
}

void TrajectoryRobot2DI::stop(const Ice::Current&){
	worker->stop();
}


