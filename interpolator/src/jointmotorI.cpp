/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
#include "jointmotorI.h"

JointMotorI::JointMotorI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


JointMotorI::~JointMotorI()
{
}

MotorParamsList JointMotorI::getAllMotorParams(const Ice::Current&)
{
	worker->getAllMotorParams();
}

void JointMotorI::getAllMotorState(MotorStateMap &mstateMap, const Ice::Current&)
{
	worker->getAllMotorState(mstateMap);
}

MotorParams JointMotorI::getMotorParams(const string &motor, const Ice::Current&)
{
	worker->getMotorParams(motor);
}

MotorState JointMotorI::getMotorState(const string &motor, const Ice::Current&)
{
	worker->getMotorState(motor);
}

void JointMotorI::setSyncVelocity(const MotorGoalVelocityList &listGoals, const Ice::Current&)
{
	worker->setSyncVelocity(listGoals);
}

void JointMotorI::setZeroPos(const string &name, const Ice::Current&)
{
	worker->setZeroPos(name);
}

BusParams JointMotorI::getBusParams(const Ice::Current&)
{
	worker->getBusParams();
}

void JointMotorI::setSyncZeroPos(const Ice::Current&)
{
	worker->setSyncZeroPos();
}

void JointMotorI::setSyncPosition(const MotorGoalPositionList &listGoals, const Ice::Current&)
{
	worker->setSyncPosition(listGoals);
}

MotorStateMap JointMotorI::getMotorStateMap(const MotorList &mList, const Ice::Current&)
{
	worker->getMotorStateMap(mList);
}

void JointMotorI::setPosition(const MotorGoalPosition &goal, const Ice::Current&)
{
	worker->setPosition(goal);
}

void JointMotorI::setVelocity(const MotorGoalVelocity &goal, const Ice::Current&)
{
	worker->setVelocity(goal);
}






