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
#ifndef OMNIROBOTI_H
#define OMNIROBOTI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <OmniRobot.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompOmniRobot;

class OmniRobotI : public QObject , public virtual RoboCompOmniRobot::OmniRobot
{
Q_OBJECT
public:
	OmniRobotI( GenericWorker *_worker, QObject *parent = 0 );
	~OmniRobotI();
	

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:

	void getBaseState(::RoboCompOmniRobot::TBaseState &a, const ::Ice::Current & = ::Ice::Current())
	{
		worker->getBaseState(a);
	}

	void getBasePose(::Ice::Int &a, ::Ice::Int &b, ::Ice::Float &c, const ::Ice::Current & = ::Ice::Current())
	{
		worker->getBasePose(a, b, c);
	}

	void setSpeedBase(::Ice::Float a, ::Ice::Float b, ::Ice::Float c, const ::Ice::Current& = ::Ice::Current())
	{
		worker->setSpeedBase(a,b,c);
	}

	void stopBase(const ::Ice::Current& = ::Ice::Current())
	{
		worker->stopBase();
	}

	void resetOdometer(const ::Ice::Current& = ::Ice::Current())
	{
		worker->resetOdometer();
	}

	void setOdometer(const ::RoboCompOmniRobot::TBaseState &a, const ::Ice::Current& = ::Ice::Current())
	{
		worker->setOdometer(a);
	}

	void setOdometerPose(::Ice::Int a, ::Ice::Int b, ::Ice::Float c, const ::Ice::Current& = ::Ice::Current())
	{
		worker->setOdometerPose(a,b,c);
	}

	void correctOdometer(::Ice::Int a, ::Ice::Int b, ::Ice::Float c, const ::Ice::Current& = ::Ice::Current())
	{
		worker->correctOdometer(a, b, c);
	}
};

#endif