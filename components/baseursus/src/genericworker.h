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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <JointMotor.h>
#include <DifferentialRobot.h>
#include <OmniRobot.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompJointMotor;
using namespace RoboCompDifferentialRobot;
using namespace RoboCompOmniRobot;

class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx, QObject *parent = 0);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;                //Shared mutex with servant

	JointMotorPrx jointmotor_proxy;
	virtual void  getBaseState(RoboCompOmniRobot::TBaseState& state) = 0;
	virtual void  getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha) = 0;
	virtual void  setSpeedBase(float advx, float advz, float rot) = 0;
	virtual void  stopBase() = 0;
	virtual void  resetOdometer() = 0;
	virtual void  setOdometer(const RoboCompOmniRobot::TBaseState& state) = 0;
	virtual void  setOdometerPose(int x, int z, float alpha) = 0;
	virtual void  correctOdometer(int x, int z, float alpha) = 0;
protected:
	QTimer timer;
	int Period;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif