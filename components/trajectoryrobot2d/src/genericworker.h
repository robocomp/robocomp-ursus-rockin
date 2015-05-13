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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <ui_mainUI.h>

#include <CommonBehavior.h>
#include <OmniRobot.h>
#include <Laser.h>
#include <InnerModelManager.h>
#include <TrajectoryRobot2D.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompOmniRobot;
using namespace RoboCompLaser;
using namespace RoboCompInnerModelManager;
using namespace RoboCompTrajectoryRobot2D;

class GenericWorker : 
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;

	InnerModelManagerPrx innermodelmanager_proxy;
	LaserPrx laser_proxy;
	OmniRobotPrx omnirobot1_proxy;
	OmniRobotPrx omnirobot2_proxy;

	virtual void go(const TargetPose &target) = 0;
	virtual NavState getState() = 0;
	virtual void goBackwards(const TargetPose &target) = 0;
	virtual void stop() = 0;
	virtual void changeTarget(const TargetPose &target) = 0;


protected:
	QTimer timer;
	int Period;
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif