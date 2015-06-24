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
#ifndef REFLEXXES_H
#define REFLEXXES_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <Reflexxes.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompReflexxes;

class ReflexxesI : public QObject , public virtual RoboCompReflexxes::Reflexxes
{
Q_OBJECT
public:
	ReflexxesI( GenericWorker *_worker, QObject *parent = 0 );
	~ReflexxesI();
	
	bool getStatePosition(const MotorAngleList  &anglesOfMotors, const Ice::Current&);
	void setJointPosition(const MotorAngleList  &newAnglesOfMotors, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
