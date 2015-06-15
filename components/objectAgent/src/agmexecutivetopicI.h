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
#ifndef AGMEXECUTIVETOPICI_H
#define AGMEXECUTIVETOPICI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <AGMExecutive.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAGMExecutive;

class AGMExecutiveTopicI : public QObject , public virtual RoboCompAGMExecutive::AGMExecutiveTopic
{
Q_OBJECT
public:
	AGMExecutiveTopicI( GenericWorker *_worker, QObject *parent = 0 );
	~AGMExecutiveTopicI();
<<<<<<< HEAD
	void  structuralChange(const RoboCompAGMWorldModel::Event& modification, const Ice::Current& = Ice::Current());
void symbolUpdated(const RoboCompAGMWorldModel::Node& modification, const Ice::Current& = Ice::Current());
void edgeUpdated(const RoboCompAGMWorldModel::Edge& modification, const Ice::Current& = Ice::Current());
=======
	void structuralChange(const RoboCompAGMWorldModel::Event&, const Ice::Current& = Ice::Current());
	void symbolUpdated(   const RoboCompAGMWorldModel::Node&,  const Ice::Current& = Ice::Current());
	void edgeUpdated(     const RoboCompAGMWorldModel::Edge&,  const Ice::Current& = Ice::Current());

>>>>>>> 78711c4beda34b828fdbf4b621aa5a357de78d07


	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif