/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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
#ifndef CUBAFEATURESI_H
#define CUBAFEATURESI_H

// QT includes
#include <QtCore>

// ICE includes
#include <Ice/Ice.h>
#include <Cuba2Dnaturallandmarks.h>

// User includes...
#include <config.h>
#include "worker.h"

using namespace RoboCompCuba2Dnaturallandmarks;

class Cuba2DnaturallandmarksI : public QObject , public virtual RoboCompCuba2Dnaturallandmarks::Cuba2Dnaturallandmarks
{
Q_OBJECT
public:
	Cuba2DnaturallandmarksI( Worker *_worker, QObject *parent = 0 );
	~Cuba2DnaturallandmarksI();

 	void computeFeatures(const ::RoboCompLaser::TLaserData&, ::RoboCompCuba2Dnaturallandmarks::Features&, const ::Ice::Current& = ::Ice::Current());
	::RoboCompCuba2Dnaturallandmarks::Features getFeatures( const ::Ice::Current& = ::Ice::Current());
	::RoboCompCuba2Dnaturallandmarks::Features getLocalFeatures( const ::Ice::Current& = ::Ice::Current());

	QMutex *mutex;
private:

	Worker *worker;
public slots:


};

#endif

