/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "cuba2dnaturallandmarksI.h"

Cuba2DnaturallandmarksI::Cuba2DnaturallandmarksI( Worker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;                   //share worker mutex
	// Component initialization ...
}


Cuba2DnaturallandmarksI::~Cuba2DnaturallandmarksI()
{
	// Free component resources here
}

// Component functions, implementation


void Cuba2DnaturallandmarksI::computeFeatures(const ::RoboCompLaser::TLaserData &data, ::RoboCompCuba2Dnaturallandmarks::Features &features, const ::Ice::Current &)
{

	features = worker->computeFeatures( data );

}

::RoboCompCuba2Dnaturallandmarks::Features Cuba2DnaturallandmarksI::getFeatures(const ::Ice::Current &)
{

	return worker->getFeatures(  );

}

::RoboCompCuba2Dnaturallandmarks::Features Cuba2DnaturallandmarksI::getLocalFeatures(const ::Ice::Current &)
{

	return worker->getLocalFeatures(  );

}
