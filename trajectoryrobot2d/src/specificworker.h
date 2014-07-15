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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <iostream>
#include <fstream>
//#include <boost/concept_check.hpp>
//#include "pointstoroad.h"
#include "controller.h"
#include "elasticband.h"
#include "waypoints.h"
#include "planner.h"
#include "forcefield.h"

//#include "ParabolicPathSmooth/smoother.h"

/**
       \brief Algorithm to control de robot along a trajectory defined by a set of points
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QWidget *parent = 0);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
public slots:
 	void compute(); 	
	void computeLuis( );
	
private:
	RoboCompDifferentialRobot::TBaseState bState;
	TLaserData datos;
	RoboCompCommonBehavior::ParameterList params;
	InnerModel *innerModel, *innerClon;
	RoboCompLaser::TLaserData laserData;
	QVec target;
 	
	QVec P;
	WayPoints road;
	Controller *controller;
	ElasticBand *elasticband;
	Planner *planner;
	ForceField *forcefield;
	
	//Smoother smoother;
	
	void readRoadFromFile(string name, WayPoints *road);
	void cleanWorld();
	void moveBoxes();
	void drawThinkingRobot(const QString &color);
	void sendRobotHome();
};

#endif