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
#include "controller.h"
#include "elasticband.h"
#include "waypoints.h"
//#include "forcefield.h"
#include "localizer.h"
#include "plannerompl.h"
#include "plannerprm.h"
#include "currenttarget.h"

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
	void go(const TargetPose& target);
	void setHeadingTo(const TargetPose& target);
	RoboCompTrajectoryRobot2D::NavState getState();
	void stop();
	void sendData(const RoboCompJoystickAdapter::TData& data);
	
public slots:
 	void compute(); 	
	void computeLuis( );
	
private:
	
	CurrentTarget currentTarget;
	RoboCompDifferentialRobot::TBaseState bState;
	TLaserData datos;
	RoboCompCommonBehavior::ParameterList params;
	InnerModel *innerModel, *innerClon;
	RoboCompLaser::TLaserData laserData;
	QVec target;
	RoboCompTrajectoryRobot2D::NavState compState;
	QTime taskReloj;
 	
	QVec P;
	WayPoints road;
	Controller *controller;
	ElasticBand *elasticband;
	PlannerOMPL *plannerOMPL;
	PlannerPRM *plannerPRM, *planner;
	//ForceField *forcefield;
	Localizer *localizer;
	
	//Smoother smoother;
	
	void readRoadFromFile(string name, WayPoints *road);
	void setRobotInitialPose(float x, float z, float alpha);
	bool updateInnerModel(InnerModel* inner);
	bool targetHasAPlan(InnerModel* inner);
	void drawTarget(const QVec &target);
	void drawGreenBoxOnTarget(const QVec &target);
	void printNumberOfElementsInRCIS();
	bool gotoCommand(InnerModel *innerModel);
	bool setHeadingCommand(InnerModel *innerModel, float alfa);
	bool avoidanceControl(InnerModel& innerModel, WayPoints& road, const TLaserData& laserData, float& vadvance, float& vrot);
	std::vector<float> computeRobotOffsets(InnerModel& innerModel, const RoboCompLaser::TLaserData &laserData);
	std::vector<float> baseOffsets;
	bool robotLaserCollision(const QVec& p1, const QVec& p2, const QVec& p3, const QVec& p);
	void filter(float &vadvance, float &vrot);
	QVec repulsionVector;
};

#endif