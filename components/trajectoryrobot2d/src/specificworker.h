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
#include "localizer.h"
#include "plannerompl.h"
#include "plannerprm.h"
#include "currenttarget.h"

//#include "ParabolicPathSmooth/smoother.h"

/**
       \brief Algorithm to control de robot along a trajectory defined by a set of points
       @author authorname
*/
class TrajectoryState
{
	public:
		TrajectoryState() { state = "IDLE";}
		~TrajectoryState() {};
		void setElapsedTime(long t){ QMutexLocker l(&m); elapsedTime = t;};
		void setEstimatedTime(long t){ QMutexLocker l(&m); estimatedTime = t;};
		void setPlanningTime(long t){ QMutexLocker l(&m); planningTime = t;};
		void setState(const std::string &state_){ state = state_;}
		long getElapsedTime(){ QMutexLocker l(&m); return elapsedTime;};
		long getEstimatedtime(){ QMutexLocker l(&m); return estimatedTime;};
		long getPlanningTime(){ QMutexLocker l(&m); return planningTime;};
		std::string getState(){ QMutexLocker l(&m); return state;};
		RoboCompTrajectoryRobot2D::NavState toMiddleware( const RoboCompOmniRobot::TBaseState &bState, const WayPoints &road)
		{
			QMutexLocker l(&m);
			RoboCompTrajectoryRobot2D::NavState n;
			n.state = state;
			n.elapsedTime = elapsedTime;
			n.planningTime = planningTime;
			n.estimatedTime = estimatedTime;
			//////////// MORE DATA CAN GO HERE LIKE robot position, dist to targt, current road, etc
			n.x = bState.x;
			n.z = bState.z;
			n.ang = bState.alpha;
			n.advV = bState.advVz;
			n.rotV = bState.rotV;
			n.distanceToTarget = road.getRobotDistanceToTarget();
			return n;
		};
	private:
		QMutex m;
		long elapsedTime;
		long estimatedTime;
		long planningTime;
		std::string state;
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QWidget *parent = 0);
	~SpecificWorker();
			
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void go(const TargetPose& target);
	void setHeadingTo(const TargetPose& target);
	void changeTarget(const TargetPose& target);
	void stop();
	RoboCompTrajectoryRobot2D::NavState	getState();
	void goBackwards(const TargetPose& target);
		
public slots:
 	void	compute(); 	
	
private:
	//RoboCompDifferentialRobot::TBaseState 	bState;
	RoboCompOmniRobot::TBaseState bState;
	//RoboCompTrajectoryRobot2D::NavState compState;
	TrajectoryState tState;
	RoboCompCommonBehavior::ParameterList params;
	RoboCompLaser::TLaserData laserData;
	RoboCompLaser::TLaserData datos;

	CurrentTarget currentTarget;
	
	InnerModel *innerModel, *innerClon;
	QVec target;
	QTime taskReloj;
 	
	QVec P;
	WayPoints road;
	Controller *controller;
	ElasticBand *elasticband;
	PlannerOMPL *plannerOMPL;
	PlannerPRM *plannerPRM, *planner;
	Localizer *localizer;
	
	//Commands correspondign to servant methods, but running on local thread
	bool gotoCommand(InnerModel* innerModel, CurrentTarget& target, TrajectoryState &state, WayPoints& myRoad, RoboCompLaser::TLaserData &lData);
	bool setHeadingCommand(InnerModel* innerModel, float alfa, CurrentTarget& target, TrajectoryState& state, WayPoints& myRoad);
	bool stopCommand( CurrentTarget& target, WayPoints& myRoad, TrajectoryState &state);
	bool changeTargetCommand(InnerModel* innerModel, CurrentTarget& target,  TrajectoryState &stat, WayPoints& myRoad);
	bool goBackwardsCommand(InnerModel *innerModel, const QVec &target, CurrentTarget &current, TrajectoryState &state, WayPoints &myRoad);
	bool updateInnerModel(InnerModel* inner, TrajectoryState &state);
	
	//Smoother smoother;
	void readRoadFromFile(string name, WayPoints *road);
	void setRobotInitialPose(float x, float z, float alpha);
	bool targetHasAPlan(InnerModel* inner, CurrentTarget& target, TrajectoryState &state, WayPoints &myRoad);
	void drawTarget(const QVec &target);
	void drawGreenBoxOnTarget(const QVec &target);
	void printNumberOfElementsInRCIS();
	void calcularModuloFloat(QVec &angles, float mod);
	float angmMPI(float angle);
	
};


#endif