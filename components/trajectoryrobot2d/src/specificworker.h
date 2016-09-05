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
#include "plannerompl.h"
#include "plannerprm.h"
//#include "plannerthunder.h"
#include "innerviewer.h"
#include "currenttarget.h"
#include "sampler.h"
#include "graphdraw.h"
#include "waypointsdraw.h"

//#include "ParabolicPathSmooth/smoother.h"

// #ifdef USE_QTGUI
// 	#include <osgviewer/osgview.h>
// 	#include <innermodel/innermodelviewer.h>
// 	#include <innermodeldraw.h>
// #endif

/**
       \brief Code to control de robot along a trajectory defined by a set of points
       @author authorname
*/


/**
 * @brief ...Auxiliary class to keep the state of the algorithm and make it accesible to the middleware
 * 
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
		void setDescription(const std::string &desc_){ description = desc_;}
		long getElapsedTime(){ QMutexLocker l(&m); return elapsedTime;};
		long getEstimatedtime(){ QMutexLocker l(&m); return estimatedTime;};
		long getPlanningTime(){ QMutexLocker l(&m); return planningTime;};
		std::string getState(){ QMutexLocker l(&m); return state;};
		RoboCompTrajectoryRobot2D::NavState toMiddleware( const RoboCompOmniRobot::TBaseState &bState, const WayPoints &road)
		{
			QMutexLocker l(&m);
			RoboCompTrajectoryRobot2D::NavState n;
			n.state = state;
			n.description = description;
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
		long elapsedTime = 0;
		long estimatedTime = 0;
		long planningTime = 0;
		std::string state, description;
};


class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx, QWidget *parent = 0);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		////////////////////////////////////////
		//SERVANTS ATTENDING EXTERNAL INTERFACE
		////////////////////////////////////////
		RoboCompTrajectoryRobot2D::NavState	getState();
		float goBackwards(const TargetPose &target);
		void stop();
		void setHeadingTo(const TargetPose& target);
		float goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold);
		float changeTarget(const TargetPose &target);
		float go(const TargetPose &target);
			
	public slots:
		void	compute(); 	
		
	private:
		RoboCompOmniRobot::TBaseState bState;
		TrajectoryState tState;	// object coding changing state for external interface
		RoboCompCommonBehavior::ParameterList params;
		RoboCompLaser::TLaserData laserData;
		CurrentTarget currentTarget;
		CurrentTarget currentTargetAnt, currentTargetBack;
		InnerModel *innerModel;
		
		//Sampler of robot's freespace
		Sampler sampler;
		QList<QRectF> innerRegions;
		QRectF outerRegion;
	
		// Road structure
		WayPoints road;
		
		// Controller
		Controller *controller;
		
		// road-world interaction through a force field (laser)
		ElasticBand *elasticband;
		
		PlannerOMPL *plannerOMPL;
		
		// Access to OMPL planners
		PlannerPRM plannerPRM;
		
		//GraphDraw and RoadDrar
		#ifdef USE_QTGUI
			GraphDraw graphdraw;
			WaypointsDraw waypointsRoad;
		#endif
	
		//Threaded InnerModelViewer
		#ifdef USE_QTGUI
			InnerViewer *viewer;
		#endif
			
		//Timers to control real time events
		QTime relojForInputRateControl;  //used to limit input frequency
		QTime taskReloj;  //Measures duration of commands


		////////////////////////////////////////////////////////////////////////
		//Commands corresponding to servant methods, but running on local thread
		/////////////////////////////////////////////////////////////////////////
		bool gotoCommand(InnerModel* innerModel, CurrentTarget& target, TrajectoryState &state, WayPoints& myRoad, RoboCompLaser::TLaserData &lData);
		bool setHeadingCommand(InnerModel* innerModel, float alfa, CurrentTarget& target, TrajectoryState& state, WayPoints& myRoad);
		bool stopCommand( CurrentTarget& target, WayPoints& myRoad, TrajectoryState &state);
		bool changeTargetCommand(InnerModel* innerModel, CurrentTarget& target,  TrajectoryState &stat, WayPoints& myRoad);
		bool goBackwardsCommand(InnerModel* innerModel, CurrentTarget& current,CurrentTarget &currentT, TrajectoryState& state, WayPoints& myRoad);
		bool learnCommand(CurrentTarget &target,const WayPoints& myRoad);
		
		////////////////////////////////////////////////////////////////////////
		//Auxiliary methods
		/////////////////////////////////////////////////////////////////////////
		void tryingToConnect();
		bool updateInnerModel(InnerModel* inner, TrajectoryState &state);
		bool insertObstacle();
		bool removeNode(const QString &item);
		void addPlane(QString item, QString parent, QString path, QVec scale, QVec t, QVec r);
		void readRoadFromFile(string name, WayPoints *road);
		void setRobotInitialPose(float x, float z, float alpha);
		void drawTarget(const QVec &target);
		void drawGreenBoxOnTarget(const QVec &target);
		void printNumberOfElementsInIMV();
		float angmMPI(float angle);

		void mapBasedTarget(const NavigationParameterMap  &parameters);

};



#endif
