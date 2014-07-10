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
 
 #include "specificworker.h"
#include <qt4/QtCore/qdatetime.h>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QWidget *parent) : GenericWorker(mprx)
{
	this->params = params;
	
	//innerModel = new InnerModel("/home/robocomp/robocomp/Files/InnerModel/betaWorld.xml");  ///CHECK IT CORRESPONDS TO RCIS

	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");  ///CHECK IT CORRESPONDS TO RCIS
//	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockin.xml");  ///CHECK IT CORRESPONDS TO RCIS
	innerModel->setUpdateTranslationPointers("robot", &(bState.x), NULL, &(bState.z));
	innerModel->setUpdateRotationPointers("robot", NULL, &(bState.alpha), NULL);
	
	//moveBoxes();
 
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	
	innerModel->update();
	cleanWorld();
	//sendRobotHome();
	
	//Set target
	//target = QVec::vec3(8000,10,-1000);
	//target = QVec::vec3(800,10,-3000);
//	target = QVec::vec3(6000,10,-8100);
	target = QVec::vec3(5800,10,-5000);
	
	
	
	//Draw target as red box	
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = target.x();
	plane.py = 1800;
	plane.pz = target.z();
	plane.nx = 1;
	plane.texture = "#990000";
	plane.thickness = 150;
	plane.height = plane.width = 100;
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "floor", plane);
	
	//qFatal("fary");
	
	//Plan 
	planner = new Planner(*innerModel);									////////PLANNER SHOOULD TAKE WAYPOINTS structure
	qDebug() << __FUNCTION__ << "Planning ...";
	//	drawThinkingRobot("red");
	planner->computePath(target, innerModel);
	if(planner->getPath().size() == 0)
		qFatal("SpecificWorker: Path NOT found. Aborting");
	
	//planner->drawTree(innermodelmanager_proxy);
	//road.readRoadFromList( planner->getPath() );
	road = planner->getPath();
	//road = planner->smoothRoad(road);
	road.print();
	
	qDebug() << __FUNCTION__ << "----- Plan obtained with elements" << road.size();	
	
	//Creates and amintains the road (elastic band) adapting it to the real world using a laser device
	elasticband = new ElasticBand(innerModel);
	elasticband->addPoints(road);
	//elasticband->adjustPoints(road);
	
	qDebug() << __FUNCTION__ << "----- elasticband set";
	
	//Converts the road and the current robot position into a set of four parameters that are used by the Controller to physically drive the robot
	pointstoroad = new PointsToRoad(innerModel, innermodelmanager_proxy);
	pointstoroad->setRoad(road);
	
	qDebug() << __FUNCTION__ << "----- pointstoroad set";
	
	//Low level controller that drives the robot on the road by computing VAdv and VRot from the relative position wrt to the local road
	controller = new Controller(2);

	qDebug() << __FUNCTION__ << "----- controller set";
	
	sleep(1);
	
//	road.draw(innermodelmanager_proxy, innerModel);
		
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::compute( )
{	
	try{	differentialrobot_proxy->getBaseState(bState);  }  
	catch(const Ice::Exception &ex){ cout << ex << endl;};
	try{	laserData = laser_proxy->getLaserData();}  
	catch(const Ice::Exception &ex){ cout << ex << endl;};
		
	innerModel->update();
		
	elasticband->update( road, laserData );
	
	pointstoroad->update(road);
	
    controller->update(differentialrobot_proxy, road);
	
	road.draw(innermodelmanager_proxy, innerModel);	
	
	if(road.finish == true)
	{
		//Draw target as red box	
		RoboCompInnerModelManager::Plane3D plane;
		plane.px = target.x(); plane.py = 1800; plane.pz = target.z(); plane.nx = 1; plane.texture = "#009900"; plane.thickness = 150; plane.height = plane.width = 100;
		RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "floor", plane);
		qFatal("GOODBYE, FINISHED ROAD");
	}
	
	if(road.requiresReplanning == true)
	{
		qDebug("STUCK, PLANNING REQUIRED");
		qDebug("Planning ...");
		//drawThinkingRobot("red");
		innerModel->update();
		bool havePlan = planner->computePath( target, innerModel );
	
		if(havePlan == false or planner->getPath().size() == 0)
			qFatal("NO PLAN AVAILABLE");
		else
		{
			//drawThinkingRobot("green");
			//road.readRoadFromList( planner->getPath() );
			road = planner->getPath();
			road.requiresReplanning = false;
			elasticband->addPoints(road);  //SEND THIS TO A RESET
			//elasticband->adjustPoints(road);  //SEND THIS TO A RESET
			road.computeDistancesToNext();
		}
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);
	timer.start(100);
	
	return true;
};

void SpecificWorker::sendRobotHome()
{
	qDebug()<< __FUNCTION__ << "Sending robot home";
	try
	{
		RoboCompInnerModelManager::Pose3D pose;
		pose.x= 0; pose.y=0; pose.z=0;pose.rx=0; pose.ry=0; pose.rz=0;
		innermodelmanager_proxy->setPoseFromParent("robot", pose);
	} 
	catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	{ std::cout << e << std::endl; }
}

void SpecificWorker::cleanWorld()
{
	qDebug() << __FUNCTION__ << "SpecificWorker::CleaningWorld()";
	//RcisDraw::removeObject(innermodelmanager_proxy, "nose");
	for (int i = 0 ; i < 80; i++) 
	{
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_line"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_lineA"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_mesh"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_point"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_mesh"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_line"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i)));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i)));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("t_" + QString::number(i)));
	}
}

void SpecificWorker::drawThinkingRobot(const QString &color)
{
	RoboCompInnerModelManager::Plane3D p;
	p.px = 0;
	p.py = 1810;
	p.pz = -200;
	p.nx = 0;
	p.ny = 1;
	p.nz = 0;
	p.height = 250;
	p.width = 250;
	p.thickness = 20;
	if(color == "red")
		p.texture = "#FF0000";
	else if (color == "green")
		p.texture = "#00FF00";
		
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "redHat", "base",p);
	
}
