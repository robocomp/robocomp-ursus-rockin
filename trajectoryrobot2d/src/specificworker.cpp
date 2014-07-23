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
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");  ///CHECK IT CORRESPONDS TO RCIS
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/wall.xml");  ///CHECK IT CORRESPONDS TO RCIS
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/vacio.xml");  ///CHECK IT CORRESPONDS TO RCIS

	//Move Robot to Hall
	//setRobotInitialPose(800,-1000,M_PI);
	
	//Update InnerModel from robot
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, robot not found");}
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, laser not found");}
	
	innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
	
 	cleanWorld();
	
	//Set target
	//target = QVec::vec3(8000,10,-1000);
 	target = QVec::vec3(7000,10,-1500);	// dormitorio
//	target = QVec::vec3(6000,10,-9100);   //detrás de la mesa
//	target = QVec::vec3(3000,10,-8100);   //detrás del sofá	
//	target = QVec::vec3(0,22,3000);
	
	//OJO CHANGE chooseRandomPointInFreeSpace to PLAN IN APARTMENT
	
	//Draw target as red box	
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = target.x();	plane.py = 10; plane.pz = target.z();
	plane.nx = 1;plane.texture = "#990000";	plane.thickness = 150;
	plane.height = plane.width = 100;
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "world", plane);
	 
	//Planning
// 	planner = new Planner(*innerModel);									
// 	qDebug() << __FUNCTION__ << "Planning ...";
// 	planner->computePath(target, innerModel);
// 	if(planner->getPath().size() == 0)
// 		qFatal("SpecificWorker: Path NOT found. Aborting");
// 	
// 	//Init road
// 	road.setInnerModel(innerModel);
// 	road.readRoadFromList( planner->getPath() );
// 	qDebug() << __FUNCTION__ << "----- Plan obtained with elements" << road.size();	
// 	road.print();
// 	road.computeForces();
// 	
// 	//Creates and amintains the road (elastic band) adapting it to the real world using a laser device
// 	elasticband = new ElasticBand(innerModel);	
// 	qDebug() << __FUNCTION__ << "----- elasticband set";
// 	
// 	//Low level controller that drives the robot on the road by computing VAdv and VRot from the relative position wrt to the local road
// 	controller = new Controller(2);
// 	qDebug() << __FUNCTION__ << "----- controller set";
	
// 	//Localizar class
 	localizer = new Localizer(innerModel);
// 	
 	sleep(1);

	//Clon para Luis
//	innerClon = new InnerModel(*innerModel);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::computeLuis( )
{	
	printf("############################################################################\n");
	try
	{
		differentialrobot_proxy->getBaseState(bState);
	}
	catch(const Ice::Exception &ex)
	{
		cout << ex << endl;
	}
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }


	innerModel->updateTranslationValues("robot", bState.x, 10, bState.z);
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);

	QVec point = innerModel->transform("world", "robot");
	point.print("robot segun IM bueno");
// 	innerClon->updateTransformValues("robot", point.x(), point.y(), point.z(), 0, 0, 0);
// 	point = innerClon->transform("world", "robot");
// 	point.print("robot segun IM clonado");
	
	// OJO con el Inner que se le manda
	if (planner->collisionDetector(point, 0, innerModel) == true) 
		printf("colision\n");
	usleep(500000);
	
// 	localizer->localize(laserData, innerModel);
}

/**
 * @brief All architecture goes here. 
 * 
 * @return void
 */
void SpecificWorker::compute( )
{	
//  	computeLuis();
//  	return;

 	static QTime reloj = QTime::currentTime();
	static QTime reloj2 = QTime::currentTime();
	
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
		
	//innerModel->update();
	innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	//qDebug() << "bState" << bState.x << bState.z;
// 	elasticband->update( road, laserData );
// 	
// 	road.computeForces();
// 
// 	road.printRobotState( innerModel);
// 
// 	controller->update(differentialrobot_proxy, road);
// 			
// 	if(reloj.elapsed() > 2000) 
// 	{
// 		road.draw(innermodelmanager_proxy, innerModel);	
// 		reloj.restart();
// 	}
// 	
// 	if (road.isFinished() == true)
// 	{
// 		RoboCompInnerModelManager::Plane3D plane;
// 		plane.px = target.x(); plane.py = 1800;	plane.pz = target.z();	plane.nx = 1;	plane.ny = 0;	plane.nz = 0;
// 		plane.texture = "#009900";	plane.thickness = 150;	plane.height = plane.width = 100;
// 		RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "world", plane);
// 		qFatal("GOODBYE, FINISHED ROAD");
// 	}
// 	
// 	if(road.requiresReplanning == true)
// 	{
// 		qDebug("STUCK, PLANNING REQUIRED");
// 		qDebug("Planning ...");
// 		innerModel->update();
// 		bool havePlan = planner->computePath(target, innerModel);
// 	
// 		if(havePlan == false or planner->getPath().size() == 0)
// 			qFatal("NO PLAN AVAILABLE");
// 		else
// 		{
// 			//drawThinkingRobot("green");
// 			//road.readRoadFromList( planner->getPath() );
// 			road.clear();
// 			road.readRoadFromList( planner->getPath() );
// 			road.requiresReplanning = false;
// 			//elasticband->addPoints(road);  //SEND THIS TO A RESET
// 			//elasticband->adjustPoints(road);  //SEND THIS TO A RESET
// 			road.computeDistancesToNext();
// 		}
// 	}
	
	localizer->localize(laserData, innerModel, 100);
	
	qDebug() << reloj2.elapsed() << "ms"; reloj2.restart();
}


////////////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);
	timer.start(20);
	
	return true;
};

void SpecificWorker::setRobotInitialPose(float x, float z, float alpha)
{
	qDebug()<< __FUNCTION__ << "Sending robot to initial position";
	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x=x; p.y=10; p.z=z;
		p.rx=0;p.ry=alpha;p.rz=0;
		innermodelmanager_proxy->setPoseFromParent("robotInitialPose",p);
		differentialrobot_proxy->setOdometerPose(x,z,alpha);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
		qDebug() << __FUNCTION__ << QString::fromStdString(e.text) << "Error sendong robot to intial position";
		qFatal("Aborting");
	}
	usleep(500000);	
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
