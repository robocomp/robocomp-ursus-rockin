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
//	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/wall.xml");  ///CHECK IT CORRESPONDS TO RCIS
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/vacio.xml");  ///CHECK IT CORRESPONDS TO RCIS

	// Move Robot to Hall
	//setRobotInitialPose(800, -10000, M_PI);
	
	//Update InnerModel from robot
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, robot not found");}
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, laser not found");}
	
	innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
	
 	cleanWorld();
		 
	//Planning
	plannerOMPL = new PlannerOMPL(*innerModel);			
	plannerRC = new Planner(*innerModel);
	plannerPRM = new PlannerPRM(*innerModel);
	planner = plannerPRM;
	qDebug() << __FUNCTION__ << "Planning ...";
	
	//Init road
	road.setInnerModel(innerModel);
	
	//Creates and amintains the road (elastic band) adapting it to the real world using a laser device
	elasticband = new ElasticBand(innerModel);	
	qDebug() << __FUNCTION__ << "----- elasticband set";
	
	//Low level controller that drives the robot on the road by computing VAdv and VRot from the relative position wrt to the local road
	controller = new Controller(2);
	qDebug() << __FUNCTION__ << "----- controller set";
	
// 	//Localizer stuff
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
// 	printf("############################################################################\n");
// 	try
// 	{
// 		differentialrobot_proxy->getBaseState(bState);
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		cout << ex << endl;
// 	}
// 	try { laserData = laser_proxy->getLaserData(); }
// 	catch(const Ice::Exception &ex) { cout << ex << endl; }
// 
// 
// 	innerModel->updateTranslationValues("robot", bState.x, 10, bState.z);
// 	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
// 
// 	QVec point = innerModel->transform("world", "robot");
// 	point.print("robot segun IM bueno");
// // 	innerClon->updateTransformValues("robot", point.x(), point.y(), point.z(), 0, 0, 0);
// // 	point = innerClon->transform("world", "robot");
// // 	point.print("robot segun IM clonado");
// 	
// 	// OJO con el Inner que se le manda
// 	if (planner->collisionDetector(point, 0, innerModel) == true) 
// 		printf("colision\n");
// 	usleep(500000);
	
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
	
	updateInnerModel(innerModel);

	if ( currentTarget.isActive() )
	{
		computePlan(innerModel);
	
		elasticband->update( road, laserData );
		
		road.computeForces();

		road.printRobotState( innerModel);

		controller->update(differentialrobot_proxy, road);
		
		if (road.isFinished() == true)
		{
			drawGreenBoxOnTarget( currentTarget.getTranslation() );
			currentTarget.reset();
			road.reset();
			compState.elapsedTime = taskReloj.elapsed();
		}
		
		if(road.requiresReplanning == true)
		{
			qDebug() << __FUNCTION__ << "STUCK, PLANNING REQUIRED";
			computePlan(innerModel);
		}
		
	//	localizer->localize(laserData, innerModel, 20);
	}
	
	//Draw in RCIS
	if(reloj.elapsed() > 2000) 
	{
		qDebug() << __FUNCTION__ << "Elapsed time: " << reloj2.elapsed();
		road.draw(innermodelmanager_proxy, innerModel);	
		reloj.restart();
	}
	reloj2.restart();
}
	
////////////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);
	timer.start(20);
	
	return true;
};

bool SpecificWorker::computePlan( InnerModel *inner)
{
	if( currentTarget.isWithoutPlan() == false) 
		return true;
		
	QTime reloj = QTime::currentTime();
	qDebug() << __FUNCTION__ << "Computing plan... ";
	compState.planning = true;
	
	cleanWorld();
	updateInnerModel(inner);
	
	planner->computePath(currentTarget.getTranslation(), inner);
	
	qFatal("fary");
	
	if(planner->getPath().size() == 0)
	{
		qDebug() << __FUNCTION__ << "SpecificWorker: Path NOT found. Aborting";
		currentTarget.reset();
		return false;
	}
	else
	{
		currentTarget.setWithoutPlan( false );
	
		//Init road
		road.reset();
		road.readRoadFromList( planner->getPath() );
		road.requiresReplanning = false;
		road.computeDistancesToNext();
		qDebug() << __FUNCTION__ << "----- Plan obtained with elements" << road.size();	
		road.print();
		road.computeForces();  //NOT SURE IF NEEDED HERE
		
		qDebug() << __FUNCTION__ << "Plan obtained after " << reloj.elapsed() << "ms. Plan length: " << planner->getPath().size();
		
		compState.planningTime = reloj.elapsed();
		compState.planning = false;
	}
	return true;
}

void SpecificWorker::updateInnerModel(InnerModel *inner)
{
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	
	inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
}

void SpecificWorker::setRobotInitialPose(float x, float z, float alpha)
{
	qDebug()<< __FUNCTION__ << "Sending robot to initial position";
	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x =  0;
		p.y =  0;
		p.z =  0;
		p.rx = 0;
		p.ry = 0;
		p.rz = 0;
		innermodelmanager_proxy->setPoseFromParent("initialRobotPose", p);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
		qDebug() << __FUNCTION__ << QString::fromStdString(e.text) << "Error setting initialRobotPose";
		qFatal("Aborting");
	}

	usleep(125000);	

	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x =  x;
		p.y =  0;
		p.z =  z;
		p.rx = 0;
		p.ry = alpha;
		p.rz = 0;
		innermodelmanager_proxy->setPoseFromParent("robot", p);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
		qDebug() << __FUNCTION__ << QString::fromStdString(e.text) << "Error setting robot pose";
		qFatal("Aborting");
	}
	usleep(125000);	

	try
	{
		differentialrobot_proxy->setOdometerPose(x, z, alpha);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
		qDebug() << __FUNCTION__ << QString::fromStdString(e.text) << "Error setting robot odometer";
		qFatal("Aborting");
	}

	usleep(125000);	
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

void SpecificWorker::drawTarget(const QVec &target)
{
	//Draw target as red box	
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = target.x();	plane.py = 10; plane.pz = target.z();
	plane.nx = 1;plane.texture = "#990000";	plane.thickness = 150;
	plane.height = plane.width = 100;
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "world", plane);
		
}

void SpecificWorker::drawGreenBoxOnTarget(const QVec& target)
{
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = target.x(); plane.py = 1800;	plane.pz = target.z();	plane.nx = 1;	plane.ny = 0;	plane.nz = 0;
	plane.texture = "#009900";	plane.thickness = 150;	plane.height = plane.width = 100;
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "world", plane);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////777
////    SERVANTS
//////////////////////////////////////////////////////////////////////////////////////////////////////777

void SpecificWorker::go(const TargetPose& target)
{
	currentTarget.setActive(true);
	currentTarget.setTranslation( QVec::vec3(target.x, target.y, target.z) );
	drawTarget( QVec::vec3(target.x,target.y,target.z));
	taskReloj.restart();
	qDebug() << __FUNCTION__ << "Curent target received:" << currentTarget.getTranslation();
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::getState()
{
	return compState;
}

void SpecificWorker::stop()
{
	currentTarget.reset();
	controller->stopTheRobot(differentialrobot_proxy);

}