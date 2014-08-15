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
//	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/vacio.xml");  ///CHECK IT CORRESPONDS TO RCIS

	// Move Robot to Hall
	//setRobotInitialPose(800, -10000, M_PI);
	
	//Update InnerModel from robot
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, robot not found");}
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, laser not found");}
	
	innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
		 
//	setRobotInitialPose(800, -1500, M_PI);
	baseOffsets = computeRobotOffsets(*innerModel, laserData);
	
	//Planning
	plannerOMPL = new PlannerOMPL(*innerModel);			
	plannerPRM = new PlannerPRM(*innerModel, 100, 30);
	planner = plannerPRM;
	
	//planner->drawGraph(innermodelmanager_proxy);

	qDebug() << "----------------inserting" ;
	
	//planner->drawGraph(innermodelmanager_proxy);
		
	qDebug() << __FUNCTION__ << "----- planner set";
	
	//Init road
	road.setInnerModel(innerModel);
	
	//Creates and amintains the road (elastic band) adapting it to the real world using a laser device
	elasticband = new ElasticBand(innerModel);	
	qDebug() << __FUNCTION__ << "----- elasticband set";
	
	//Low level controller that drives the robot on the road by computing VAdv and VRot from the relative position wrt to the local road
	controller = new Controller(2);
	qDebug() << __FUNCTION__ << "----- controller set";
	
// 	//Localizer stuff
 //	localizer = new Localizer(innerModel);
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

	float a;
	avoidanceControl(*innerModel, road, laserData, a, a);
	
 	static QTime reloj = QTime::currentTime();
	static QTime reloj2 = QTime::currentTime();
	
	if ( updateInnerModel(innerModel) )
	{
		if( currentTarget.isActive() and currentTarget.command == CurrentTarget::Command::GOTO)
			gotoCommand(innerModel);
		
		else if( currentTarget.isActive() and currentTarget.command == CurrentTarget::Command::SETHEADING)
			setHeadingCommand(innerModel, currentTarget.getRotation().y());
	}	
	else //LOST connection to robot
	{
		currentTarget.reset();
		road.reset();
		compState.state = "DISCONNECTED";
	}

	if(reloj.elapsed() > 2000) 
	{
		qDebug() << __FUNCTION__ << "Elapsed time: " << reloj2.elapsed();
		if( reloj2.elapsed() < 100 )
		{
			road.clearDraw(innermodelmanager_proxy);
			//try {	
			road.draw(innermodelmanager_proxy, innerModel);
			//
			//	planner->drawGraph(innermodelmanager_proxy);
		}
		//printNumberOfElementsInRCIS();
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

/////////////////////////////////////////////////////////

bool SpecificWorker::gotoCommand(InnerModel *innerModel)
{
	if( targetHasAPlan(innerModel))
	{
		elasticband->update( road, laserData, currentTarget);
		
		road.computeForces();
	
		road.printRobotState( innerModel, currentTarget);

		controller->update(*innerModel, laserData, differentialrobot_proxy, road);
		
		if (road.isFinished() == true)
		{		
			//if( currentTarget.hasRotation )
			drawGreenBoxOnTarget( currentTarget.getTranslation() );
			currentTarget.print();
			currentTarget.reset();
			planner->learnPath( road.backList );
			road.reset();
			road.endRoad();
			compState.elapsedTime = taskReloj.elapsed();
			//planner->drawGraph(innermodelmanager_proxy);
			compState.state = "IDLE";
		}
		
		if(road.requiresReplanning == true)
		{
			//qDebug() << __FUNCTION__ << "STUCK, PLANNING REQUIRED";
			//computePlan(innerModel);
		}
		
		compState.planningTime = road.getETA();
	//	localizer->localize(laserData, innerModel, 20);
	}
	return true;
}

bool SpecificWorker::setHeadingCommand(InnerModel* innerModel, float alfa)
{
	
	return true;
}

/////////////////////////////////////////////////////////

bool SpecificWorker::targetHasAPlan( InnerModel *inner)
{
	if( currentTarget.isWithoutPlan() == false ) 
		return true;
		
	QTime reloj = QTime::currentTime();
	qDebug() << __FUNCTION__ << "Computing plan... ";
	
	if (updateInnerModel(inner))
	{	
		compState.state = "PLANNING";
		if ( planner->computePath(currentTarget.getTranslation(), inner) == false)
		{
			qDebug() << __FUNCTION__ << "SpecificWorker: Path NOT found. Resetting";
			currentTarget.reset();
			return false;
		}
		qDebug() << __FUNCTION__ << "Plan obtained after " << reloj.elapsed() << "ms. Plan length: " << planner->getPath().size();
		
		// take inner to current values
		updateInnerModel(inner);
		currentTarget.setWithoutPlan( false );
		currentTarget.print();
		//planner->cleanGraph(innermodelmanager_proxy);
		compState.state = "EXECUTING";	
		//Init road
		road.reset();
		road.readRoadFromList( planner->getPath() );
		//road.last() = currentTarget.getRotation();
		road.requiresReplanning = false;
		road.computeDistancesToNext();
		road.print();
		road.computeForces();  //NOT SURE IF NEEDED HERE
		road.startRoad();
		compState.planningTime = road.getETA();
		//compState.planningTime = reloj.elapsed();
	
		return true;
	}
	else 
		return false;
}

bool SpecificWorker::updateInnerModel(InnerModel *inner)
{
	try 
	{ 
		differentialrobot_proxy->getBaseState(bState); 
		inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	/*	QMat r1q = innerModel->getRotationMatrixTo("world", "robot");	
		qDebug() << __FUNCTION__ << "robot state" << bState.x << bState.z << bState.alpha << r1q.extractAnglesR_min().y();
	*/	
	}
	catch(const Ice::Exception &ex) { cout << ex << endl; return false; }
	try 
	{ 
		laserData = laser_proxy->getLaserData(); 
	}
	catch(const Ice::Exception &ex) { cout << ex << endl; return false; }
	return true;
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
	}
	usleep(125000);	

	try
	{
		differentialrobot_proxy->setOdometerPose(x, z, alpha);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
		qDebug() << __FUNCTION__ << QString::fromStdString(e.text) << "Error setting robot odometer";
	}

	usleep(125000);	
}

// void SpecificWorker::cleanWorld()  ///CAMBIAR ESTO PARA QUE TODO CUELGUE DE "MARCAS" y de pueda borrar marcas de golpe.
// {
// 	qDebug() << __FUNCTION__ << "SpecificWorker::CleaningWorld()";
// 	//RcisDraw::removeObject(innermodelmanager_proxy, "nose");
// 	for (int i = 0 ; i < 150; i++) 
// 	{
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_line"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_lineA"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_mesh"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_point"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_mesh"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_line"));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i)));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i)));
// 		RcisDraw::removeObject(innermodelmanager_proxy, QString("t_" + QString::number(i)));
// 	}
// }

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

void SpecificWorker::printNumberOfElementsInRCIS()
{
	try
	{	RoboCompInnerModelManager::NodeInformationSequence ni;
		innermodelmanager_proxy->getAllNodeInformation(ni);
		qDebug() << __FUNCTION__ << "..............Number of elements in: " << ni.size();
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////777
////    SERVANTS
//////////////////////////////////////////////////////////////////////////////////////////////////////777

void SpecificWorker::go(const TargetPose& target)
{
	currentTarget.setActive(true);
	currentTarget.setTranslation( QVec::vec3(target.x, target.y, target.z) );
	currentTarget.setRotation( QVec::vec3(target.rx, target.ry, target.rz) );
	currentTarget.command = CurrentTarget::Command::GOTO;
	if( target.onlyRot == true) 
		currentTarget.doRotation = true;
	drawTarget( QVec::vec3(target.x,target.y,target.z));
	taskReloj.restart();
	qDebug() << __FUNCTION__ << "GO command received, with target" << currentTarget.getTranslation() << currentTarget.getRotation();
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::getState()
{
	return compState;
}

void SpecificWorker::stop()
{
	road.setFinished(true);		//make threadsafe
	controller->stopTheRobot(differentialrobot_proxy);
	currentTarget.command = CurrentTarget::Command::STOP;
	qDebug() << __FUNCTION__ << "STOP command received";
}

/**
 * @brief Integrator to orient the robot making an alfa angle with the world's Z axis
 * 
 * @param alfa ...
 * @return void
 */
void SpecificWorker::setHeadingTo(const TargetPose& target)
{
	currentTarget.command = CurrentTarget::Command::SETHEADING;	
	qDebug() << __FUNCTION__ << "SETHEADING command received";
}

/////////////////////////////////////////////////////7
///  Subcription to JoyStick
/////////////////////////////////////////////////////

/**
 * @brief Data from JoyStick. Axes values come in (-1,1) range
 * 
 * @param data ...
 * @return void
 */
void SpecificWorker::sendData(const RoboCompJoystickAdapter::TData& data)
{
	//qDebug() << __FUNCTION__ << "Data from Joy";
	try 
	{	
		QList<QPair<QPointF,QPointF> > intervals;
		intervals.append(QPair<QPointF,QPointF>(QPointF(-1,1),QPointF(-600,600))); 
	
		//qDebug() << __FUNCTION__ << intervals << "X" << X;
		QMat m = QMat::afinTransformFromIntervals( intervals );
		float vadvance = (m * QVec::vec2(data.axes[data.dirAxisIndex].value, 1))[0];
		float vrot = data.axes[data.velAxisIndex].value;
		//filter(vadvance, vrot);
		differentialrobot_proxy->setSpeedBase( vadvance, vrot);		
	} 
   	catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl; }	
}

void SpecificWorker::filter(float& vadvance, float& vrot)
{	
	if( repulsionVector != QVec::zeros(3))
 	{
 		qDebug() << "---COLLISION!!!!!!! at direction " << repulsionVector; 
 		vadvance = -fabs(vadvance);
 	}
	
}


bool SpecificWorker::avoidanceControl(InnerModel& innerModel, WayPoints& road, const RoboCompLaser::TLaserData& laserData, float& vadvance, float& vrot)
{
	if( bState.advV == 0 and bState.rotV == 0 )
		return false;
	
	//integrate speed to obtain base next position using Borenstein's approximation
	float deltaT = 500 / 1000.f;
	float xN = bState.x + (bState.advV * deltaT) * sin(bState.rotV * deltaT);  //sgs to ms
	float zN = bState.z + (bState.advV * deltaT) * cos(bState.rotV * deltaT);
	float angN = bState.alpha + (bState.rotV * deltaT);
// 	qDebug() << __FUNCTION__ << bState.x << bState.z << bState.alpha << deltaT << bState.advV << bState.rotV;
// 	qDebug() << __FUNCTION__ << xN << zN << angN;
// 	
	//now check if there will be collision in the future position using current laserData check if collides
	innerModel.updateTransformValues("robot", xN, 0, zN, 0, angN, 0);
	QVec p1 = innerModel.transform("world", QVec::vec3(200,0,200), "robot" );
	QVec p2 = innerModel.transform("world", QVec::vec3(200,0,-200), "robot" );
	QVec p3 = innerModel.transform("world", QVec::vec3(-200,0,200), "robot" );
	innerModel.updateTransformValues("robot", bState.x, 10, bState.z, 0, bState.alpha, 0);
	
	QVec p(3,0.f);
	//QVec sum(3,0.f);
	int k=0;
	repulsionVector = QVec::zeros(3);
	for(auto i : laserData)
	{
		p = innerModel.laserTo("world","laser",i.dist,i.angle);
		if( robotLaserCollision( p1, p2, p3, p ))
		{
			p = innerModel.laserTo("robot","laser",i.dist,i.angle);
			p = p.normalize() * (T)(-1);
			repulsionVector += p;
		}	
	}
	
	// Combine repulsionVector with (vadvance, vrot) and (VAdv,VRot) to whether stop or compute a safe deviation	
	
	//sum = (sum * QVec::vec3(vrot, 0, vadvance).norm2()) + QVec::vec3(vrot, 0, vadvance);
	if( repulsionVector != QVec::zeros(3))
	{
		qDebug() << "---COLLISION!!!!!!! at direction " << repulsionVector; 
		vadvance = -fabs(vadvance);
		differentialrobot_proxy->setSpeedBase( vadvance, vrot);		
	}

	//vadvance = sum.z();
	//vrot = sum.x();
	return true;
}

bool SpecificWorker::robotLaserCollision( const QVec &p1, const QVec &p2, const QVec &p3,  const QVec &p)
{
	QVec p21 = p2-p1;
	QVec p31 = p3-p1;
	
	if ((p.x() - p1.x()) * p21.x() + (p.z() - p1.z()) * p21.z() < 0.0) return false;
	if ((p.x() - p2.x()) * p21.x() + (p.z() - p2.z()) * p21.z() > 0.0) return false;
	if ((p.x() - p1.x()) * p31.x() + (p.z() - p1.z()) * p31.z() < 0.0) return false;
	if ((p.x() - p3.x()) * p31.x() + (p.z() - p3.z()) * p31.z() > 0.0) return false;
	
	return true;
}

/**
 * @brief Offset computation of each laser beam to account for the geometry of the getBaseState
 * 
 * @param innerModel ...
 * @param laserData ...
 * @return std::vector< float, std::allocator >
 */
std::vector<float> SpecificWorker::computeRobotOffsets(InnerModel& innerModel, const RoboCompLaser::TLaserData &laserData)
{
	//Base geometry GET FROM IM!!!
	QRectF base( QPointF(-200, 200), QPointF(200, -200));
	std::vector<float> baseOffsets;
	QVec p(3,0.f);
	int k;
	
	for(auto i : laserData)
	{
		for(k=10; k<4000; k++)
		{ 
			p = innerModel.laserTo("robot","laser",k,i.angle);
			if( base.contains( QPointF( p.x(), p.z() ) ) == false )
				break;
		}
		baseOffsets.push_back(k);
	}
	/*qDebug() << __FUNCTION__ << "BaseOffsets";
	for(auto i : baseOffsets)
		qDebug() << i;
	*/		
	return baseOffsets;
}


