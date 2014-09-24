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
	compState.state = "IDLE";
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

////////////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);
	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel");
		if( QFile::exists(QString::fromStdString(par.value)) )
			innerModel = new InnerModel(par.value);
		else
		{
			std::cout << "Innermodel path " << par.value << " not found. "; qFatal("Abort");
		}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}

		//Update InnerModel from robot
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, robot not found");}
	// DESCOMENTAR:!!!!!!!!!!!!!!
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, laser not found");}
	
	innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
		 
//	setRobotInitialPose(800, -1500, M_PI);
//	baseOffsets = computeRobotOffsets(innerModel, laserData);
	
	//Planning
	plannerOMPL = new PlannerOMPL(innerModel);			
	plannerPRM = new PlannerPRM(innerModel, 100, 30);
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
	controller = new Controller(innerModel, laserData, 2);
	qDebug() << __FUNCTION__ << "----- controller set";
	
 	//Localizer stuff
 	localizer = new Localizer(innerModel);
// 	
// 	sleep(1);

	//Clon para Luis
//	innerClon = new InnerModel(innerModel);
	
	timer.start(20);	
	return true;
};




void SpecificWorker::computeLuis( )
{	
	//PARA PROBAR LOS MUEBLES:
	updateInnerModel(innerModel) ;
	
 	if (planner->getSampler().checkRobotValidStateAtTarget( innerModel->transform("world","robot") )) 
 		qDebug() << "Ho hay colision";

}




/**
 * @brief All architecture goes here. 
 * 
 * @return void
 */
void SpecificWorker::compute( )
{		
	static QTime reloj = QTime::currentTime();
	static QTime reloj2 = QTime::currentTime();
	
	//localizer->localize(laserData, innerModel, 16);
	
	if ( updateInnerModel(innerModel) )
	{
		if( currentTarget.isActive() and currentTarget.command == CurrentTarget::Command::STOP)
			stopCommand();
		
		else if( currentTarget.isActive() and currentTarget.command == CurrentTarget::Command::CHANGETARGET)
			changeTargetCommand(innerModel);
		
		else if( currentTarget.isActive() and currentTarget.command == CurrentTarget::Command::GOTO)
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
// 			planner->cleanGraph(innermodelmanager_proxy);
			road.draw(innermodelmanager_proxy, innerModel);
// 			planner->drawGraph(innermodelmanager_proxy);
		}
		reloj.restart();
	}
	reloj2.restart();
}
	


/////////////////////////////////////////////////////////

bool SpecificWorker::stopCommand()
{
	road.setFinished(true);	
	currentTarget.reset();
	controller->stopTheRobot(differentialrobot_proxy);
	compState.state = "IDLE";
	//drawGreenBoxOnTarget( currentTarget.getTranslation() );
	road.reset();
	road.endRoad();
	compState.elapsedTime = taskReloj.elapsed();
	return true;
}

/**
 * @brief Changes current target without interrupting the ongoing process. Introduced to support visual-servoing
 * 
 * @param innerModel ...
 * @return bool
 */
bool SpecificWorker::changeTargetCommand(InnerModel *innerModel)
{
	qDebug() << __FUNCTION__ << "with robot at" << innerModel->transform("world","robot");; 
	road.changeTarget( currentTarget.getTranslation());
	road.setFinished( false );
	drawTarget(currentTarget.getTranslation());
	currentTarget.command = CurrentTarget::Command::GOTO;
	return true;
}

bool SpecificWorker::gotoCommand(InnerModel *innerModel)
{
	qDebug() << __FUNCTION__;
	if( targetHasAPlan(innerModel))
	{
		qDebug() << "laserData" << laserData.size();
		elasticband->update( road, laserData, currentTarget);
		
		road.computeForces();
		
		//road.print();
	
		road.printRobotState( innerModel, currentTarget);

		controller->update(innerModel, laserData, differentialrobot_proxy, road);
		
		if (road.isFinished() == true)
		{		
			if( currentTarget.hasRotation() )
			{
				qDebug() << __FUNCTION__ << "Changing to SETHEADING command";
				road.setFinished(false);
				currentTarget.command = CurrentTarget::Command::SETHEADING;
			}
			else
			{
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
	qDebug() << __FUNCTION__;
	const float MAX_ORIENTATION_ERROR  = 0.05;
	
	float angRobot = innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	QVec v(1); 
	v[0] = (angRobot -alfa);
	calcularModuloFloat(v , M_PI);
	qDebug() << __FUNCTION__ << (angRobot-alfa) << v[0];

	if( fabs(v[0]) < MAX_ORIENTATION_ERROR)
	{
		currentTarget.setHasRotation(false);
		road.setFinished(true);
		drawGreenBoxOnTarget( currentTarget.getTranslation() );
		currentTarget.print();
		currentTarget.reset();
		road.reset();
		road.endRoad();
		compState.elapsedTime = taskReloj.elapsed();
		compState.state = "IDLE";
		try 
		{
		  differentialrobot_proxy->setSpeedBase(0, 0);	
		} catch (const Ice::Exception &ex) { std::cout << ex << std::cout; }
	}
	else
	{
		float vrot = -0.8 * v[0];  //Proportional controller
		try 
		{
		  differentialrobot_proxy->setSpeedBase(0, vrot);	
		} catch (const Ice::Exception &ex) { std::cout << ex << std::cout; }
	}
		
	return true;
}

/////////////////////////////////////////////////////////

/**
 * @brief If there is no plan, this method computes a plan to achieve the current target
 * 
 * @param inner ...
 * @return true if a plan has been obtained
 */
bool SpecificWorker::targetHasAPlan(InnerModel *inner)
{
	if( currentTarget.isWithoutPlan() == false ) 
		return true;
		
	QTime reloj = QTime::currentTime();
	qDebug() << __FUNCTION__ << "Computing plan... ";
	
	if (updateInnerModel(inner))
	{	
		compState.state = "PLANNING";
		QVec localTarget = currentTarget.getTranslation();
		if ( planner->computePath(localTarget, inner) == false)
		{
			qDebug() << __FUNCTION__ << "SpecificWorker: Path NOT found. Resetting";
			currentTarget.reset();
			return false;
		}
		currentTarget.setTranslation( localTarget );
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
	
	// DESCOMENTAR:!!!!!!!!!!!!!
	try 
	{ 
		laserData = laser_proxy->getLaserData(); 
	}
	catch(const Ice::Exception &ex) { cout << ex << endl; return false; }
	if( compState.state == "DISCONNECTED")
		compState.state = "IDLE";
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
void SpecificWorker::changeTarget(const TargetPose& target)
{
	QVec t = QVec::vec3((T)target.x, (T)target.y, (T)target.z);
	
	qDebug() << __FUNCTION__ << "CHANGE TARGET command received, with target" << t << "and robot at" << innerModel->transform("world","robot") ; 

	if ( currentTarget.isActive() == false) 
	{
		if( searchRobotValidStateCloseToTarget(innerModel, laserData, t ))
		{
			t.print("target after relocation");
			if( (currentTarget.getTranslation() - t).norm2() > 30 )
			{
				currentTarget.setTranslation( t );
				currentTarget.setRotation( QVec::vec3(target.rx, target.ry, target.rz) );
				if( target.onlyRot == true) 
					currentTarget.setHasRotation(true);
				else
					currentTarget.setHasRotation(false);
				currentTarget.command = CurrentTarget::Command::CHANGETARGET;
			}
		}
		else
		{
			qDebug() << __FUNCTION__ << "No valid target reposition found!";
			currentTarget.command = CurrentTarget::Command::STOP;
		}
		qDebug() << __FUNCTION__ << "No currentTarget active!";
	}
}

/**
 * @brief Sends the robot to the target position. the state of the process is recorded in the NavState structure that cnan be accessed through the getState() method
 * 
 * @param target ...
 * @return void
 */
void SpecificWorker::go(const TargetPose& target)
{
	
	stop();
	while( currentTarget.isActive() == true){};
	currentTarget.setActive(true);
	currentTarget.setTranslation( QVec::vec3(target.x, target.y, target.z) );
	currentTarget.setRotation( QVec::vec3(target.rx, target.ry, target.rz) );
	currentTarget.command = CurrentTarget::Command::GOTO;
	if( target.onlyRot == true) 
		currentTarget.setHasRotation(true);
	drawTarget( QVec::vec3(target.x,target.y,target.z));
	taskReloj.restart();
	qDebug() << __FUNCTION__ << "-------------------------------------------------------------------------GO command received, with target" << currentTarget.getTranslation() << currentTarget.getRotation();
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::getState()
{
	return compState;
}

void SpecificWorker::stop()
{
	qDebug() << __FUNCTION__ << "STOP command received";
	currentTarget.command = CurrentTarget::Command::STOP;
}

/**
 * @brief Integrator to orient the robot making an alfa angle with the world's Z axis
 * 
 * @param alfa ...
 * @return void
 */
void SpecificWorker::setHeadingTo(const TargetPose& target)
{
	//stop();
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
// void SpecificWorker::sendData(const RoboCompJoystickAdapter::TData& data)
// {
// 	//qDebug() << __FUNCTION__ << "Data from Joy";
// 	try 
// 	{	
// 		QList<QPair<QPointF,QPointF> > intervals;
// 		intervals.append(QPair<QPointF,QPointF>(QPointF(-1,1),QPointF(-600,600))); 
// 	
// 		//qDebug() << __FUNCTION__ << intervals << "X" << X;
// 		QMat m = QMat::afinTransformFromIntervals( intervals );
// 		float vadvance = (m * QVec::vec2(data.axes[data.dirAxisIndex].value, 1))[0];
// 		float vrot = data.axes[data.velAxisIndex].value;
// 		newData = true;
// 		ad = vadvance; ro = vrot;
// 		//avoidanceControl(innerModel, laserData, vadvance, vrot);
// 		//differentialrobot_proxy->setSpeedBase( vadvance, vrot);		
// 	} 
//    	catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl; }	
// }

///////////////////////////////////////7
/// PRUEBAS DEL CONTROLADOR
///////////////////////////////////////


// void SpecificWorker::filter(float& vadvance, float& vrot)
// {	
// 	if( repulsionVector != QVec::zeros(3))
//  	{
//  		qDebug() << "---COLLISION!!!!!!! at direction " << repulsionVector; 
//  		vadvance = -fabs(vadvance);
//  	}
// 	
// }


// /**
//  * @brief To go in Controller class or as a separate class with soft real time restrictions.
//  * Keeps the robot safe but, being the last level of control, should try to compute a safe diverging trajectory to keep things moving on the road.
//  * 
//  * @param innerModel ...
//  * @param road ...
//  * @param laserData ...
//  * @param vadvance ...
//  * @param vrot ...
//  * @return bool
//  */
// bool SpecificWorker::avoidanceControl(InnerModel& innerModel, const TLaserData& laserData, float& vadvance, float& vrot, uint elapsed)
// {
// 	const float MAX_ADV_ACC = 400;
// 	const float MAX_ROT_ACC = 0.5;
// 	//if( bState.advV == 0 and bState.rotV == 0 )
// 	//	return false;
// 	
// 	//check collision for a future potential position
// 	//given current speed and maximun acceleration, compute the minimun time to stop
// 	//float time = std::min<float>( fabs((vadvance + bState.advV) / 400.f), 0.1); // mm/sg2
// 	//float time = std::max<float>( (fabs(bState.advV) / 400.f), fabs(bState.rotV / 2.));
// 	float time = fabs(bState.advV + vadvance) / 400.f;
// 	//float time = elapsed / 1000.f;
// 	//we want to decide here if the next movement will cause a collision
// 	QVec repulsionVector;
// 	bool collision;
// 	std::tie(repulsionVector, collision) = checkInminentCollision( innerModel, laserData, 
// 																   bState.advV + vadvance, 
// 																   bState.rotV + vrot, time);
// 	
// 	//if( repulsionVector != QVec::zeros(3))
// 	if( collision )	
// 	{
// 		qDebug() << "---COLLISION!!!!!!! at direction " << repulsionVector; 
// 		
// 		//compute a <vadv,vrot> command to save the situation
// 		//check the outcome of all possible velocities to find one that saves the obstacle or else stop the robot
// 		//differentialrobot_proxy->setSpeedBase( vadvance, vrot);		
// 		differentialrobot_proxy->setSpeedBase( 0, 0);		
// 	}
// 	else
// 		try 
// 		{
// 			differentialrobot_proxy->setSpeedBase( vadvance, vrot);
// 		}
// 		catch (const Ice::Exception &e) { std::cout << e << "Differential robot not responding" << std::endl; }	 
// 	
// 	return true;
// }
// 
// std::tuple<QVec, bool> SpecificWorker::checkInminentCollision(InnerModel& innerModel, const RoboCompLaser::TLaserData& laserData, float vadv, float vrot, float delta)
// {
// 	//integrate speed to obtain base next position using Borenstein's approximation
// 	float deltaT = delta; // / 1000.f;
// 	float xN = bState.x + (vadv * deltaT) * sin(vrot * deltaT);
// 	float zN = bState.z + (vadv * deltaT) * cos(vrot * deltaT);
// 	float angN = bState.alpha + (vrot * deltaT);
// 	qDebug() << vadv << vrot << deltaT << "now" << bState.x << bState.z << bState.alpha << "new" << xN << zN << angN;
// 	
// 	//now check if there will be collision in the future position using current laserData
// 	innerModel->updateTransformValues("robot", xN, 0, zN, 0, angN, 0);
// 	// Three points of rectangle approximating the robot base 
// 	QVec p1 = innerModel->transform("world", QVec::vec3(220,0,220), "robot" );
// 	QVec p2 = innerModel->transform("world", QVec::vec3(220,0,-220), "robot" );
// 	QVec p3 = innerModel->transform("world", QVec::vec3(-220,0,220), "robot" );
// 	innerModel->updateTransformValues("robot", bState.x, 10, bState.z, 0, bState.alpha, 0);
// 	
// 	QVec p(3,0.f);
// 	int k=0;
// 	bool collision = false;
// 	QVec repulsionVector = QVec::zeros(3);
// 	for(auto i : laserData)
// 	{
// 		p = innerModel->laserTo("world","laser",i.dist,i.angle);
// 		if( robotLaserCollision( p1, p2, p3, p ))
// 		{
// 			//p = p * (T)(-1);
// 			repulsionVector += ( p * (T)(-1));
// 			collision = true;
// 		}	
// 	}
// 	return std::make_tuple(repulsionVector, collision);
// }
// 
// 
// 
// bool SpecificWorker::robotLaserCollision( const QVec &p1, const QVec &p2, const QVec &p3,  const QVec &p)
// {
// 	QVec p21 = p2-p1;
// 	QVec p31 = p3-p1;
// 	
// 	if ((p.x() - p1.x()) * p21.x() + (p.z() - p1.z()) * p21.z() < 0.0) return false;
// 	if ((p.x() - p2.x()) * p21.x() + (p.z() - p2.z()) * p21.z() > 0.0) return false;
// 	if ((p.x() - p1.x()) * p31.x() + (p.z() - p1.z()) * p31.z() < 0.0) return false;
// 	if ((p.x() - p3.x()) * p31.x() + (p.z() - p3.z()) * p31.z() > 0.0) return false;
// 	
// 	return true;
// }
// 
// /**
//  * @brief Offset computation of each laser beam to account for the geometry of the getBaseState
//  * 
//  * @param innerModel ...
//  * @param laserData ...
//  * @return std::vector< float, std::allocator >
//  */
// std::vector<float> SpecificWorker::computeRobotOffsets(InnerModel& innerModel, const RoboCompLaser::TLaserData &laserData)
// {
// 	//Base geometry GET FROM IM!!!
// 	QRectF base( QPointF(-200, 200), QPointF(200, -200));
// 	std::vector<float> baseOffsets;
// 	QVec p(3,0.f);
// 	int k;
// 	
// 	for(auto i : laserData)
// 	{
// 		for(k=10; k<4000; k++)
// 		{ 
// 			p = innerModel->laserTo("robot","laser",k,i.angle);
// 			if( base.contains( QPointF( p.x(), p.z() ) ) == false )
// 				break;
// 		}
// 		baseOffsets.push_back(k);
// 	}
// 	return baseOffsets;
// }
// 


bool SpecificWorker::checkRobotValidStateAtTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec &target)
{
	//now check if there will be collision in the future position using current laserData
	float height = innerModel->transform("world","robot").y();
	//qDebug() << __FUNCTION__ << "height" << height << target.y();
	innerModel->updateTransformValues("robot", target.x(), target.y(), target.z(), 0, 0, 0);

	// Three points of rectangle approximating the robot base 
	QVec p1 = innerModel->transform("world", QVec::vec3(-220,10,220), "robot" );
	QVec p2 = innerModel->transform("world", QVec::vec3(220,10,220), "robot" );
	QVec p3 = innerModel->transform("world", QVec::vec3(-220,10,-220), "robot" );
	//QVec p4 = innerModel->transform("world", QVec::vec3(220,0,-220), "robot" );
	
	QVec p21 = p2-p1;
	QVec p31 = p3-p1;
	//put back the robot to where it was
	innerModel->updateTransformValues("robot", bState.x, height, bState.z, 0, bState.alpha, 0);

	// 	target.print("target");
	// 	p1.print("p1");
	// 	p2.print("p2");
	// 	p3.print("p3");
	
	//Check if any laser point falls inside the rectangle using an angle criterium
	QVec p(3,0.f);
	for(auto i : laserData)
	{
		p = innerModel->laserTo("world","laser", i.dist, i.angle);
		//Check if inside
		if ((p-p1) * p21 > 0 
			and 
			(p-p1) * p21 < p21 * p21 
			and 
			(p-p1) * p31 > 0 
			and 
			(p-p1) * p31 < p31*p31 )
		return false;
	}
	return true;
}

bool SpecificWorker::searchRobotValidStateCloseToTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec& target)
{
	QVec lastPoint;

	QVec origin = innerModel->transform("world","robot");
	float stepSize = 30.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
	float step;
	
	//if too close return target
	if (nSteps == 0) 
	{
		return false;
	}
	step = 1./nSteps;

	//go along visual ray connecting robot pose and target pos in world coordinates. l*robot + (1-r)*roiPos = 0
	QVec point(3);
	float landa = step;
	
	lastPoint = origin;
	for(uint i=1 ; i<=nSteps; i++)
	{
		point = (origin * (1-landa)) + (target * landa);
		if (checkRobotValidStateAtTarget(innerModel, laserData, point) ) 
		{
			lastPoint  = point;
			landa = landa + step;
		}
		else
		{
			target = lastPoint;
			return true;
		}
	}
	return true;
}

// bool SpecificWorker::searchRobotValidStateCloseToTarget(InnerModel &innerModel, const RoboCompLaser::TLaserData &laserData, QVec& target)
// {
// 	//If current is good, return
// 	if( checkRobotValidStateAtTarget(innerModel, laserData, target) )
// 		return true;
// 	
// 	//Start searching radially from target to origin and adding the vertices of a n regular polygon of radius 1000 and center "target"
// 	const int nVertices = 12;
// 	const float radius = 1000.f;
// 	QVec lastPoint, minVertex, vertex;
// 	float fi,vert;
// 	float dist, minDist = radius;
// 	
// 	for(int i=0; i< nVertices; i++)
// 	{
// 		fi = (2.f*M_PI/nVertices) * i;
// 		int k;
// 		bool free;
// 		for(k=100; k<radius; k=k+100)
// 		{
// 			vertex = QVec::vec3(target.x() + k*sin(fi), target.y(), target.z() + k*cos(fi));
// 			free = checkRobotValidStateAtTarget(innerModel, laserData, vertex);
// 			if (free == true) 
// 				break;
// 		}
// 		if( free and k < minDist )
// 		{
// 			minVertex = vertex;
// 			minDist = k;	
// 			vert = fi;
// 		}
// 	}
// 	if( minDist < radius)
// 	{
// 		target = minVertex;
// 		target.print("new target");
// 		qDebug() << minDist << vert;
// 		qFatal("fary");
// 		return true;
// 	}
// 	else
// 		return false;
// }

///////////////// AUX /////////////

/*
* Metodo moduloFloat
* Devuelve el modulo entre dos numeros reales.   ///HAS PROBADO FMOD?
* FUNCIONA.
*/ 
void SpecificWorker::calcularModuloFloat(QVec &angles, float mod)
{
	for(int i=0; i<angles.size(); i++)
	{
		int cociente = (int)(angles[i] / mod);
		angles[i] = angles[i] -(cociente*mod);
		
		if(angles[i] > M_PI)
			angles[i] = angles[i]- M_PI;
		else
			if(angles[i] < -M_PI)
				angles[i] = angles[i] + M_PI;
	}
}
