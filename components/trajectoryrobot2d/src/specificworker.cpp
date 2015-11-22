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

#define RCIS

/**
* \brief Default constructor of trajectory2D main class
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, QWidget *parent) : GenericWorker(mprx)
{
	this->params = params;
	tState.setState("IDLE");
#ifdef USE_QTGUI
	innerViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
#else
	hide();
#endif

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Reads run time parameters from config file
 *
 * @param params ...
 * @return bool
 */

bool SpecificWorker::removeNode(const QString &item)
{
	if (item=="root")
	{
		qDebug() << "Can't remove root elements" << item;
		return false;
	}

	InnerModelNode *node = innerModel->getNode(item);
	if (node == NULL)
	{
		qDebug() << "Can't remove not existing elements" << item;
		return false;
	}

	QStringList l;
	innerModel->getSubTree(node, &l);
	innerModel->removeSubTree(node, &l);

	return true;
}

/**
 * @brief Adds a plane to InnerModel
 * 
 * @param item ...
 * @param parentS ...
 * @param path ...
 * @param scale ...
 * @param t ...
 * @param r ...
 * @return void
 */
void SpecificWorker::addPlane(QString item, QString parentS, QString path, QVec scale, QVec t, QVec r)
{
	InnerModelTransform *parent = dynamic_cast<InnerModelTransform*>(innerModel->getNode(parentS));
	if (innerModel->getNode(item) != NULL)
		removeNode(item);
	InnerModelMesh *mesh = innerModel->newMesh (item, parent, path, scale(0), scale(1), scale(2), 0, t(0), t(1), t(2), r(0), r(1), r(2));
	mesh->setScale(scale(0), scale(1), scale(2));
	parent->addChild(mesh);
}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel");
		if( QFile::exists(QString::fromStdString(par.value)) )
		{
			innerModel = new InnerModel(par.value);
#ifdef USE_QTGUI
			innerVisual = new InnerModel(par.value);	//USED TO REPRESENT innerModel in innerViewer
			innerViewer = new InnerModelViewer(innerVisual, "root", osgView->getRootGroup(), true);
			show();
#endif
		}
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
	//try { differentialrobot_proxy->getBaseState(bState); }

 	try {  omnirobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, can't communicate with robot proxy");}
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, can't communicate with laser proxy");}

	innerModel->updateTranslationValues("robot", bState.correctedX, 0, bState.correctedZ);	//"robot" should be an external parameter
	innerModel->updateRotationValues("robot", 0, bState.correctedAlpha, 0);
	innerModel->newTransform("virtualRobot","static",innerModel->getNode("robot"));
	
// 	QString obstacleParent = "floor";
// 	QString obstacleName = "ostias";
// 	QVec T = QVec::vec3(800, 400, 800);
// 	QVec R = QVec::vec3(0,0,1);
// 	QVec sizeObstacle = QVec::vec3(800, 800, 800);
// 	InnerModelDraw::addPlane_ignoreExisting(innerViewer, obstacleName, obstacleParent, T, R, "#555555", sizeObstacle);
// 	addPlane(obstacleName, obstacleParent, "/home/robocomp/robocomp/files/osgModels/basics/cube.3ds", sizeObstacle, T, R);

	//	setRobotInitialPose(800, -1500, M_PI);
	//	baseOffsets = computeRobotOffsets(innerModel, laserData);

	//Planning
	plannerPRM = new PlannerPRM(innerModel, 100, 20);
	planner = plannerPRM;
#ifdef USE_QTGUI
	planner->cleanGraph(innerViewer);
 	planner->drawGraph(innerViewer);
#endif

	// 	qDebug() << "----------------inserting" ;

	//Init road
	road.setInnerModel(innerModel);

	//Creates and amintains the road (elastic band) adapting it to the real world using a laser device
	elasticband = new ElasticBand();
	// 	qDebug() << __FUNCTION__ << "----- elasticband set";

	//Low level controller that drives the robot on the road by computing VAdv and VRot from the relative position wrt to the local road
	controller = new Controller(innerModel, laserData, 2);
	// 	qDebug() << __FUNCTION__ << "----- controller set";

 	//Localizer stuff
 	localizer = new Localizer(innerModel);
	
	timer.start(20);
	return true;
};


/**
 * @brief Calls current command on currentTarget until goal is reached
 *
 * @return void
 */
void SpecificWorker::compute( )
{
	static QTime reloj = QTime::currentTime();
	static int cont = 0;
	// Check for connection failure
	if ( updateInnerModel(innerModel, tState) == false )
	{
		controller->stopTheRobot(omnirobot_proxy);
		stopCommand(currentTarget, road, tState);
		tState.setState("DISCONNECTED");
	}
	
	CurrentTarget local;
	switch( currentTarget.command )
	{
		case CurrentTarget::Command::STOP:
			//qDebug() << __FUNCTION__ << "STOP";
			stopCommand(currentTarget, road, tState);
			break;
		case CurrentTarget::Command::CHANGETARGET:
			changeTargetCommand(innerModel, currentTarget, tState, road);
			break;
		case CurrentTarget::Command::GOTO:
			//qDebug() << __FUNCTION__ << "GOTO";
			gotoCommand(innerModel, currentTarget, tState, road, laserData);
			break;
		case CurrentTarget::Command::SETHEADING:
			//qDebug() << __FUNCTION__ << "SETHEADING";
			setHeadingCommand(innerModel, currentTarget.getRotation().y(), currentTarget, tState, road);
			break;
		case CurrentTarget::Command::GOBACKWARDS:
			//goBackwardsCommand(innerModel, currentTargetAnt, currentTarget, tState, road);
			 goBackwardsCommand(innerModel, currentTargetBack, currentTarget, tState, road);
			
			break;
		case CurrentTarget::Command::INSERTOBSTACLE:
			insertObstacle();
			break;
		case CurrentTarget::Command::IDLE:
			break;
	}

	if(reloj.elapsed() > 2000)	//to draw only every 2 secs
	{
#ifdef USE_QTGUI
		road.clearDraw(innerViewer);
		road.draw(innerViewer, innerModel, currentTarget);
#endif
		qDebug() << __FUNCTION__ << "Computed period" << reloj.elapsed()/cont << "State. Robot at:" << innerModel->transform("world","robot");
		cont = 0;
		reloj.restart();
	}
	cont++;

#ifdef USE_QTGUI// 	//CHECK PARAMETERS
// 	QVec target = current.getTranslation();
// 	if( target.size() < 3 or std::isnan(target.x()) or std::isnan(target.y()) or std::isnan(target.z()))
// 	{
// 		qDebug() << __FUNCTION__ << "Returning. Invalid target";
// 		RoboCompTrajectoryRobot2D::RoboCompException ex; ex.text = "Returning. Invalid target";
// 		throw ex;
// 		return false;
// 	}
// 	printf("doing backwards\n");
// 	float MAX_ADV_SPEED = 600.f;
// 	const float MAX_POSITIONING_ERROR  = 50;  //mm
// 	state.setState("EXECUTING");
// 	QVec rPose = innerModel->transform("world","robot");
// 	float error = (rPose-target).norm2();
// 	//float error = target.norm2();
// 	// 	qDebug() << __FUNCTION__ << "Error: " << error;
// 
// 	if( error < MAX_POSITIONING_ERROR )		//TASK IS FINISHED
// 	{
// //		current.setHasRotation(false);
// //		myRoad.setFinished(true);
// 		drawGreenBoxOnTarget( current.getTranslation() );
// // 		current.print();
// // 		current.reset();
// // 		myRoad.reset();
// // 		myRoad.endRoad();
// 		state.setElapsedTime(taskReloj.elapsed());
// 		//state.setState("IDLE");
// 		try
// 		{
// 		  omnirobot_proxy->setSpeedBase(0, 0, 0);
// 		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
// 		//myRoad.requiresReplanning = true;
// 
// 		currentT.setWithoutPlan(false);
// 		///////
// 		//agregar plane		AQUI HAY QUE AGREGAR EL PLANO Y DAR LA ORDEN DE REPLANIFICAR!!
// 		
// 		changeCommand(currentT,CurrentTarget::Command::INSERTOBSTACLE);
// 	}
// 	else
// 	{
// 		float vadv = -0.5 * error;  //Proportional controller
// 		if( vadv < -MAX_ADV_SPEED ) vadv = -MAX_ADV_SPEED;
// 		try
// 		{
// 		  //differentialrobot_proxy->setSpeedBase(vadv, 0);
// 		  omnirobot_proxy->setSpeedBase(0, vadv, 0);
// // 		  omnirobot2_proxy->setSpeedBase(0, vadv, 0);
// 		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
// 	}
// 

	if (innerViewer)
	{
		QMutexLocker ml(&mutex_inner);
		innerViewer->update();
		osgView->autoResize();
		osgView->frame();
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::insertObstacle()
{
	QString obstacleParent = "floor";
	QString obstacleName = "blockWall";
	QVec T = QVec::vec3(800, 400, 800);
	QVec R = QVec::vec3(0,0,1);
	QVec final = innerModel->transform("world",QVec::vec3(0,0,300),"laser");
	QVec sizeObstacle = QVec::vec3(500, 800, 100);
	
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, obstacleName, obstacleParent, final, R, "#555555", sizeObstacle);
	addPlane(obstacleName, obstacleParent, "/home/robocomp/robocomp/files/osgModels/basics/cube.3ds", sizeObstacle, final, R);
	innerViewer->update();
	try{ controller->stopTheRobot(omnirobot_proxy); } catch(const Ice::Exception &ex){std::cout << ex.what() << std::endl;};
	//replaning
	planner->removeGraph(innerViewer);
	qDebug() << __FUNCTION__ << "creating graph";
	planner->createGraph();
	currentTarget.setWithoutPlan(true);
	changeCommand(currentTarget,CurrentTarget::Command::GOTO);
	return true;
}


/**
 * @brief Stops the robot
 *
 * @return bool
 */
bool SpecificWorker::stopCommand(CurrentTarget& target, WayPoints& myRoad, TrajectoryState& state)
{
	// 	qDebug() << __FUNCTION__ ;
	controller->stopTheRobot(omnirobot_proxy);
	myRoad.setFinished(true);
	myRoad.reset();
	myRoad.endRoad();
#ifdef USE_QTGUI
	myRoad.clearDraw(innerViewer);
#endif
	drawGreenBoxOnTarget( target.getTranslation() );
	target.reset();
	changeCommand(target,CurrentTarget::Command::IDLE);
	state.setElapsedTime(taskReloj.elapsed());
	state.setState("IDLE");
	return true;
}

/**
 * @brief Changes current target without interrupting the ongoing process. Introduced to support visual-servoing
 *
 * @param innerModel
 * @return bool
 */
bool SpecificWorker::changeTargetCommand(InnerModel *innerModel, CurrentTarget &target, TrajectoryState &state, WayPoints &myRoad)
{
	// 	qDebug() << __FUNCTION__ ;
	myRoad.setFinished(true);
	myRoad.reset();
	myRoad.endRoad();
#ifdef USE_QTGUI
	myRoad.clearDraw(innerViewer);
#endif
	target.reset();
	changeCommand(target,CurrentTarget::Command::IDLE);
	state.setState("IDLE");
	return true;
}

/**
 * @brief Sends the robot to the target in currentTarget variable.
 * Calls in order
 * 		planner (if currentTarget has not a plan)
 * 			elasticband (to project the road against the laserData)
 * 				road (to  compute all parameters relating the robot to the robot )
 * 					controller (to compute control outputs for the robot to follow the road)
 *
 *  The insert-delete operators should be put here
 *  A new function could be inserted to check model validity: i.e. real laser profile against synthetic laser profile
 *  If a big difference is detected, it will be whether additive or substractive.
 *  If additive a new object can be inserted into innerModel and IMV,
 *  then the difference should be cancelled and currentTarget.isWithoutPlan can be set to True for replanning
 *  If substractive the innerModel object should be removed bacause someone already removed it in IMV (or the world),
 *  no replanning es necessary here.
 *
 * @param innerModel ...
 * @return bool
 */

bool SpecificWorker::gotoCommand(InnerModel* innerModel, CurrentTarget& target, TrajectoryState& state, WayPoints& myRoad, RoboCompLaser::TLaserData& lData)

{
	// 	qDebug() << __FUNCTION__;
	//printf("%d\n", __LINE__);
	if( targetHasAPlan(innerModel, target, state, myRoad) == true)
	{
		//printf("%d\n", __LINE__);
		//project road to the laserData so it adjust to real geometry

		elasticband->update( innerModel, myRoad, laserData, target);

		//compute all measures relating the robot to the road
		myRoad.update();

		//myRoad.printRobotState(innerModel, target);
		//move the robot according to the current force field
		controller->update(innerModel, lData, omnirobot_proxy, myRoad);
		
		if(myRoad.isBlocked() == true)
		{
		  currentTargetBack.setTranslation(innerModel->transform("world",QVec::vec3(0,0,-250),"robot"));
		  changeCommand(target,CurrentTarget::Command::GOBACKWARDS);
		  return true;
		}

		if (myRoad.isFinished() == true)
		{
			if( target.hasRotation() )
			{
				// qDebug() << __FUNCTION__ << "Changing to SETHEADING command";
				myRoad.setFinished(false);
				changeCommand(target,CurrentTarget::Command::SETHEADING);
			}
			else
			{
// 				planner->learnPath( road.backList );
				changeCommand(target,CurrentTarget::Command::STOP);
#ifdef USE_QTGUI
				planner->cleanGraph(innerViewer);
				planner->drawGraph(innerViewer);
#endif
			}
			planner->learnPath( road.backList );
		}

		// Get here when robot is stuck
		if(myRoad.requiresReplanning == true)
		{
			//qDebug() << __FUNCTION__ << "STUCK, PLANNING REQUIRED";
			//computePlan(innerModel);
		}

		//printf("%d\n", __LINE__);
		state.setEstimatedTime(myRoad.getETA());
		//printf("%d\n", __LINE__);
	}
	//printf("%d\n", __LINE__);
	return true;
}

/**
 * @brief Turns the robot until it reaches the desired orientation.
 *
 * @param innerModel ...
 * @param alfa angle between robot's z axis and worlds Z axis
 * @return bool Not used
 */
bool SpecificWorker::setHeadingCommand(InnerModel* innerModel, float alfa, CurrentTarget& target, TrajectoryState& state, WayPoints& myRoad)
{
	// 	qDebug() << __FUNCTION__;
	const float MAX_ORIENTATION_ERROR  = 0.08726646259722222;
	float angRobot = angmMPI(innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min().y());
	alfa = angmMPI(alfa);
	float error = angmMPI(angRobot-alfa);
	state.setState("EXECUTING-TURNING");
	//qDebug() << __FUNCTION__ <<"Error" << fabs(error);

	if( fabs(error) < MAX_ORIENTATION_ERROR)
		changeCommand(target,CurrentTarget::Command::STOP);
	else
	{
		float vrot = -0.7 * error;  //Proportional controller
		
		if(fabs(vrot)>0.3)    //ñapa para limitar la rotacion
                    if(vrot>0) vrot=0.3;
                    else vrot=-0.3;
                    
		try
		{
		  //differentialrobot_proxy->setSpeedBase(0, vrot);
		  omnirobot_proxy->setSpeedBase(0, 0, vrot);
// 		  omnirobot2_proxy->setSpeedBase(0, 0, vrot);
		} catch (const Ice::Exception &ex)
		{
			 std::cout << ex.what() << std::endl;
		}
	}
	return true;
}

/**
 * @brief Sends the robot bakcwards on a stright line until target is reached.
 *
 * @param innerModel ...
 * @param target position in World Reference System
 * @return bool
 */
bool SpecificWorker::goBackwardsCommand(InnerModel *innerModel, CurrentTarget &current,CurrentTarget &currentT, TrajectoryState &state, WayPoints &myRoad )
{
	//CHECK PARAMETERS
	QVec target = current.getTranslation();
	if( target.size() < 3 or std::isnan(target.x()) or std::isnan(target.y()) or std::isnan(target.z()))
	{
		qDebug() << __FUNCTION__ << "Returning. Invalid target";
		RoboCompTrajectoryRobot2D::RoboCompException ex; ex.text = "Returning. Invalid target";
		throw ex;
		return false;
	}
	float MAX_ADV_SPEED = 600.f;
	const float MAX_POSITIONING_ERROR  = 40;  //mm
	state.setState("EXECUTING");
	QVec rPose = innerModel->transform("world","robot");
	float error = (rPose-target).norm2();
	qDebug() << __FUNCTION__ << "doing backwards" << error;
	
	//float error = target.norm2();
	// 	qDebug() << __FUNCTION__ << "Error: " << error;

	if( error < MAX_POSITIONING_ERROR )		//TASK IS FINISHED
	{
//		current.setHasRotation(false);
//		myRoad.setFinished(true);
		drawGreenBoxOnTarget( current.getTranslation() );
// 		current.print();
// 		current.reset();
// 		myRoad.reset();
// 		myRoad.endRoad();
		state.setElapsedTime(taskReloj.elapsed());
		//state.setState("IDLE");
		try
		{
		  omnirobot_proxy->setSpeedBase(0, 0, 0);
		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
		//myRoad.requiresReplanning = true;

		currentT.setWithoutPlan(true);
		///////
		//agregar plane		AQUI HAY QUE AGREGAR EL PLANO Y DAR LA ORDEN DE REPLANIFICAR!!
		
		//changeCommand(currentT,CurrentTarget::Command::INSERTOBSTACLE);
		changeCommand(currentT,CurrentTarget::Command::GOTO);
	}
	else
	{
		float vadv = -0.5 * error;  //Proportional controller
		if( vadv < -MAX_ADV_SPEED ) vadv = -MAX_ADV_SPEED;
		try
		{
		  //differentialrobot_proxy->setSpeedBase(vadv, 0);
		  omnirobot_proxy->setSpeedBase(0, vadv, 0);
// 		  omnirobot2_proxy->setSpeedBase(0, vadv, 0);
		} catch (const Ice::Exception &ex) { std::cout << ex << std::endl; }
	}


	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// CLASS PRIVATE METHODS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief If there is no plan, this method computes a plan to achieve the current target
 *
 * @param inner ...
 * @return true if a plan has been obtained
 */
bool SpecificWorker::targetHasAPlan(InnerModel *inner, CurrentTarget &target, TrajectoryState &state, WayPoints &myRoad)
{
	if(  target.isWithoutPlan() == false )
		return true;

	QTime reloj = QTime::currentTime();
	// 	qDebug() << __FUNCTION__ << "Computing plan... ";

	state.setState("PLANNING");
	QVec localTarget = target.getTranslation();

	if ( planner->computePath(localTarget, inner) == false)
	{
		qDebug() << __FUNCTION__ << "SpecificWorker: Path NOT found. Resetting";
		state.setState("IDLE");
		target.reset();
		return false;
	}
	target.setTranslation( localTarget );
	qDebug() << __FUNCTION__ << "Plan obtained after " << reloj.elapsed() << "ms. Plan length: " << planner->getPath().size();

	// take inner to current values
	updateInnerModel(inner, state);
	target.setWithoutPlan( false );
	target.print();
#ifdef USE_QTGUI
	planner->cleanGraph(innerViewer);
 	planner->drawGraph(innerViewer);
#endif
	state.setState("EXECUTING");
	//Init road
	myRoad.reset();
	myRoad.readRoadFromList( planner->getPath() );
	//road.last() = currentTarget.getRotation();
	myRoad.requiresReplanning = false;
	myRoad.computeDistancesToNext();
//  	road.print();
	myRoad.update();  //NOT SURE IF NEEDED HERE
	myRoad.startRoad();
	state.setPlanningTime(reloj.elapsed());
#ifdef USE_QTGUI
	myRoad.draw(innerViewer, inner, target);
#endif
	//compState.planningTime = reloj.elapsed();
	return true;
}
void SpecificWorker::changeCommand(CurrentTarget& target, CurrentTarget::Command newCommand)
{
		QMutexLocker ml(&mutex_command);
		target.command=newCommand;
}

/**
 * @brief Updates an InnerModel from values read from the robot. Reads laserData too.
 *
 * @param inner InnerModel that is to be updated
 * @return bool
 */
bool SpecificWorker::updateInnerModel(InnerModel *inner, TrajectoryState &state)
{
	try
	{
		omnirobot_proxy->getBaseState(bState);
		inner->updateTransformValues("robot", bState.correctedX, 0, bState.correctedZ, 0, bState.correctedAlpha, 0);
		innerVisual->updateTransformValues("robot", bState.correctedX, 0, bState.correctedZ, 0, bState.correctedAlpha, 0);
		try
		{
			laserData = laser_proxy->getLaserData();
		}
		catch(const Ice::Exception &ex)
		{
			cout << ex << endl;
			state.setState("DISCONNECTED");
			return false;
		}
	}
	catch(const Ice::Exception &ex)
	{
		cout << ex << endl;
		state.setState("DISCONNECTED");
		return false;
	}

	if( state.getState() == "DISCONNECTED")
		state.setState("IDLE");
	return true;
}

void SpecificWorker::setRobotInitialPose(float x, float z, float alpha)
{
	// 	qDebug()<< __FUNCTION__ << "Sending robot to initial position";
#ifdef USE_QTGUI
	innerVisual->updateTransformValues("initialRobotPose", 0,0,0,  0,0,0);
	innerVisual->updateTransformValues("robot", x,0,z,  0,alpha,0);
#endif

	try
	{
		omnirobot_proxy->setOdometerPose(x, z, alpha);
	}
	catch(...)
	{
		qDebug() << __FUNCTION__ << "Error setting robot odometer";
	}
}

/////////////////////////////////////////////////////////////////////////////
//// DRAWING METHODS
////////////////////////////////////////////////////////////////////////////

void SpecificWorker::drawTarget(const QVec& target)
{
#ifdef USE_QTGUI
	//Draw target as red box
	QMutexLocker ml(&mutex_inner);
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target", "world", QVec::vec3(target(0), 5, target(2)), QVec::vec3(1,0,0), "#990000", QVec::vec3(80,80,80));
#endif
}

void SpecificWorker::drawGreenBoxOnTarget(const QVec& target)
{
 #ifdef USE_QTGUI
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target", "world", QVec::vec3(target(0), 1800, target(2)), QVec::vec3(1,0,0), "#009900", QVec::vec3(100,100,150));
#endif

}

void SpecificWorker::printNumberOfElementsInIMV()
{
/*
#ifdef USE_QTGUI
	try
	{	RoboCompInnerModelManager::NodeInformationSequence ni;
		innermodelmanager_proxy->getAllNodeInformation(ni);
// 		qDebug() << __FUNCTION__ << "..............Number of elements in: " << ni.size();
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
#endif
*/

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
////    SERVANTS
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Changes final point in current trajectory
 *
 * @param target New target pose. Only fist three elements are used as translation
 * @return void
 */
float SpecificWorker::changeTarget(const TargetPose& target)
{
	qDebug() <<__FUNCTION__ << "DEPRECATED";
	return 100;
}

/**
 * @brief Sends the robot to the target position.
 * The state of the process is recorded in the TrajectoryState structure that cnan be accessed through the getState() method
 * This method DOES NOT stop the robot before assigening a new target
 * @param target ...
 * @return void
 */
float SpecificWorker::go(const TargetPose& target)
{
	goReferenced(target,0,0,100);
}

RoboCompTrajectoryRobot2D::NavState SpecificWorker::getState()
{
	return tState.toMiddleware(this->bState, this->road);
}

void SpecificWorker::stop()
{
	// 	qDebug() << __FUNCTION__ << "STOP command received";
	changeCommand(currentTarget,CurrentTarget::Command::STOP);
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
	changeCommand(currentTarget,CurrentTarget::Command::SETHEADING);
	qDebug() << __FUNCTION__ << "SETHEADING command received";
}

/**
 * @brief Moves the rogbot backwards
 *
 * @param target ...
 * @return void
 */
float SpecificWorker::goBackwards(const TargetPose& target)
{
	qDebug() << __FUNCTION__ << "GOBACKWARDS command received";

	//PARAMETERS CHECK
	if( isnan(target.x) or std::isnan(target.y) or std::isnan(target.z) )
	{
		qDebug() <<__FUNCTION__ << "Returning. Input parameter -target- is not valid";
		RoboCompTrajectoryRobot2D::RoboCompException ex; ex.text = "Doing nothing. Invalid Target with nan in it";
		throw ex;
	}
	else
	{
		stop();
		while( tState.getState() != "IDLE"){};
		currentTarget.setTranslation( QVec::vec3(target.x, target.y, target.z) );
		currentTarget.setRotation( QVec::vec3(target.rx, target.ry, target.rz) );
		changeCommand(currentTarget,CurrentTarget::Command::GOBACKWARDS);
		if( target.doRotation == true)
			currentTarget.setHasRotation(true);
		drawTarget( QVec::vec3(target.x,target.y,target.z));
		taskReloj.restart();
		qDebug() << __FUNCTION__ << "-------------------------------------------------------------------------GOBACKWARDS command received, with target" << currentTarget.getTranslation() << currentTarget.getRotation();
	}
	
	return 0;
}

///////////////////////////////////
///////////////// AUX /////////////
///////////////////////////////////

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

float SpecificWorker::angmMPI(float angle)
{
	while (angle > +M_PI) angle -= 2.*M_PI;
	while (angle < -M_PI) angle += 2.*M_PI;
	return angle;
}



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


// bool SpecificWorker::checkRobotValidStateAtTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec &target)
// {
// 	//now check if there will be collision in the future position using current laserData
// 	float height = innerModel->transform("world","robot").y();
// 	//qDebug() << __FUNCTION__ << "height" << height << target.y();
// 	innerModel->updateTransformValues("robot", target.x(), target.y(), target.z(), 0, 0, 0);
//
// 	// Three points of rectangle approximating the robot base
// 	QVec p1 = innerModel->transform("world", QVec::vec3(-220,10,220), "robot" );
// 	QVec p2 = innerModel->transform("world", QVec::vec3(220,10,220), "robot" );
// 	QVec p3 = innerModel->transform("world", QVec::vec3(-220,10,-220), "robot" );
// 	//QVec p4 = innerModel->transform("world", QVec::vec3(220,0,-220), "robot" );
//
// 	QVec p21 = p2-p1;
// 	QVec p31 = p3-p1;
// 	//put back the robot to where it was
// 	innerModel->updateTransformValues("robot", bState.x, height, bState.z, 0, bState.alpha, 0);
//
// 	// 	target.print("target");
// 	// 	p1.print("p1");
// 	// 	p2.print("p2");
// 	// 	p3.print("p3");
//
// 	//Check if any laser point falls inside the rectangle using an angle criterium
// 	QVec p(3,0.f);
// 	for(auto i : laserData)
// 	{
// 		p = innerModel->laserTo("world","laser", i.dist, i.angle);
// 		//Check if inside
// 		if ((p-p1) * p21 > 0
// 			and
// 			(p-p1) * p21 < p21 * p21
// 			and
// 			(p-p1) * p31 > 0
// 			and
// 			(p-p1) * p31 < p31*p31 )
// 		return false;
// 	}
// 	return true;
// }
//
// bool SpecificWorker::searchRobotValidStateCloseToTarget(InnerModel *innerModel, const RoboCompLaser::TLaserData &laserData, QVec& target)
// {
// 	QVec lastPoint;
//
// 	QVec origin = innerModel->transform("world","robot");
// 	float stepSize = 30.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
// 	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);
// 	float step;
//
// 	//if too close return target
// 	if (nSteps == 0)
// 	{
// 		return false;
// 	}
// 	step = 1./nSteps;
//
// 	//go along visual ray connecting robot pose and target pos in world coordinates. l*robot + (1-r)*roiPos = 0
// 	QVec point(3);
// 	float landa = step;
//
// 	lastPoint = origin;
// 	for(uint i=1 ; i<=nSteps; i++)
// 	{
// 		point = (origin * (1-landa)) + (target * landa);
// 		if (checkRobotValidStateAtTarget(innerModel, laserData, point) )
// 		{
// 			lastPoint  = point;
// 			landa = landa + step;
// 		}
// 		else
// 		{
// 			target = lastPoint;
// 			return true;
// 		}
// 	}
// 	return true;
// }


float SpecificWorker::goReferenced(const TargetPose &target, const float xRef, const float zRef, const float threshold)
{
	QVec current = currentTarget.getTranslation();
	QVec currentRot = currentTarget.getRotation();
	if(fabs(target.x - current.x()) < 10 and fabs(target.z - current.z()) < 10 and (target.ry - currentRot.y()) < 0.05)
	{
		printf("same target \n");
		return road.getRobotDistanceToTarget();
	}
	//PARAMETERS CHECK
	if( isnan(target.x) or std::isnan(target.y) or std::isnan(target.z) )
	{
		qDebug() <<__FUNCTION__ << "Returning. Input parameter -target- is not valid";
		RoboCompTrajectoryRobot2D::RoboCompException ex; ex.text = "Doing nothing. Invalid Target with nan in it";
		throw ex;
	}
	else
	{
	    innerModel->updateTransformValues("virtualRobot",xRef,0,zRef,0,0,0,"robot");
		InnerModelDraw::addPlane_ignoreExisting(innerViewer, "virtualRobot", "robot", QVec::vec3(xRef,0,zRef), QVec::vec3(0,0,0), "#555555", QVec::vec3(50,1000,50));
		tState.setState("EXECUTING");
		road.setThreshold(threshold);
		currentTarget.setTranslation( QVec::vec3(target.x, target.y, target.z) );
		currentTarget.setRotation( QVec::vec3(target.rx, target.ry, target.rz) );
		qDebug() << currentTarget.getRotation() << "------ currentTarget ROtataion";
		changeCommand(currentTarget,CurrentTarget::Command::GOTO);
		currentTarget.setWithoutPlan(true);
		if( target.doRotation == true)
			currentTarget.setHasRotation(true);
		drawTarget( QVec::vec3(target.x,target.y,target.z));
		taskReloj.restart();
		qDebug() << __FUNCTION__ << "---------- GO command received with target at Tr:" << currentTarget.getTranslation() << "Angle:" << currentTarget.getRotation().alfa();
	}
	
	return road.getRobotDistanceToTarget();
}


void SpecificWorker::mapBasedTarget(const NavigationParameterMap &parameters)
{
	
}
void SpecificWorker::publicarROS()
{ 
	// time stamp = calcular;
	static uint32_t seq = 0;
	string frame_id="/map";
	for (auto p : road)
	{
		;
	}
	seq++;
}

