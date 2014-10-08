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



/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx,QWidget *parent) : GenericWorker(mprx)
{
//	connect(goPushButton, SIGNAL(clicked()), this, SLOT(goButton()));
	connect(bedroomPushButton, SIGNAL(clicked()), this, SLOT(goBedRoom()));
	connect(kitchenPushButton, SIGNAL(clicked()), this, SLOT(goKitchen()));
	connect(kitchenPushButton_2, SIGNAL(clicked()), this, SLOT(goKitchen2()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(goKitchen()));
	connect(hallPushButton, SIGNAL(clicked()), this, SLOT(goHall()));
	connect(livingPushButton, SIGNAL(clicked()), this, SLOT(goLiving()));
	connect(entrancePushButton, SIGNAL(clicked()), this, SLOT(goEntrance()));
	connect(livingPushButton_2, SIGNAL(clicked()), this, SLOT(goLiving2()));
	connect(livingPushButton_3, SIGNAL(clicked()), this, SLOT(goLiving3()));
	connect(doorPushButton, SIGNAL(clicked()), this, SLOT(goDoor()));
	connect(diningPushButton, SIGNAL(clicked()), this, SLOT(goDining()));
	connect(diningPushButton_2, SIGNAL(clicked()), this, SLOT(goDining2()));
	connect(diningPushButton_3, SIGNAL(clicked()), this, SLOT(goDining3()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(stopRobot()));
	
	connect(t1Button, SIGNAL(clicked()), this, SLOT(step1()));
	connect(t2Button, SIGNAL(clicked()), this, SLOT(step2()));
	connect(t3Button, SIGNAL(clicked()), this, SLOT(step3()));
	connect(t4Button, SIGNAL(clicked()), this, SLOT(step4()));
	connect(t5Button, SIGNAL(clicked()), this, SLOT(step5()));
	
	
	connect(goHomeButton, SIGNAL(clicked()), this, SLOT(goHome()));
	
	
	plantWidget = new PlantWidget(frame, QPointF(82,457), QPointF(0,10000), QPointF(65,387), QPointF(0,-10000));
	plantWidget->show();
	statusLabel->setText("");
	
	state = State::IDLE;
	
	tag11 = false;
	tag12 = false;
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinBIKTest.xml");
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinBIKTest2.xml");
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/makeMeCoffee/simulation.xml");
	
	
	try 
	{	
		bodyinversekinematics_proxy->begin_goHome("HEAD");
		bodyinversekinematics_proxy->begin_goHome("RIGHTARM");
		bodyinversekinematics_proxy->setFingers(0);
		
		
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << "in Closefingers" << std::endl;}
	catch (const Ice::Exception &ex) 
	{ std::cout << ex << "in Close fingers" << std::endl;}
	
	removeAxis("marca-segun-head_AXIS");
	removeAxis("mano-segun-head_AXIS");
	removeAxis("marca-segun-head-cercana_AXIS");
	removeAxis("april-mug_AXIS");
	removeAxis("marca-segun-head-orientada_AXIS");
 	
	try{	innermodelmanager_proxy->removeNode("mugT2"); } catch(const RoboCompInnerModelManager::InnerModelManagerError &ex){};
	
	listaMotores 	<< "leftShoulder1" << "leftShoulder2" << "leftShoulder3" << "leftElbow" << "leftForeArm" << "leftWrist1" << "leftWrist2"
					<< "rightShoulder1" << "rightShoulder2" << "rightShoulder3" << "rightElbow"<< "rightForeArm" << "rightWrist1" << "rightWrist2"
					<< "base" << "head1" << "head3";
	
	attachMug();
	
	float alturaMesaHumano = innerModel->transform("world","april-table0").y();
	qDebug() << "alturaMesaHumano" << alturaMesaHumano;
					
	connect(plantWidget, SIGNAL(mouseMove(QVec)), this, SLOT(setTargetCoorFromPlant(QVec)));
	connect(plantWidget, SIGNAL(mousePress(QVec)), this, SLOT(setNewTargetFromPlant(QVec)));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::step1()
{ state = State::INIT_GO_KITCHEN;}

void SpecificWorker::step2()
{ state = State::SERVOING;}

void SpecificWorker::step3()
{	state = State::INIT_MOVE_ARM;}

void SpecificWorker::step4()
{ state = State::GRASP;}

void SpecificWorker::step5()
{ state = State::DETACH_TO_GET;}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(200);
	return true;
};

void SpecificWorker::compute( )
{
	try
	{
		planningState = trajectoryrobot2d_proxy->getState();
		bikState = bodyinversekinematics_proxy->getState("RIGHTARM");
		
		statusLabel->setText(QString::fromStdString( planningState.state ));
		
		if( planningState.state == "PLANNING" )
		{
			segsLcd->display(reloj.elapsed() );
		}
		if( planningState.state == "EXECUTING" )
		{
			float distance = (target - current).norm2();
			//estimatedDurationLcd->display((int)state.estimatedTime);
			estimatedDurationLcd->display(distance);
			executionTimeLcd->display(distance / 300);			
		}
	}
	catch(const Ice::Exception &ex)
	{	std::cout << ex << "Error talking to BIK in Compute" <<  std::endl;	}
	
	actualizarInnermodel(listaMotores);

	if( tag12 ) //mug
	{
		tag1LineEdit->setText("Id: 12");	
	}
	
	if( tag0 ) //Mesa human
	{
		tag1LineEdit->setText("Id: 12");	
	}
	if( tag11 ) //mano robot
	{
		tag2LineEdit->setText("Id: 11");	
	}
	doStateMachine();
	
}

void SpecificWorker::addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D)
{
		InnerModelNode *nodeParent = innerModel->getNode(parent);
		if( innerModel->getNode(name) == NULL)
		{
			InnerModelTransform *node = innerModel->newTransform(name, "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
			nodeParent->addChild(node);
		}
		innerModel->updateTransformValues(name, pose6D.x(), pose6D.y(), pose6D.z(), pose6D.rx(), pose6D.ry(), pose6D.rz());	
}

void SpecificWorker::addMeshInnerModel(const QString &name, const QString &parent, const meshType &mesh, const QVec &pose6D)
{
		InnerModelNode *nodeParent = innerModel->getNode(parent);
		if( innerModel->getNode(name) == NULL)
		{
			InnerModelMesh *node = innerModel->newMesh(name, nodeParent, QString::fromStdString(mesh.meshPath), mesh.scaleX, mesh.scaleY, mesh.scaleZ, 1,
																								 mesh.pose.x, mesh.pose.y, mesh.pose.z, mesh.pose.rx, mesh.pose.ry, mesh.pose.rz, false);
			nodeParent->addChild(node);
		}
		innerModel->updateTransformValues(name, pose6D.x(), pose6D.y(), pose6D.z(), pose6D.rx(), pose6D.ry(), pose6D.rz());	
}

void SpecificWorker::addPlaneInnerModel(const QString &name, const QString &parent, const Plane3D &plane, const QVec &pose6D)
{
		InnerModelNode *nodeParent = innerModel->getNode(parent);
		if( innerModel->getNode(name) == NULL)
		{
			InnerModelPlane *node = innerModel->newPlane(name, nodeParent, QString::fromStdString(plane.texture), plane.width, plane.height, plane.thickness, 0,
																								  plane.nx, plane.ny, plane.nz, plane.py, plane.py, plane.pz, 0);
			nodeParent->addChild(node);
		}
		innerModel->updateTransformValues(name, pose6D.x(), pose6D.y(), pose6D.z(), pose6D.rx(), pose6D.ry(), pose6D.rz());	
}

void SpecificWorker::actualizarInnermodel(const QStringList &listaJoints)
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		current = QVec::vec3(bState.x, 0, bState.z);  //For widget
		plantWidget->moveRobot( bState.x, bState.z, bState.alpha);	
		innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
		innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << "Error talking to Differentialrobot" << std::endl;
	}
	try 
	{
		RoboCompJointMotor::MotorList mList;
		for(int i=0; i<listaJoints.size(); i++)
			mList.push_back(listaJoints[i].toStdString());
		
		RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap(mList);
		
		for(int j=0; j<listaJoints.size(); j++)
			innerModel->updateJointValue(listaJoints[j], mMap.at(listaJoints[j].toStdString()).pos);

	} catch (const Ice::Exception &ex) {
		cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;
	}
}

void SpecificWorker::doStateMachine()
{
	switch (state) 
	{
		case State::IDLE:
				break;
		case State::INIT_GO_KITCHEN:
				state = init_go_kitchen();
		break;
		case State::GO_KITCHEN:
				state = go_kitchen();
		break;
		case State::INIT_PREPARE_ARM:
				state = initPrepareArm();
		break;
		case State::PREPARE_ARM:
				state = prepareArm();
		break;
		case State::INIT_APPROACH:
				state = initApproach();
		break;
		case State::APPROACH:
				state = approach();
		break;
		case State::SERVOING:
				state = servoing();
				break;
		case State::MOVE_ARM:
				state = moveArm();
				break;
		case State::INIT_MOVE_ARM:
				state = initMoveArm();
				break;
		case State::GRASP:
				state = grasp();
				break;
		case State::DETACH_TO_GET:
				state = detachToGet();
				break;
		case State::INIT_REDRAW_ARM:
				state = initRedrawArm();
				break;
		case State::REDRAW_ARM:
				state = redrawArm();
				break;
		case State::INIT_BACKUP:
				state = initBackUp();
				break;
		case State::BACKUP:
				state = backUp();
				break;
		case State::INIT_GO_OTHER_TABLE:
				state = initGoOtherTable();
				break;
		case State::GO_OTHER_TABLE:
				state = goOtherTable();
				break;
		case State::PUT_MUG_ON_TABLE:
				state = putMugOntable();
				break;
		case State::DETACH_TO_PUT:
				state = detachToPut();
				break;
		case State::INIT_REDRAW_ARM2:
				state = initRedrawArm2();
				break;				
		case State::REDRAW_ARM2:
				state = redrawArm2();
				break;				
		case State::BACKUP2:
				state = backUp2();
				break;		
		case State::INIT_GO_CENTER:
			state = initGoCenter();
			break;
		case State::GO_CENTER:
			state = goCenter();
			break;	
		default:
			qDebug() << __FUNCTION__ << "NO VALID STATE";
			break;
	}
}

SpecificWorker::State SpecificWorker::init_go_kitchen()
{
	attachMug();
	try 
	{	
		bodyinversekinematics_proxy->goHome("HEAD");
		bodyinversekinematics_proxy->goHome("RIGHTARM");
		bodyinversekinematics_proxy->setFingers(0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << "in goHome" << std::endl;}
	
	go(QVec::vec3(1300,0,-750), QVec::vec3(0,M_PI,0));
	sleep(1);   //ÑAÂ
	return State::GO_KITCHEN;
}

SpecificWorker::State SpecificWorker::go_kitchen()
{
	qDebug() << __FUNCTION__ << "planningState=" << QString::fromStdString(planningState.state);	
		
	if( planningState.state == "IDLE" )
	{
		qDebug() << __FUNCTION__ << "Made it...";
		stopRobot();
		return State::INIT_PREPARE_ARM;		
	}

	if( planningState.state == "EXECUTING" )
	{
		qDebug() << __FUNCTION__ << "Working...";
		gazeToTag("mugT");
		
	//if( close-to-the-target ) 
	//	initPrepareArm;
		
		return State::GO_KITCHEN;
	}
	
	if( planningState.state == "PLANNING" )
	{
		qDebug() << __FUNCTION__ << "Waiting for a plan...";
		return State::GO_KITCHEN;
	}
// 	initiated = false;
// 	qDebug() << __FUNCTION__ << "Otherwise. Should not happen. Passing to IDLE" << QString::fromStdString(planningState.state) << tag12 << initiated;
// 	return State::IDLE;

	return State::GO_KITCHEN;

}

SpecificWorker::State SpecificWorker::initPrepareArm()
{
	QVec prepPos = QVec::vec3(150, 800, 300);  //In robot RS
	QVec prepPosW = innerModel->transform("world", prepPos,"robot");
	
	closeFingers();
	try
	{
			bodyinversekinematics_proxy->setJoint("rightElbow", 1.9, 0.5);
			bodyinversekinematics_proxy->setJoint("rightShoulder1", -0.08, 0.3);
			bodyinversekinematics_proxy->setJoint("rightShoulder2", -1.2, 0.3);
			bodyinversekinematics_proxy->setJoint("rightShoulder3", 0.39, 0.3);
			bodyinversekinematics_proxy->setJoint("rightWrist1", 1, 0.3);
			bodyinversekinematics_proxy->setJoint("rightWrist2", 0.5, 0.3);
			sleep(3);
			return State::PREPARE_ARM;
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl; };
	
	qDebug() << __FUNCTION__ << "Something failed"; 
	return State::IDLE;
}

SpecificWorker::State SpecificWorker::prepareArm()
{
	qDebug() << __FUNCTION__;
	if( bikState.finish == false )
		return State::PREPARE_ARM;
	else
	{
		return State::INIT_APPROACH;
		//return State::IDLE;
	}	
}

SpecificWorker::State SpecificWorker::initApproach()  
{
	qDebug() << __FUNCTION__;	
	go(QVec::vec3(1300,0,-1250), QVec::vec3(0,M_PI,0));
	usleep(100000);
	return State::APPROACH;
}

SpecificWorker::State SpecificWorker::approach()
{
	qDebug() << __FUNCTION__;
	if( planningState.state  == "IDLE" )
	{
		gazeBetweenTags("handMesh2","april-mug");	
		openFingers();
		sleep(1);
		return State::GRASP;
	}
	else
 		return State::APPROACH;
}

/**
 * @brief Compute new final target.	It is a predefined position from the point of view of the just seen target 350 away from the target
 * 
 * @return SpecificWorker::State
 */


	
SpecificWorker::State SpecificWorker::servoing()
{
	static QVec ant = innerModel->transform("world", QVec::vec3(tag12Pose.x(),tag12Pose.y(),tag12Pose.z()), "rgbd_transform");
	
	if( planningState.state == "IDLE") //and (newRobotTarget - QVec::vec3(bState.x,0,bState.z)).norm2() < 40)
 	{
 		qDebug() << __FUNCTION__ << "Made it...";
 		stopRobot();
 		return State::INIT_MOVE_ARM;
 	}
	
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		//we need to give the Head target in ROBOT coordinates!!!!
		QVec loc;
		if( tag12 == true )
			loc = innerModel->transform("robot", QVec::vec3(tag12Pose.x(),tag12Pose.y(),tag12Pose.z()), "rgbd_transform");
		else
			loc = innerModel->transform("robot","mugT");
		loc.print("loc"); 
		target.rx=0; target.ry=0; target.rz=0; 	
		target.x = loc.x(); target.y=loc.y(); target.z = loc.z();
		RoboCompBodyInverseKinematics::Axis axis; 
		axis.x = 0; axis.y = -1; axis.z = 0;
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling pointAxisTowardsTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}

		
	QVec tagInWorld = innerModel->transform("world", QVec::vec3(tag12Pose.x(),tag12Pose.y(),tag12Pose.z()), "rgbd_transform");
	tagInWorld(1) = innerModel->transform("world","robot").y();
	try 
	{	
		RoboCompTrajectoryRobot2D::TargetPose tp;
		tp.x = tagInWorld.x(); tp.y = tagInWorld.y(); tp.z = tagInWorld.z() +150;
		trajectoryrobot2d_proxy->changeTarget( tp );
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling changeTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}
		
	return State::SERVOING;
	//return State::IDLE;
	
}


/**
 * @brief Sends the arms to a predefined position prior to grasping
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::initMoveArm()
{
	qDebug() << __FUNCTION__;
	closeFingers();
	
	//Gaze
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		//Gaze to where you think the mark is
		QVec loc = innerModel->transform("world","mesh-mug");  
		loc.print("loc");
		target.rx=0; target.ry=0; target.rz=0; 	
		target.x = loc.x()-80; target.y=loc.y(); target.z = loc.z();  
		RoboCompBodyInverseKinematics::Axis axis; 
		axis.x = 0; axis.y = -1; axis.z = 0;
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling pointAxisTowardsTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}
	
	usleep(100000);
	
	qDebug() << "Move arm";
	//Send the arm
	try 
	{
		QVec p = innerModel->transform("world","mesh-mug");  ////SI VE LA MARCA dEBERIA IR A LA MARCA. TO DO
		qDebug() << "Sending arm to" << p << "robot at" << innerModel->transform("world", QVec::zeros(6), "robot");
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = p.x()-120; pose.y = p.y(); pose.z = p.z();   	
		pose.rx =  M_PI; pose.ry=-M_PI/2; pose.rz= 0;  //don't care
		RoboCompBodyInverseKinematics::WeightVector weight;
		weight.x = 1; weight.y = 1; weight.z = 1; 
		weight.rx = 0; weight.ry = 0; weight.rz = 0;
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", pose, weight, true); 
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << std::endl; }
	catch (const Ice::Exception &ex) 
	{ std::cout << ex << std::endl; }
	
	initialDistance= 200;

	return State::MOVE_ARM;
}

/**
 * @brief Wait to finish the arm-cadic
 * 
 * @return SpecificWorker::State
 */

SpecificWorker::State SpecificWorker::moveArm()
{
	qDebug() << __FUNCTION__;

	if( bikState.finish == false )
	{
		qDebug() << "BIK is planning or moving";
		return State::MOVE_ARM;
	}
	else
	{
		sleep(1);
		openFingers();
		initialDistance = 200;
		sleep(1);
		if (tag11 == true and tag12 == true)
			return State::GRASP;
		else
		{
			qDebug() << __FUNCTION__ << "Can't see mark 11";
			return State::IDLE;
		}
	}
}


/**
 * @brief Visual servo following Peter Storke paper. We define an error pose as a circular transformation form desired Tip position to current Tip position.
 * Then, the error is sent to the position controller and updated again in a servo loop. the method guaratees that when the error is zero, both marks are in the same place, regardeless 
 * of the kinematic and sensor errors.
 * @return void
 *
 * 
 */
 
SpecificWorker::State SpecificWorker::grasp()
{
	qDebug() << __FUNCTION__;
// 	if( bikState.finish == false )
// 		return State::GRASP;			//REPLACE BY A POSITION CONTROL
	
	//innerModel->transform("world",QVec::zeros(6),"grabPositionHandR").print("Grab en el mundo antes de todo");
	
	//AprilTags Z axis points outwards the mark, X to the right and Y upwards (left hand convention)
	
	//Create hand as seen by head
	if( tag11 == true)
	{
		addTransformInnerModel("mano-segun-head", "rgbd_transform", tag11Pose);
	}
	else
		qDebug() << "No veo el 11";
// 	else
// 	{
// 		addTransformInnerModel("mano-segun-head", "rgbd_transform", innerModel->transform("rgbd_transform", QVec::zeros(6), "grabPositionHandR"));
// 		qFatal("fary");
// 	}
		
	//innerModel->transform("world", QVec::zeros(6), "mano-segun-head").print("mano-segun-head en world");
	//drawAxis("mano-segun-head", "rgbd_transform");
	//drawAxis("mano-segun-head", "world");		
	//Compute hand as felt in world
	//innerModel->transform("world",QVec::zeros(6),"ThandMesh2").print("mano through arm in world");
	//Difference should be zero if correctly calibrated
	//qDebug() << "Diferencia entre felt-hand y seen-hand (should be zero)" << innerModel->transform("world", QVec::zeros(6), "mano-segun-head")-innerModel->transform("world",QVec::zeros(6),"ThandMesh2");
	
	// Ponemos la marca de la mano vista desde la cámara en el sistma de coordenadas de la marca de la mano, si el target se ha alcanzado error debería ser todo cero
	QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh2", QVec::zeros(3), "mano-segun-head");
	//qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << innerModel->transform("ThandMesh2", QVec::zeros(6), "mano-segun-head");
	
	///CALIBRACION
	
	// Cogemos la matriz de rotación dek tHandMesh2 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que 
	// hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh2
	QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("ThandMesh2","mano-segun-head");
	QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh2_pre","ThandMesh2");
	
	// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
	QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
	//QMat finalHandMarcaRMat = visualMarcaRInHandMarcaMat;
	QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
	
	// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
	QVec handMarcaTInParent = innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2");
	QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
	//QVec finalHandMarcaT = visualMarcaTInHandMarca;

	// Esto es sólo para mostar como está el ThandMesh2 respecto al padre antes de las modificaciones
	//innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2").print("Posicion inicial del ThandMesh2 respecto al padre");

	/// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
	
	QVec finalHandMarca(6);
	finalHandMarca.inject(finalHandMarcaT,0);
	finalHandMarca.inject(finalHandMarcaR,3);
	//qDebug() << "Posicion final corregida del ThandMesh2 respecto al padre" << finalHandMarca;

	//Actualizamos el transform de la marca en la mano (ThandMesh1) con las rotaciones y translaciones calculadas
	innerModel->updateTransformValues("ThandMesh2",finalHandMarca.x(), finalHandMarca.y(), finalHandMarca.z(), finalHandMarca.rx(), finalHandMarca.ry(), finalHandMarca.rz());	

	//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
	qDebug() << "Grab en el mundo despues de modificar" << innerModel->transform("world", QVec::zeros(6), "grabPositionHandR");
	
	qDebug() << "-----------------------------------------------------------------------------------------------------\n";
	

	/////////////////////////////////////////////
	
	/// MODOFICAR LA POSICION DEL TIP EN EL BIK AQUI
	
	/////////////////////////////////////////////
	

	if( tag12 == true)
	{
		addTransformInnerModel("marca-segun-head", "rgbd_transform", tag12Pose);
 		//drawAxis("marca-segun-head", "world");
 		//drawAxis("april-mug", "world");
	}
	else 
	{
		//addTransformInnerModel("marca-segun-head", "rgbd_transform", innerModel->transform("rgbd_transform", QVec::zeros(6), "mesh-mug"));
		qDebug() << "Didn't see the mark 12";
 	}
	
	//innerModel->transform("world", QVec::zeros(6),"marca-segun-head").print("marca-segun-head en world");
	//innerModel->transform("world", QVec::zeros(6),"mesh-mug").print("marca en world");
	//qDebug() << "Differencia entre mano y marca visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"marca-segun-head") -
	//															innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
	//qDebug() << "Differencia entre mano y marca visual en el SR de la mano" << innerModel->transform("mano-segun-head", QVec::zeros(6),"marca-segun-head");
																
	
	//Build a CHANGING temporary target position close to the real target. SHOULD CHANGE TO APPROXIMATE INCREMTANLLY THE OBJECT 
	QVec nearTarget(6,0.f);	
	//nearTarget[0] = -initialDistance; nearTarget[1] = 0 ;nearTarget[2] = 0 ;nearTarget[3] = M_PI/2;nearTarget[4] = -M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES
	nearTarget[0] = -initialDistance; nearTarget[1] = 0 ;nearTarget[2] = 0 ;nearTarget[3] = M_PI/2;nearTarget[4] = M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES por lo que giro en Y al reves
	
	//Add a node in IM, hanging from marca-segun-head with a pose showing where we want the hand to be (adding a translation to the right)
	addTransformInnerModel("marca-segun-head-cercana", "marca-segun-head", nearTarget);
	
	//Add a node in IM, hanging from marca-segun-head with a pose showing where we to be in its final position (without translation to the right)
	QVec nearTargetR(6,0.f);
	nearTargetR[3] = M_PI/2; nearTargetR[4] = M_PI/2;; 
	addTransformInnerModel("marca-segun-head-orientada", "marca-segun-head", nearTargetR);
	
	//Rotatin matrices that must be equal. grabPositionHandR to ThandMesh2  must be equal than marca-segun-head-orientada to mano-segun-head
	QMat m = innerModel->getRotationMatrixTo("ThandMesh2", "grabPositionHandR");
	QMat mm = innerModel->getRotationMatrixTo("mano-segun-head", "marca-segun-head-orientada");
	//m.print("m");
	//mm.print("mm");
	
	//(mm * m.transpose()).print("RESTO6");
	//(mm * m.transpose()).extractAnglesR_min().print("angles");
	QVec anglesDiff = (mm*m.transpose()).extractAnglesR_min();
	
	
	//QVec mh = innerModel->transform("mano-segun-head",QVec::zeros(6),"marca-segun-head-cercana"); mh.print("mm");
	//QVec gp = innerModel->transform("ThandMesh2",QVec::zeros(6),"grabPositionHandR"); gp.print("grabPositionHandR");
	//innerModel->transform("world",QVec::zeros(6),"marca-segun-head-orientada").print("marca segun head orientada");
	
	//innerModel->transform("world",QVec::zeros(6),"mano-segun-head").print("mano-segun head");
	//innerModel->transform("world",QVec::zeros(6),"ThandMesh2").print("Thandmesh2");
	
// 	Rot3D mhm(mh.rx(),mh.ry(), mh.rz());
// 	mhm.print("mhm");
// 	Rot3D gpm(gp.rx(),gp.ry(), gp.rz());
// 	gpm.print("gpm");
		
	//drawAxis("marca-segun-head-cercana", "rgbd_transform");
// 	drawAxis("marca-segun-head-orientada", "rgbd_transform");
// 	drawAxis("mano-segun-head", "rgbd_transform");
	
	//Error between where the hand is and where it should end up
	float distCenters = innerModel->transform("mano-segun-head","marca-segun-head-orientada").norm2();
	qDebug() << "initialDistance" << initialDistance << "dist among centers" << distCenters << "angular distance" << anglesDiff;

	if( distCenters > 136 or anglesDiff.norm2() > 0.1) //120 is the distance along Z of grabPositionHandR wrt to hand mark
	{
		initialDistance = initialDistance - (distCenters/10.); 
 		if(initialDistance < 0 ) 
 			initialDistance = 0;
	
				//qDebug() << "Differencia entre mano y marca cercana visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"marca-segun-head-cercana") -
				//															innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
				//qDebug() << "Differencia entre mano y marca cercana visual en el SR de la mano" << innerModel->transform("marca-segun-head-cercana", QVec::zeros(6),"mano-segun-head");
				
				//drawAxis("marca-segun-head", "rgbd_transform");
				//drawAxis("marca-segun-head-cercana", "rgbd_transform");
				//drawAxis("marca-segun-head-cercana", "world");
			try 
			{
				RoboCompBodyInverseKinematics::Pose6D pose;
				QVec p = innerModel->transform("world",QVec::zeros(6),"marca-segun-head-cercana");
				pose.x = p.x();pose.y = p.y();pose.z = p.z();
				pose.rx = p.rx();pose.ry = p.ry();pose.rz = p.rz();
						
				RoboCompBodyInverseKinematics::WeightVector weights;
				weights.x = 1; 		weights.y = 1; 		weights.z = 1;
				weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
				//qDebug() << __FUNCTION__ << "Sent to target in RCIS" << p;
				bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
				qDebug() << "anteS";
				bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
				usleep(50000);
				qDebug() << "despues";
			} 
			catch (const Ice::Exception &ex) 
			{
				std::cout << ex << endl;
			} 
				
			//tag11 = false;		
			//tag12 = false;
				
			return State::GRASP;
	}	
	else
	{
		graspFingers();
		return State::DETACH_TO_GET;
	}	
}


/**
 * @brief Detatch mug from table and atatch to hand
 * 
 * @return void
 */
SpecificWorker::State SpecificWorker::detachToGet()
{
	qDebug() << __FUNCTION__ << "Detatching mug";
	try
	{
		//Get mugT info
		QVec mugTFix = innerModel->transform("grabPositionHandR",QVec::zeros(6),"mugT");
	
		//Get mug info
		RoboCompInnerModelManager::meshType mesh;
		InnerModelMesh *nmesh = dynamic_cast<InnerModelMesh *>(innerModel->getNode("mesh-mug"));
		mesh.pose.x=nmesh->tx; mesh.pose.y=nmesh->ty; mesh.pose.z=nmesh->tz; mesh.pose.rx=nmesh->rx; mesh.pose.ry=nmesh->ry; mesh.pose.rz=nmesh->rz;	
		mesh.scaleX=nmesh->scalex; mesh.scaleY=nmesh->scaley; mesh.scaleZ=nmesh->scalez; mesh.meshPath=nmesh->meshPath.toStdString();
	
		//Get mark info
		RoboCompInnerModelManager::Plane3D plane;
		InnerModelPlane *nplane = dynamic_cast<InnerModelPlane *>(innerModel->getNode("april-mug"));
		plane.px=nplane->point.x(); plane.py=nplane->point.y(); plane.pz=nplane->point.z(); 
		plane.nx=nplane->normal.x(); plane.ny=nplane->normal.y(); plane.nz=nplane->normal.z();	
		plane.width=nplane->width; plane.height=nplane->height; plane.thickness=nplane->depth; plane.texture=nplane->texture.toStdString();

		//remove mugT
		innermodelmanager_proxy->removeNode("mugT");
		innerModel->removeNode("mugT");

		RoboCompInnerModelManager::Pose3D pose;
		pose.x=mugTFix.x(); pose.y=mugTFix.y(); pose.z=mugTFix.z(); pose.rx=mugTFix.rx(); pose.ry=mugTFix.ry(); pose.rz=mugTFix.rz(); 
		innermodelmanager_proxy->addTransform("mugT","static","grabPositionHandR", pose);
		addTransformInnerModel("mugT","grabPositionHandR",mugTFix);
		innermodelmanager_proxy->addMesh("mesh-mug" , "mugT", mesh);
		addMeshInnerModel("mesh-mug","mugT",mesh, QVec::zeros(6));
		innermodelmanager_proxy->addPlane("april-mug" , "mugT", plane);
		addPlaneInnerModel("april-mug","mugT",plane, QVec::zeros(6));

	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{ 
		std::cout << ex << std::endl;
	}
	
	return State::INIT_REDRAW_ARM;
}


/**
 * @brief Lift and redraw little bit the arm to perform a safe transport
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::initRedrawArm()
{
	QVec p = innerModel->transform("world", QVec::zeros(6), "grabPositionHandR");
	p.print("Hand position after grabbing the mug");
	QVec cRobot = innerModel->transform("robot","grabPositionHandR");
	cRobot[1] += 100;
	cRobot[2] -= 100;
	QVec cWorld = innerModel->transform("world", cRobot,"robot");
	p.inject(cWorld,0);
	
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose;				
		pose.x = p.x();pose.y = p.y();pose.z = p.z(); pose.rx = p.rx();pose.ry = p.ry();pose.rz = p.rz();
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;	weights.rx = 1; 	weights.ry = 0; 	weights.rz = 1; 
		qDebug() << __FUNCTION__ << "Sent to target:" << p;
		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
		usleep(300000);
		return State::REDRAW_ARM;
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl; };
}

SpecificWorker::State SpecificWorker::redrawArm()
{
	qDebug() << __FUNCTION__;
	if( bikState.finish == false )
		return State::REDRAW_ARM;
	else
	{
		sleep(1);
		return State::INIT_BACKUP;
		//return State::INIT_GO_OTHER_TABLE;
	}	
}


SpecificWorker::State SpecificWorker::initBackUp()  
{
	qDebug() << __FUNCTION__;	
	goBackwards(innerModel->transform("world", QVec::vec3(0,0,-400), "robot"));
	return State::BACKUP;
}

SpecificWorker::State SpecificWorker::backUp()
{
	qDebug() << __FUNCTION__;
	if( planningState.state  == "IDLE" )
		return State::INIT_GO_OTHER_TABLE;
	else
 		return State::BACKUP;
}

SpecificWorker::State SpecificWorker::initGoOtherTable()
{
	qDebug() << __FUNCTION__;
	go(QVec::vec3(1200,0, 1100), QVec::vec3(0,0,0));
	sleep(1);
	return State::GO_OTHER_TABLE;
}

SpecificWorker::State SpecificWorker::goOtherTable()
{
	qDebug() << __FUNCTION__;
	if( planningState.state  == "PLANNING" or planningState.state == "EXECUTING" )
	{
		try 
		{	
			RoboCompBodyInverseKinematics::Pose6D target;
			target.rx=0; target.ry=0; target.rz=0; 
			QVec loc = innerModel->transform("world","mesh-table0");  //HARDCODED -------------------------
			target.x = loc.x(); target.y=loc.y()+750; target.z = loc.z(); //HARDCODED subimos la Y porque la taza parte del suelo
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = 0; axis.y = -1; axis.z = 0;
			bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
		} 
		catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
			{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
			
		return State::GO_OTHER_TABLE;
	}
	else
	{
		sleep(1);
		initialDistance = 150;
		if( tag0 == true)
			return State::PUT_MUG_ON_TABLE;
		else
		{
			qDebug() << __FUNCTION__ << "Could not see the mark 0. Going IDLE";
			return State::IDLE;
		}
	}
}

SpecificWorker::State SpecificWorker::putMugOntable()
{
	qDebug() << __FUNCTION__ ;
	
	if(tag0 == true)  //HARDCODED --------------------------
	{
		addTransformInnerModel("mesa-humano-segun-head", "rgbd_transform", tag0Pose);
	}
// 	else
// 	{
// 		addTransformInnerModel("mesa-humano-segun-head", "rgbd_transform", innerModel->transform("rgbd_transform", QVec::zeros(6), "april-table0"));
// 		qDebug() << "MISSING TAG0--------------------------------------";
// 	}
	
	innerModel->transform("world", QVec::zeros(6),"mesa-humano-segun-head").print("mesa-humano-segun-head en world");
	innerModel->transform("world", QVec::zeros(6),"mesa-humano-segun-head").print("mesa-humano en world");
	
	qDebug() << "Differencia entre mano y marca mesa visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"mesa-humano-segun-head") -
																innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
	qDebug() << "Differencia entre mano y marca mesa visual en el SR de la mano" << innerModel->transform("mano-segun-head", QVec::zeros(6),"mesa-humano-segun-head");
				
	
	/////////////////////////////////////////////
	
	/// MODOFICAR LA POSICION DEL TIP EN EL BIK AQUI
	
	/////////////////////////////////////////////
	
	//drawAxis("marca-segun-head", "rgbd_transform");
	//Build a CHANGING temporary target position close to the real target. SHOULD CHANGE TO APPROXIMATE INCREMTANLLY THE OBJECT 
	QVec nearTarget(6,0.f);
	qDebug() << "initialDistance" << initialDistance << "real dist" << innerModel->transform("mano-segun-head", "mesa-humano-segun-head").norm2() ;
	//nearTarget[0] = -initialDistance; nearTarget[1] = 0 ;nearTarget[2] = 0 ;nearTarget[3] = M_PI/2;nearTarget[4] = -M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES
	nearTarget[0] = initialDistance; nearTarget[1] = 0 ;nearTarget[2] = -80 ;nearTarget[3] = M_PI/2;nearTarget[4] = -M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES por lo que giro en Y al reves
	
// 	if( innerModel->transform("mano-segun-head", "mesa-humano-segun-head").norm2() > 100) 
// 	{
// 		initialDistance = initialDistance * 0.8;
// 		if(initialDistance < 100 ) 
// 			initialDistance = 109;
// 	}
// 	else
// 	{
// 		sleep(1);
// 		openFingers();
//  		return State::DETACH_TO_PUT;
// 	}
			
	addTransformInnerModel("mesa-humano-segun-head-cercana", "mesa-humano-segun-head", nearTarget);
	qDebug() << "Differencia entre mano y marca cercana visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"mesa-humano-segun-head-cercana") -
																innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
	qDebug() << "Differencia entre mano y marca cercana visual en el SR de la mano" << innerModel->transform("mesa-humano-segun-head-cercana", QVec::zeros(6),"mano-segun-head");
	
	//drawAxis("marca-segun-head", "rgbd_transform");
	//drawAxis("marca-segun-head-cercana", "rgbd_transform");
	//drawAxis("marca-segun-head-cercana", "world");
	try 
	{
		RoboCompBodyInverseKinematics::Pose6D pose;
		QVec p = innerModel->transform("world",QVec::zeros(6),"mesa-humano-segun-head-cercana");
		pose.x = p.x();pose.y = p.y();pose.z = p.z();	pose.rx = p.rx();pose.ry = p.ry();pose.rz = p.rz();
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;	weights.rx = 1; 	weights.ry = 0; 	weights.rz = 1; 
		qDebug() << __FUNCTION__ << "Sent to target in RCIS" << p;
		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
		sleep(1);
	} 
	catch (const Ice::Exception &ex) 
	{	std::cout << ex << endl; } 	
	
	
	//Gaze
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		target.rx=0; target.ry=0; target.rz=0; 
		QVec loc = innerModel->transform("world","mugT");  //HARDCODED -------------------------MARK ON TABLE
		target.x = loc.x(); target.y=loc.y(); target.z = loc.z();
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}

	
	tag0 = false;
	openFingers();
	sleep(1);
	return State::DETACH_TO_PUT;
	//return State::INIT_PUT_MUG_ON_TABLE;
}



SpecificWorker::State SpecificWorker::detachToPut()
{
	qDebug() << __FUNCTION__ << "Detatching to put mug on table";
	try
	{	
		//Get mugT info
		QVec mugTInHand = innerModel->transform("t_table0",QVec::zeros(6),"mesh-mug");
		QVec mugTOnTable = innerModel->transform("t_table0",QVec::zeros(6),"mugT");
		
		//Get mug info
		RoboCompInnerModelManager::meshType mesh;
		InnerModelMesh *nmesh = dynamic_cast<InnerModelMesh *>(innerModel->getNode("mesh-mug"));
		mesh.pose.x=nmesh->tx; mesh.pose.y=nmesh->ty; mesh.pose.z=nmesh->tz; mesh.pose.rx=nmesh->rx; mesh.pose.ry=nmesh->ry; mesh.pose.rz=nmesh->rz;	
		mesh.scaleX=nmesh->scalex; mesh.scaleY=nmesh->scaley; mesh.scaleZ=nmesh->scalez; mesh.meshPath=nmesh->meshPath.toStdString();
	
		//Get mark info
		RoboCompInnerModelManager::Plane3D plane;
		InnerModelPlane *nplane = dynamic_cast<InnerModelPlane *>(innerModel->getNode("april-mug"));
		plane.px=nplane->point.x(); plane.py=nplane->point.y(); plane.pz=nplane->point.z(); 
		plane.nx=nplane->normal.x(); plane.ny=nplane->normal.y(); plane.nz=nplane->normal.z();	
		plane.width=nplane->width; plane.height=nplane->height; plane.thickness=nplane->depth; plane.texture=nplane->texture.toStdString();

		//remove mugT
		innermodelmanager_proxy->removeNode("mugT");
		innerModel->removeNode("mugT");

		//mugTInHand.print("mugTInHand");
		//mugTPoseOnTable.print("mugTPoseOnTable");

		RoboCompInnerModelManager::Pose3D pose;		
		pose.x=mugTInHand.x(); pose.y=mugTOnTable.y(); pose.z=mugTInHand.z();
		pose.rx=mugTOnTable.rx(); pose.ry=mugTOnTable.ry(); pose.rz=mugTOnTable.rz();
		
		//Vector to create mutT in InnerModel
		QVec poseV = mugTOnTable;
		poseV[0] = mugTInHand.x(); mugTInHand[2] = mugTInHand.z();
		
		innermodelmanager_proxy->addTransform("mugT","static","t_table0", pose);	
		addTransformInnerModel("mugT","t_table0", poseV);
		innermodelmanager_proxy->addMesh("mesh-mug", "mugT", mesh);
		addMeshInnerModel("mesh-mug","mugT",mesh, QVec::zeros(6));
		innermodelmanager_proxy->addPlane("april-mug" , "mugT", plane);
		addPlaneInnerModel("april-mug","mugT",plane, QVec::zeros(6));	
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{ 
		std::cout << ex << std::endl;
	}
	
	sleep(1);
	return State::INIT_REDRAW_ARM2;   
}


/**
 * @brief Moves back the arm to a safe position to maneuvre
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::initRedrawArm2()
{
	QVec p = innerModel->transform("world", QVec::zeros(6), "grabPositionHandR");
	QVec cRobot = innerModel->transform("robot","grabPositionHandR");
	// New position closer to the body so its safe to move around
	cRobot[1] += 100;
	cRobot[2] -= 100;
	QVec cWorld = innerModel->transform("world", cRobot,"robot");
	p.inject(cWorld,0);
	
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose;				
		pose.x = p.x();pose.y = p.y();pose.z = p.z(); pose.rx = p.rx();pose.ry = p.ry();pose.rz = p.rz();
		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;	weights.rx = 1; 	weights.ry = 0; 	weights.rz = 1; 
		qDebug() << __FUNCTION__ << "Sent to target:" << p;
		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
		sleep(1);
		return State::REDRAW_ARM2;
	}
	catch(const Ice::Exception &ex)
	{ std::cout << ex << std::endl; };
	qDebug() << __FUNCTION__ << "A problem occured talking to BIK";
	return State::IDLE;
}

SpecificWorker::State SpecificWorker::redrawArm2()
{
	qDebug() << __FUNCTION__;
	if( bikState.finish == false )
		return State::REDRAW_ARM2;
	else
	{
		return State::BACKUP2;
	}	
}


SpecificWorker::State SpecificWorker::backUp2()  
{
	static bool firstTime=true;
	
	if( firstTime )
	{
		qDebug() << __FUNCTION__;
		QVec target = innerModel->transform("world", QVec::vec3(0,0,-400), "robot"); //HARDCODED ----------------------------
		try{ bodyinversekinematics_proxy->goHome("HEAD");} catch(const Ice::Exception &ex){ std::cout << ex << std::endl;};
		goBackwards(target);
		firstTime = false;
		return State::BACKUP2;
	}
	if( planningState.state  == "IDLE" )
	{
		firstTime = true;
		return State::INIT_GO_CENTER;
	}
	return State::BACKUP2;
}


SpecificWorker::State SpecificWorker::initGoCenter()
{
	//Sample a safe center spot to send the robot.
	float x = (2000.f / RAND_MAX)*qrand() -1000;
	float z = (2000.f / RAND_MAX)*qrand() -1000;
	float alfa = (2*M_PI / RAND_MAX)*qrand() - M_PI;
	
	go(QVec::vec3(x,0,z), QVec::vec3(0,alfa,0));
	sleep(1);
	try 
	{	
		bodyinversekinematics_proxy->goHome("RIGHTARM");
		bodyinversekinematics_proxy->setFingers(0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << "in initGoCenter" << std::endl;}
		
	return State::GO_CENTER;   	
}

SpecificWorker::State SpecificWorker::goCenter()  
{
	qDebug() << __FUNCTION__;
	if ( planningState.state == "EXECUTING" )
	{
		return State::GO_CENTER;
	}
	else
 		return State::INIT_GO_KITCHEN;
}


///////////////////////////////
///  HIGH LEVEL BEHAVIOR LAYER
//////////////////////////////

/**
 * @brief Make the head look at ->
 * 
 * @return void
 */
bool SpecificWorker::gazeToTag(const QString &tag)
{
	InnerModelNode *node = innerModel->getNode(tag);
	if( node == NULL)
	{
		qDebug() << __FUNCTION__ << tag << "does not exist in InnerModel";
		return false;
	}
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		QVec loc = innerModel->transform("world",tag);  
		target.rx=0; target.ry=0; target.rz=0; 	
		target.x = loc.x(); target.y=loc.y(); target.z = loc.z();  
		RoboCompBodyInverseKinematics::Axis axis; 
		axis.x = 0; axis.y = -1; axis.z = 0;
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling pointAxisTowardsTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}
		
	return true;
}

bool SpecificWorker::gazeBetweenTags(const QString &tag1, const QString &tag2)
{
	InnerModelNode *nodeA = innerModel->getNode(tag1);
	InnerModelNode *nodeB = innerModel->getNode(tag2);
	
	if( nodeA == NULL or nodeB == NULL)
	{
		qDebug() << __FUNCTION__ << "tags do not exist in InnerModel";
		return false;
	}
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		QVec loc = (innerModel->transform("world",tag1) + innerModel->transform("world",tag2))/2.f;  
		target.rx=0; target.ry=0; target.rz=0; 	
		target.x = loc.x(); target.y=loc.y(); target.z = loc.z();  
		RoboCompBodyInverseKinematics::Axis axis; 
		axis.x = 0; axis.y = -1; axis.z = 0;
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling pointAxisTowardsTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}
		
	return true;
}


void SpecificWorker::closeFingers()
{
	qDebug() << __FUNCTION__;

	//Close fingers
	try
	{	
		qDebug() << __FUNCTION__ << "Close fingers RCIS";
		bodyinversekinematics_proxy->setRobot(0);
		bodyinversekinematics_proxy->setFingers(0);  //ALLOW HAND SPECIFICATON
		usleep(100000);
	} 
	catch (Ice::Exception ex) {cout <<"ERROR EN CERRAR PINZA: "<< ex << endl;}
}

void SpecificWorker::graspFingers()
{
	qDebug() << __FUNCTION__;

	//Close fingers
	try
	{	
		qDebug() << __FUNCTION__ << "Close fingers RCIS";
		bodyinversekinematics_proxy->setRobot(0);
		bodyinversekinematics_proxy->setFingers(50);  //ALLOW HAND SPECIFICATON
		usleep(100000);
	} 
	catch (Ice::Exception ex) {cout <<"ERROR EN CERRAR PINZA: "<< ex << endl;}
}


void SpecificWorker::openFingers()
{
	qDebug() << __FUNCTION__;

	//Close fingers
	try
	{	
		qDebug() << __FUNCTION__ << "Open fingers RCIS";
		bodyinversekinematics_proxy->setRobot(0);
		bodyinversekinematics_proxy->setFingers(70);
		usleep(100000);
	} 
	catch (Ice::Exception ex) {cout <<"ERROR EN CERRAR PINZA: "<< ex << endl;}
}

void SpecificWorker::attachMug()
{
	qDebug() << __FUNCTION__ << "Attching mug";
	try
	{	
		//Get mug info
		RoboCompInnerModelManager::meshType mesh;
		InnerModelMesh *nmesh = dynamic_cast<InnerModelMesh *>(innerModel->getNode("mesh-mug"));
		mesh.pose.x=nmesh->tx; mesh.pose.y=nmesh->ty; mesh.pose.z=nmesh->tz; mesh.pose.rx=nmesh->rx; mesh.pose.ry=nmesh->ry; mesh.pose.rz=nmesh->rz;	
		mesh.scaleX=nmesh->scalex; mesh.scaleY=nmesh->scaley; mesh.scaleZ=nmesh->scalez; mesh.meshPath=nmesh->meshPath.toStdString();
	
		//Get mark info
		RoboCompInnerModelManager::Plane3D plane;
		InnerModelPlane *nplane = dynamic_cast<InnerModelPlane *>(innerModel->getNode("april-mug"));
		plane.px=nplane->point.x(); plane.py=nplane->point.y(); plane.pz=nplane->point.z(); 
		plane.nx=nplane->normal.x(); plane.ny=nplane->normal.y(); plane.nz=nplane->normal.z();	
		plane.width=nplane->width; plane.height=nplane->height; plane.thickness=nplane->depth; plane.texture=nplane->texture.toStdString();

		//remove mugT
		innermodelmanager_proxy->removeNode("mugT");
		
qDebug() << "hola";

		RoboCompInnerModelManager::Pose3D pose;
		QVec mugTPoseOnTable = innerModel->transform("t_table1",QVec::zeros(6),"mugPos1");
 		pose.x=mugTPoseOnTable.x(); pose.y=mugTPoseOnTable.y(); pose.z=mugTPoseOnTable.z();
// 		pose.x += (300.f / RAND_MAX)*qrand() -150;
// 		pose.z += (100.f / RAND_MAX)*qrand() -50;
		pose.rx=mugTPoseOnTable.rx(); pose.ry=mugTPoseOnTable.ry(); pose.rz=mugTPoseOnTable.rz();  
		innermodelmanager_proxy->addTransform("mugT","static","t_table1", pose);
		innermodelmanager_proxy->addMesh("mesh-mug", "mugT", mesh);
		innermodelmanager_proxy->addPlane("april-mug" , "mugT", plane);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{ 
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::drawAxis(const QString& name, const QString &parent)
{
	std::string axisName = name.toStdString() + "_AXIS";
	removeAxis(QString::fromStdString(axisName));
	try
	{
		float OX = 0; 
		float OZ = 10;

		QVec p =innerModel->transform(parent,QVec::zeros(6),name);
		RoboCompInnerModelManager::Pose3D pose;
		pose.x=p.x() + OX ; pose.y=p.y(); pose.z=p.z() + OZ; pose.rx=p.rx(); pose.ry=p.ry(); pose.rz=p.rz();
		innermodelmanager_proxy->addTransform(axisName, "static", parent.toStdString(), pose);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	
	try
	{
		RoboCompInnerModelManager::Plane3D planeX, planeY, planeZ;
		planeX.px = 50 ; planeX.py = 0; planeX.pz = 0; planeX.nx = 1; planeX.ny = 0; planeX.nz = 0; planeX.width = 3; planeX.height = 3; planeX.thickness = 100;	planeX.texture = "#ff0000";
		planeY.px = 0 ; planeY.py = 50; planeY.pz = 0; planeY.nx = 1; planeY.ny = 0; planeY.nz = 0; planeY.width = 3; planeY.height = 100; planeY.thickness = 3;	planeY.texture = "#00ff00";
		planeZ.px = 0 ; planeZ.py = 0; planeZ.pz = 50; planeZ.nx = 1; planeZ.ny = 0; planeZ.nz = 0; planeZ.width = 100; planeZ.height = 3; planeZ.thickness = 3;	planeZ.texture = "#0000ff";
		innermodelmanager_proxy->addPlane(axisName+"_X", axisName, planeX);
		innermodelmanager_proxy->addPlane(axisName+"_Y", axisName, planeY);
		innermodelmanager_proxy->addPlane(axisName+"_Z", axisName, planeZ);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{ 
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::removeAxis(const QString &name)
{
	try
	{
		innermodelmanager_proxy->removeNode(name.toStdString());
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}



void SpecificWorker::goHome()
{
	try 
	{	
		bodyinversekinematics_proxy->begin_goHome("HEAD");
		bodyinversekinematics_proxy->begin_goHome("RIGHTARM");
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << "in GoHome" << std::endl;}
	
	try{	innermodelmanager_proxy->removeNode("mugT2"); } 
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex.text << "in removeNode mugT2" << std::endl; };
}


////////////////////////////////////////////////////////////////////

/**
 * @brief Just for TrajectoryRobot2D. Go to t
 * 
 * @param t ...
 * @param r ...
 * @return void
 */
void SpecificWorker::go(const QVec& t, const QVec &r)
{
	qDebug() << __FUNCTION__;
	
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = t.x();
	tp.z = t.z();
	tp.y = 0;
	if( r.size() == 3 )
	{
		tp.rx = r.x(); tp.ry = r.y(); tp.rz = r.z();
		tp.doRotation = true;
	}
	else
		tp.doRotation = false;
	target = t;
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target " << t << " sent";
		reloj.restart();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::goBackwards(const QVec& t, const QVec &r)
{
	
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = t.x();
	tp.z = t.z();
	tp.y = t.y();
// 	if( r.size() == 3 )
// 	{
// 		tp.rx = r.x(); tp.ry = r.y(); tp.rz = r.z();
// 		tp.doRotation = true;
// 	}
// 	else
		tp.doRotation = false;
	target = t;
	try
	{
		trajectoryrobot2d_proxy->goBackwards(tp);
		qDebug() << __FUNCTION__ << "Target " << t << " sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}

void SpecificWorker::goButton()
{
	go(QVec::vec3(xSpinBox->value(),0, zSpinBox->value()));
}

void SpecificWorker::goDining()
{
	go(QVec::vec3(6000,0,-9100));
}

void SpecificWorker::goDining2()
{
	go(QVec::vec3(4600,0,-7800));
}

void SpecificWorker::goDining3()
{
	go(QVec::vec3(6200,0,-7100));
}

void SpecificWorker::goLiving()
{
	go(QVec::vec3(3000,0,-8100));
}

void SpecificWorker::goLiving2()
{
	go(QVec::vec3(666,0,-7861));
}

void SpecificWorker::goLiving3()
{
	go(QVec::vec3(790,0,-5062));
}

void SpecificWorker::goEntrance()
{
	go(QVec::vec3(2700,0,-4000));
}

void SpecificWorker::goDoor()
{
	go(QVec::vec3(4500,0,-3300));
}

void SpecificWorker::goKitchen()  
{
	go(QVec::vec3(6000,0,-5900), QVec::vec3(0,0,0));	
}

void SpecificWorker::goKitchen2()//Stove table
{
	go(QVec::vec3(5500,0,-5000), QVec::vec3(0,0,0));
}

void SpecificWorker::goBedRoom()
{
	go(QVec::vec3(7000,0,-1500));

}

void SpecificWorker::goHall()
{
	go(QVec::vec3(800,0,-1000));
}

void SpecificWorker::setTargetCoorFromPlant(QVec t)
{
	xSpinBox->display(t.x());
	zSpinBox->display(t.z());
}


void SpecificWorker::setNewTargetFromPlant(QVec t)
{
	xLcd->display(t.x());
	yLcd->display(t.z());
	go(t);
}

void SpecificWorker::stopRobot()
{
	state = State::IDLE;
	try
	{	trajectoryrobot2d_proxy->stop();	}
	catch(const Ice::Exception &ex)
	{		std::cout << ex << "Error talking to TrajectoryRobot2D" << std::endl;	}
	try
	{
		bodyinversekinematics_proxy->stop("HEAD");
		bodyinversekinematics_proxy->stop("RIGHTARM");
		bodyinversekinematics_proxy->stop("LEFTARM");
	}
	catch(const Ice::Exception &ex)
	{		std::cout << ex << "Error talking to BIK" << std::endl;	}
}


/////////////////////////////////////////////////////////////////
//// AprilTags subscription
/////////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	tag1LineEdit->setText("");
	tag11 = false;
	tag12 = false;
	
	for(auto i: tags)
	{
		if( i.id == 11) //ROBOT HAND
		{
			tag11 = true;
			tag11Pose.resize(6);
			tag11Pose[0] = i.tx;tag11Pose[1] = i.ty;tag11Pose[2] = i.tz;
			tag11Pose[3] = i.rx;tag11Pose[4] = i.ry;tag11Pose[5] = i.rz;
		}
		if( i.id == 12) //MUG
		{
			//qDebug() << __FUNCTION__ << "God damn got it!";
			tag12 = true;
			tag12Pose.resize(6);
			tag12Pose[0] = i.tx;tag12Pose[1] = i.ty;tag12Pose[2] = i.tz;
			tag12Pose[3] = i.rx;tag12Pose[4] = i.ry;tag12Pose[5] = i.rz;
		}
		if( i.id == 0) //ROBOT TABLE
		{
			//qDebug() << __FUNCTION__ << "God damn got it!";
			tag0 = true;
			tag0Pose.resize(6);
			tag0Pose[0] = i.tx;tag0Pose[1] = i.ty;tag0Pose[2] = i.tz;
			tag0Pose[3] = i.rx;tag0Pose[4] = i.ry;tag0Pose[5] = i.rz;
		}

	}
}
