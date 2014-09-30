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
	
	removeAxis("marca-segun-head");
	removeAxis("mano-segun-head");
	removeAxis("marca-segun-head-cercana");
 	try{	innermodelmanager_proxy->removeNode("mugT2"); } catch(const RoboCompInnerModelManager::InnerModelManagerError &ex){};
	
	listaMotores 	<< "leftShoulder1" << "leftShoulder2" << "leftShoulder3" << "leftElbow" << "leftForeArm" << "leftWrist1" << "leftWrist2"
					<< "rightShoulder1" << "rightShoulder2" << "rightShoulder3" << "rightElbow"<< "rightForeArm" << "rightWrist1" << "rightWrist2"
					<< "base" << "head1" << "head2" << "head3";
	
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
{ state = State::GO_KITCHEN;}

void SpecificWorker::step2()
{ state = State::SERVOING;}

void SpecificWorker::step3()
{ 
	state = State::INIT_MOVE_ARM;
}

void SpecificWorker::step4()
{ state = State::GRASP;}

void SpecificWorker::step5()
{ state = State::DETACH;}


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
	{	std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;	}
	
	actualizarInnermodel(listaMotores);

	if( tag12 ) 
	{
		tag1LineEdit->setText("Id: 12");	
	}
	if( tag11 ) 
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
				qDebug() << "IDLE";
				break;
		case State::GO_KITCHEN:
				qDebug() << "GO_KITCHEN";
				state = go_kitchen();
		break;
		case State::SERVOING:
				qDebug() << "SERVOING";
				state = servoing();
				break;
		case State::MOVE_ARM:
				qDebug() << "MOVE_ARM";
				state = moveArm();
				break;
		case State::INIT_MOVE_ARM:
				qDebug() << "INIT_MOVE_ARM";
				state = initMoveArm();
				break;
		case State::GRASP:
				
				qDebug() << "GRASP";
				state = grasp();
				break;
		case State::CLOSE_FINGERS:
				qDebug() << "CLOSE_FINGERS";
				state = closeFingers();
				break;
		case State::OPEN_FINGERS:
				qDebug() << "OPEN_FINGERS";
				state = openFingers();
				break;
		case State::DETACH:
				qDebug() << "DETACH";
				state = detach();
				break;
				
		default:
			break;
	}
}

SpecificWorker::State SpecificWorker::go_kitchen()
{
	static bool initiated = false;
	
	if( planningState.state == "IDLE" and not initiated)
	{
		qDebug() << __FUNCTION__ << "sending command";	
		
		//go(QVec::vec3(5500,0,-5100), QVec::vec3(0,0,0));
		go(QVec::vec3(1200,0,-1300), QVec::vec3(0,M_PI,0));
		
		initiated = true;
		try 
		{	
			bodyinversekinematics_proxy->begin_goHome("HEAD");
			bodyinversekinematics_proxy->begin_goHome("RIGHTARM");
			bodyinversekinematics_proxy->setFingers(0);
		} 
		catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
		
		return State::GO_KITCHEN;
	}
	
	//if( planningState.state == "IDLE" and initiated and (QVec::vec3(5500,0,-5000) - QVec::vec3(bState.x,0,bState.z)).norm2() < 100)
	
	if( planningState.state == "IDLE" and initiated and (QVec::vec3(1200,0,-1300) - QVec::vec3(bState.x,0,bState.z)).norm2() < 100)
	{
		qDebug() << __FUNCTION__ << "Made it...";
		initiated = false;
		stopRobot();
		return State::IDLE;
	}
	
	if( tag12 == true) 
	{
		qDebug() << __FUNCTION__ << "TAG11";
		initiated = false;
		stopRobot();
		tag11 = false;

		return State::IDLE;

		
		QVec tagInWorld = innerModel->transform("world", QVec::vec3(tag12Pose.x(),tag12Pose.y(),tag12Pose.z()), "rgbd_transform");
		tagInWorld(1) = innerModel->transform("world","robot").y();
		go(tagInWorld, QVec::vec3(0,0,0));  //Should be perpendicular to table long side
		qDebug() << "send to tag location " << tagInWorld;
		sleep(1);
		return State::SERVOING;
	}
	
	if( planningState.state == "EXECUTING" )
	{
		qDebug() << __FUNCTION__ << "Working...";
		try 
		{	
			RoboCompBodyInverseKinematics::Pose6D target;
			target.rx=0; target.ry=0; target.rz=0; 		
			//we need to give the Head target in ROBOT coordinates!!!!
			QVec loc = innerModel->transform("world","mugT");
			target.x = loc.x(); target.y=loc.y(); target.z = loc.z();
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = 0; axis.y = -1; axis.z = 0;
			bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, true, 0);
		} 
		catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
			{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
		
		return State::GO_KITCHEN;
	}
	
	if( planningState.state == "PLANNING" )
	{
		qDebug() << __FUNCTION__ << "Waiting for a plan...";
		return State::GO_KITCHEN;
	}
	initiated = false;
	qDebug() << __FUNCTION__ << "Otherwise. Should not happen. Passing to IDLE" << QString::fromStdString(planningState.state) << tag11 << initiated;
	return State::IDLE;
}

/**
 * @brief Compute new final target.	It is a predefined position from the point of view of the just seen target 350 away from the target
 * 
 * @return SpecificWorker::State
 */


	
SpecificWorker::State SpecificWorker::servoing()
{
	static QVec ant = innerModel->transform("world", QVec::vec3(tag11Pose.x(),tag11Pose.y(),tag11Pose.z()), "rgbd_transform");
	
	if( planningState.state == "IDLE") //and (newRobotTarget - QVec::vec3(bState.x,0,bState.z)).norm2() < 40)
 	{
 		qDebug() << __FUNCTION__ << "Made it...";
 		stopRobot();
 		return State::MOVE_ARM;
 	}
	
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		//we need to give the Head target in ROBOT coordinates!!!!
		QVec locA = innerModel->transform("robot", QVec::vec3(tag11Pose.x(),tag11Pose.y(),tag11Pose.z()), "rgbd_transform");
		QVec loc = innerModel->transform("robot","mugT");
		locA.print("locA"); loc.print("loc");
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

		
	QVec tagInWorld = innerModel->transform("world", QVec::vec3(tag11Pose.x(),tag11Pose.y(),tag11Pose.z()), "rgbd_transform");
	tagInWorld(1) = innerModel->transform("world","robot").y();
	try 
	{	
		RoboCompTrajectoryRobot2D::TargetPose tp;
		tp.x = tagInWorld.x(); tp.y = tagInWorld.y(); tp.z = tagInWorld.z();
		trajectoryrobot2d_proxy->changeTarget( tp );
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
		{ std::cout << ex.text << "calling changeTarget" << std::endl;}
	catch (const Ice::Exception &ex) 
		{ std::cout << ex << std::endl;}
		
	return State::SERVOING;
	//return State::IDLE;
	
}

SpecificWorker::State SpecificWorker::closeFingers()
{
	qDebug() << __FUNCTION__;

	//Close fingers
	try
	{	
		qDebug() << __FUNCTION__ << "Close fingers RCIS";
		bodyinversekinematics_proxy->setRobot(0);
		bodyinversekinematics_proxy->setFingers(0);
		usleep(100000);
	} 
	catch (Ice::Exception ex) {cout <<"ERROR EN CERRAR PINZA: "<< ex << endl;}
	return State::INIT_MOVE_ARM;
}


SpecificWorker::State SpecificWorker::openFingers()
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
	return State::INIT_MOVE_ARM;
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
		QVec loc = innerModel->transform("world","mugT");
		loc.print("loc");
		target.rx=0; target.ry=0; target.rz=0; 	
		target.x = loc.x()-80; target.y=loc.y(); target.z = loc.z();  /// OJO. EL TAG en el mundo está girado al revés que en el ROCKIN
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
		//QVec p = innerModel->transform("robot", QVec::vec3(tag.tx,tag.ty,tag.tz), "rgbd_transform");
		QVec p = innerModel->transform("world","april-mug");
		qDebug() << "Sending arm to" << p << "robot at" << innerModel->transform("world", QVec::zeros(6), "robot");
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = p.x()-120; pose.y = p.y(); pose.z = p.z();   /// OJO. EL TAG en el mundo está girado al revés que en el ROCKIN. Restamos en X
	//	drawAxis("april-mug", "world");
		
		pose.rx =  M_PI; pose.ry=-M_PI/2; pose.rz= 0;  //don't care
		RoboCompBodyInverseKinematics::WeightVector weight;
		weight.x = 1; weight.y = 1; weight.z = 1; 
		weight.rx = 0; weight.ry = 0; weight.rz = 0;
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", pose, weight, 0); 
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
		return State::MOVE_ARM;
	else
	{
		openFingers();
 		return State::GRASP;
//		return State::IDLE;
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
	
	innerModel->transform("world",QVec::zeros(6),"grabPositionHandR").print("Grab en el mundo antes de todo");
	
	//AprilTags Z axis points outwards the mark, X to the right and Y upwards (left hand convention)
	
	//Create hand as seen by head
	if( tag11 == true)
	{
		addTransformInnerModel("mano-segun-head", "rgbd_transform", tag11Pose);
	}
	else
	{
		addTransformInnerModel("mano-segun-head", "rgbd_transform", innerModel->transform("rgbd_transform", QVec::zeros(6), "handMesh2"));
	}
		
	innerModel->transform("world", QVec::zeros(6), "mano-segun-head").print("mano-segun-head en world");
	//drawAxis("mano-segun-head", "rgbd_transform");
	//drawAxis("mano-segun-head", "world");		
	
	//Compute hand as felt in world
	innerModel->transform("world",QVec::zeros(6),"ThandMesh2").print("mano through arm in world");
	
	//Difference should be zero if correctly calibrated
	qDebug() << "Diferencia entre felt-hand y seen-hand (should be zero)" << innerModel->transform("world", QVec::zeros(6), "mano-segun-head")-innerModel->transform("world",QVec::zeros(6),"ThandMesh2");
	
	// Ponemos la marca de la mano vista desde la cámara en el sistma de coordenadas de la marca de la mano, si el target se ha alcanzado error debería ser todo cero
	QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh2", QVec::zeros(3), "mano-segun-head");
	qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << innerModel->transform("ThandMesh2", QVec::zeros(6), "mano-segun-head");
	
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
	innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2").print("Posicion inicial del ThandMesh2 respecto al padre");

	/// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
	
	QVec finalHandMarca(6);
	finalHandMarca.inject(finalHandMarcaT,0);
	finalHandMarca.inject(finalHandMarcaR,3);
	qDebug() << "Posicion final corregida del ThandMesh2 respecto al padre" << finalHandMarca;

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
	}
	else
	{
		addTransformInnerModel("marca-segun-head", "rgbd_transform", innerModel->transform("rgbd_transform", QVec::zeros(6), "april-mug"));
	}
	
	addTransformInnerModel("marca-segun-head", "rgbd_transform", tag12Pose);
	innerModel->transform("world", QVec::zeros(6),"marca-segun-head").print("marca-segun-head en world");
	innerModel->transform("world", QVec::zeros(6),"mesh-mug").print("marca en world");
	
	
	qDebug() << "Differencia entre mano y marca visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"marca-segun-head") -
																innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
	qDebug() << "Differencia entre mano y marca visual en el SR de la mano" << innerModel->transform("mano-segun-head", QVec::zeros(6),"marca-segun-head");
																
	//drawAxis("marca-segun-head", "rgbd_transform");
		
	//Build a CHANGING temporary target position close to the real target. SHOULD CHANGE TO APPROXIMATE INCREMTANLLY THE OBJECT 
	QVec nearTarget(6,0.f);
	

	qDebug() << "initialDistance" << initialDistance << "real dist" << innerModel->transform("mano-segun-head", "marca-segun-head").norm2() ;
	
	
	//nearTarget[0] = -initialDistance; nearTarget[1] = 0 ;nearTarget[2] = 0 ;nearTarget[3] = M_PI/2;nearTarget[4] = -M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES
	nearTarget[0] = -initialDistance; nearTarget[1] = 0 ;nearTarget[2] = 0 ;nearTarget[3] = M_PI/2;nearTarget[4] = M_PI/2;nearTarget[5] = 0;  //OJJOO MARCA X REVES por lo que giro en Y al reves
	
	
	if( innerModel->transform("mano-segun-head", "marca-segun-head").norm2() > 110) 
	{
		initialDistance = initialDistance * 0.8;
		if(initialDistance < 110 ) 
			initialDistance = 109;
	}
	else
	{
		closeFingers();
 		return State::DETACH;
	}
			
		
	addTransformInnerModel("marca-segun-head-cercana", "marca-segun-head", nearTarget);
	qDebug() << "Differencia entre mano y marca cercana visual" << innerModel->transform("rgbd_transform", QVec::zeros(6),"marca-segun-head-cercana") -
																innerModel->transform("rgbd_transform", QVec::zeros(6),"mano-segun-head"); 
	qDebug() << "Differencia entre mano y marca cercana visual en el SR de la mano" << innerModel->transform("marca-segun-head-cercana", QVec::zeros(6),"mano-segun-head");
	
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
		qDebug() << __FUNCTION__ << "Sent to target in RCIS" << p;
		bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
		bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
		
} 
	catch (const Ice::Exception &ex) 
	{
		std::cout << ex << endl;
	} 
		
	tag11 = false;		
	tag12 = false;
		
 	return State::GRASP;
}


/**
 * @brief Detatch mug from table and atatch to hand
 * 
 * @return void
 */
SpecificWorker::State SpecificWorker::detach()
{
	qDebug() << __FUNCTION__ << "Detatching mug";
	try
	{	
		innermodelmanager_proxy->removeNode("mugT");
		
		qDebug() << "hola";
		
		RoboCompInnerModelManager::Pose3D pose;
		pose.x=0; pose.y=50; pose.z=110; pose.rx=0; pose.ry=0; pose.rz=M_PI;
		innermodelmanager_proxy->addTransform("mugT2","static","grabPositionHandR", pose);
		
		RoboCompInnerModelManager::meshType mesh;
		mesh.pose.x=0; mesh.pose.y=0; mesh.pose.z=0; mesh.pose.rx=1.57079; mesh.pose.ry=0; /*mesh.pose.rz=-2.62;*/	mesh.pose.rz=-1.62;	
		mesh.scaleX=100; mesh.scaleY=100; mesh.scaleZ=100; mesh.meshPath="/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/makeMeCoffee/milk.3ds";
		innermodelmanager_proxy->addMesh("mesh-milk2" , "mugT2", mesh);
		
		RoboCompInnerModelManager::Plane3D plane;
		plane.py=40; plane.ny=1; plane.width=71.25;plane.height=71.25;plane.thickness=5;plane.texture="/home/robocomp/robocomp/files/innermodel/tar36h11-12.png";	
		innermodelmanager_proxy->addPlane("mesh-mug2" , "mugT2", plane);
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ 
		std::cout << ex.text << std::endl;
	}
	catch(const Ice::Exception &ex)
	{ 
		std::cout << ex << std::endl;
	}
	
	return State::IDLE;
}

///////////////////////////////
//////////////////////////////

void SpecificWorker::drawAxis(const QString& name, const QString &parent)
{
	removeAxis(name);
	try
	{
		float OX = 0; 
		float OZ = 0;

		QVec p =innerModel->transform(parent,QVec::zeros(6),name);
		RoboCompInnerModelManager::Pose3D pose;
		pose.x=p.x() + OX ; pose.y=p.y(); pose.z=p.z() - OZ; pose.rx=p.rx(); pose.ry=p.ry(); pose.rz=p.rz();
		innermodelmanager_proxy->addTransform(name.toStdString(), "static", parent.toStdString(), pose);
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
		innermodelmanager_proxy->addPlane(name.toStdString()+"_X", name.toStdString(), planeX);
		innermodelmanager_proxy->addPlane(name.toStdString()+"_Y", name.toStdString(), planeY);
		innermodelmanager_proxy->addPlane(name.toStdString()+"_Z", name.toStdString(), planeZ);
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
	{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
	
	try{	innermodelmanager_proxy->removeNode("mugT2"); } 
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex.text << "in removeNode mugT2" << std::endl; };
}


////////////////////////////////////////////////////////////////////

void SpecificWorker::go(const QVec& t, const QVec &r)
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = t.x();
	tp.z = t.z();
	tp.y = 0;
	if( r.size() == 3 )
	{
		tp.rx = r.x(); tp.ry = r.y(); tp.rz = r.z();
		tp.onlyRot = true;
	}
	else
		tp.onlyRot = false;
	target = t;
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		//qDebug() << __FUNCTION__ << "Target " << t << " sent";
		reloj.restart();
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
		if( i.id == 11) 
		{
			//qDebug() << __FUNCTION__ << "God damn got it!";
			tag11 = true;
			tag11Pose.resize(6);
			tag11Pose[0] = i.tx;tag11Pose[1] = i.ty;tag11Pose[2] = i.tz;
			tag11Pose[3] = i.rx;tag11Pose[4] = i.ry;tag11Pose[5] = i.rz;
				
// 			InnerModelNode *nodeParent = innerModel->getNode("rgbd");
// 			InnerModelTransform *node = innerModel->newTransform("marcaShit", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
// 			nodeParent->addChild(node);
// 			mutex->lock();
// 				innerModel->updateTransformValues("marcaShit",i.tx, i.ty, i.tz, i.rx, i.ry, i.rz);	
// 			mutex->unlock();
// 			
// 			// Esto es sólo para mostrar la posición de la marca vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
// 			QVec marca2TInWorld = innerModel->transform("world", QVec::zeros(3), "marcaShit");
// 			QVec marca2RInWorld = innerModel->getRotationMatrixTo("world","marcaShit").extractAnglesR_min();
// 			QVec marca2InWorld(6);
// 			marca2InWorld.inject(marca2TInWorld,0);
// 			marca2InWorld.inject(marca2RInWorld,3);
// 			qDebug() << "Marca SHIT de la mano en el mundo vista desde la camara" << marca2InWorld;
// 			innerModel->transform("world", "mugTag").print("marca mesa RCIS");
// 			
// 				//Eliminamos el nodo creado
// 			innerModel->removeNode("marcaShit");
// 			
// 			qDebug() ;
	
		}
		if( i.id == 12) 
		{
			//qDebug() << __FUNCTION__ << "God damn got it!";
			tag12 = true;
			tag12Pose.resize(6);
			tag12Pose[0] = i.tx;tag12Pose[1] = i.ty;tag12Pose[2] = i.tz;
			tag12Pose[3] = i.rx;tag12Pose[4] = i.ry;tag12Pose[5] = i.rz;
		
// 			QVec manoApril(6);
// 			manoApril[0] = i.tx; manoApril[1] = i.ty; manoApril[2] = i.tz; manoApril[3] = i.rx; manoApril[4] = i.ry; manoApril[5] = i.rz;    
// 			
// 			qDebug() << "\n";
// 			
// 			
// 			// Inicio de los cálculos
// 				
// 			// Creamos el nodo de la marca vista desde la cámara
// 			InnerModelNode *nodeParent = innerModel->getNode("rgbd");
// 			InnerModelTransform *node = innerModel->newTransform("marcaHandInCamera3", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
// 			nodeParent->addChild(node);
// 			
// 			mutex->lock();
// 				innerModel->updateTransformValues("marcaHandInCamera3",manoApril.x(), manoApril.y(), manoApril.z(), manoApril.rx(), manoApril.ry(), manoApril.rz());	
// 			mutex->unlock();
// 
// 
// 			// Esto es sólo para mostrar la posición de la marca vista desde la cámara y desde el innermodel expresadas en el sistema de referencia del mundo
// 			QVec marca2TInWorld = innerModel->transform("world", QVec::zeros(3), "marcaHandInCamera3");
// 			QVec marca2RInWorld = innerModel->getRotationMatrixTo("world","marcaHandInCamera3").extractAnglesR_min();
// 			QVec marca2InWorld(6);
// 			marca2InWorld.inject(marca2TInWorld,0);
// 			marca2InWorld.inject(marca2RInWorld,3);
// 			qDebug() << "Marca de la mano en el mundo vista desde la camara" << marca2InWorld;
// 			
// 			QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "ThandMesh2");
// 			QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","ThandMesh2").extractAnglesR_min();
// 			QVec marcaInWorld(6);
// 			marcaInWorld.inject(marcaTInWorld,0);
// 			marcaInWorld.inject(marcaRInWorld,3);
// 			qDebug() << "ThandMesh2 en el mundo vista desde RCIS" << marcaInWorld;
// 			qDebug() << "Diferencia" <<  marca2InWorld - marcaInWorld;
// 			
// 			
// 			// Calculamos el error de la marca
// 			// Ponemos la marca vista desde la cámara en el sistma de coordenadas de la marca de la mano, si no hay error debería ser todo cero
// 			QVec visualMarcaTInHandMarca = innerModel->transform("ThandMesh2", QVec::zeros(3), "marcaHandInCamera3");
// 			QVec visualMarcaRInHandMarca = innerModel->getRotationMatrixTo("ThandMesh2","marcaHandInCamera3").extractAnglesR_min();
// 			QVec visualMarcaInHandMarca(6);
// 			visualMarcaInHandMarca.inject(visualMarcaTInHandMarca,0);
// 			visualMarcaInHandMarca.inject(visualMarcaRInHandMarca,3);
// 			qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << visualMarcaInHandMarca;
// 			
// 			// Cogemos la matriz de rotación dek tHandMesh1 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh1
// 			QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("ThandMesh2","marcaHandInCamera3");
// 			QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh2_pre","ThandMesh2");
// 				
// 			// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
// 			QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
// 			QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
// 				
// 			// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
// 			QVec handMarcaTInParent = innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2");
// 			QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
// 			
// 			// Esto es sólo para mostar como está el ThandMesh1 respecto al padre antes de las modificaciones
// 			QVec inicialHandMarca(6);
// 			inicialHandMarca.inject(handMarcaTInParent,0);
// 			inicialHandMarca.inject(handMarcaRInParentMat.extractAnglesR_min(),3);	
// 			qDebug() << "Posicion inicial del ThandMesh1 respecto al padre" << inicialHandMarca;
// 			
// 			// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
// 			QVec finalHandMarca(6);
// 			finalHandMarca.inject(finalHandMarcaT,0);
// 			finalHandMarca.inject(finalHandMarcaR,3);
// 			
// 			qDebug() << "Posicion final si se corrigiese del ThandMesh1 respecto al padre" << finalHandMarca;
// 			
// 				
// 			//Eliminamos el nodo creado
// 			innerModel->removeNode("marcaHandInCamera3");
// 	
// 			qDebug() << "\n";
// 			
// 			
// 			
// 			
			
			//qDebug() << "tag dist" << QVec::vec3(tag.tx,tag.ty,tag.tz).norm2() << (innerModel->transform("world", "rgbd_transform") - innerModel->transform("world","mugTag")).norm2();
		}
	}
}


// 		innerModel->transform("rgbd_transform", QVec::zeros(6), "marca-segun-head").print("marca-segun-head en camera");
// 	
// 		QVec targetTemp(6);
// 		targetTemp[0] = 150; targetTemp[1] = 0; targetTemp[2] = 0;
// 		targetTemp[3] = 0;   targetTemp[4] = 0; targetTemp[5] = 0;
// 		
// 		qDebug();
// 		addTransformInnerModel("target-temporal", "marca-segun-head", targetTemp);
// 		
// 		QVec A = innerModel->transform("target-temporal", QVec::zeros(6),"mano-segun-head");
// 		A.print("mano segun target-temporal");
// 		QVec B = innerModel->transform("rgbd_transform", QVec::zeros(6),"target-temporal");
// 		B.print("target-temporal segun head");
// 		QVec C = innerModel->transform("mano-segun-head", QVec::zeros(6),"rgbd_transform");
// 		C.print("head segun mano");
// 		
// 		QMat Err = innerModel->getTransformationMatrix("mano-segun-head","rgbd_transform") *
// 				   innerModel->getTransformationMatrix("rgbd_transform","target-temporal") *
// 				   innerModel->getTransformationMatrix("target-temporal","target-temporal");
// 		
// 		Err.print("Err");
