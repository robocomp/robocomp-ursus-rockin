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
	
	connect(bik1Button, SIGNAL(clicked()), this, SLOT(bik1()));
	connect(bik2Button, SIGNAL(clicked()), this, SLOT(bik2()));
	connect(goHomeButton, SIGNAL(clicked()), this, SLOT(goHome()));
	
	
	plantWidget = new PlantWidget(frame, QPointF(82,457), QPointF(0,10000), QPointF(65,387), QPointF(0,-10000));
	plantWidget->show();
	statusLabel->setText("");
	state = State::IDLE;
	tag11 = false;
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");
	//innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinBIKTest.xml");
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinBIKTest2.xml");
	
	
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
		case State::LIFT_ARM:
			qDebug() << "LIFT_ARM";
			state = liftArm();
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
		go(QVec::vec3(5500,0,-5100), QVec::vec3(0,0,0));
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
	if( planningState.state == "IDLE" and initiated and (QVec::vec3(5500,0,-5000) - QVec::vec3(bState.x,0,bState.z)).norm2() < 100)
	{
		qDebug() << __FUNCTION__ << "Made it...";
		initiated = false;
		stopRobot();
		return State::IDLE;
	}
	if( tag11 == true) 
	{
		qDebug() << __FUNCTION__ << "TAG11";
		initiated = false;
		stopRobot();
		tag11 = false;
		
		QVec tagInWorld = innerModel->transform("world", QVec::vec3(tag11Pose.x(),tag11Pose.y(),tag11Pose.z()), "rgbd_transform");
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
			QVec loc = innerModel->transform("robot","mugT");
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
	qDebug() << __FUNCTION__ << "Otherwise. Should not happen" << QString::fromStdString(planningState.state) << tag11 << initiated;
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
 		return State::LIFT_ARM;
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

/**
 * @brief Sends the arms to a predefined position prior to grasping
 * 
 * @return SpecificWorker::State
 */
SpecificWorker::State SpecificWorker::liftArm()
{
	qDebug() << __FUNCTION__ << "Lifting Arm";
	try 
	{
		//QVec p = QVec::vec3(200,1100,100);
		//QVec p = innerModel->transform("robot", QVec::vec3(tag.tx,tag.ty,tag.tz), "rgbd_transform");
		QVec p = innerModel->transform("robot","mugT");
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = p.x(); pose.y = p.y(); pose.z = p.z();
		pose.rx = 0; pose.ry= 0; pose.rz= M_PI;
		RoboCompBodyInverseKinematics::WeightVector weight;
		weight.x = 1; weight.y = 1; weight.z = 1; 
		weight.rx = 1; weight.ry = 0; weight.rz = 1;
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", pose, weight, 0); 
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << std::endl; }
	catch (const Ice::Exception &ex) 
	{ std::cout << ex << std::endl; }
	
	return State::IDLE;
}

/////////////////////////////////////////////////////////////
///  TASK DEVELOPMENT
////////////////////////////////////////////////////////////

void SpecificWorker::step1()
{
	state = State::GO_KITCHEN;
}

/**
 * @brief Reposition de robot servoing the mark
 * 
 * @return void
 */

void SpecificWorker::step2()
{
	state = State::SERVOING;
}

void SpecificWorker::step3()
{

}

///////////////////////////////////////////////////

void SpecificWorker::bik1()
{
	//Close fingers
	try
	{	
		qDebug() << __FUNCTION__ << "Open fingers RCIS";
		bodyinversekinematics_proxy->setRobot(0);
		bodyinversekinematics_proxy->setFingers(0);
	} 
	catch (Ice::Exception ex) {cout <<"ERROR EN ABRIR PINZA: "<< ex << endl;}
	
	//Oriente the head
	try 
	{	
		RoboCompBodyInverseKinematics::Pose6D target;
		//we need to give the Head target in ROBOT coordinates!!!!
		QVec locA = innerModel->transform("robot", QVec::vec3(tag11Pose.x(),tag11Pose.y(),tag11Pose.z()), "rgbd_transform");
		QVec loc = innerModel->transform("world","mugT");
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
	
	usleep(100000);
	
	//Send the arm
	try 
	{
		//QVec p = innerModel->transform("robot", QVec::vec3(tag.tx,tag.ty,tag.tz), "rgbd_transform");
		QVec p = innerModel->transform("world","mugT");
		RoboCompBodyInverseKinematics::Pose6D pose;
		pose.x = p.x()+40; pose.y = p.y(); pose.z = p.z()+120;
		pose.rx =  0; pose.ry=0; pose.rz= 0;
		RoboCompBodyInverseKinematics::WeightVector weight;
		weight.x = 1; weight.y = 1; weight.z = 1; 
		weight.rx = 0; weight.ry = 0; weight.rz = 0;
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", pose, weight, 1); 
	} 
	catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
	{ std::cout << ex.text << std::endl; }
	catch (const Ice::Exception &ex) 
	{ std::cout << ex << std::endl; }
// 	
}

/**
 * @brief Visual servo following Peter Storke paper. We define an error pose as a circular transformation form desired Tip position to current Tip position.
 * Then, the error is sent to the position controller and updated again in a servo loop. the method guaratees that when the error is zero, both marks are in the same place, regardeless 
 * of the kinematic and sensor errors.
 * @return void
 */
void SpecificWorker::bik2()
{
	
	if( tag11 and tag12)
	{
	
		//Grab en el mundo antes después del sacádico
		//innerModel->transform("world", QVec::zeros(6), "grabPositionHandR").print("Grab en el mundo antes de modificar");
			
		//AprilTags Z axis points outwards the mark, X to the right and Y upwards (left hand convention)
		
		//Create hand as seen by head
		addTransformInnerModel("mano-segun-head", "rgbd_transform", tag11Pose);
		innerModel->transform("world", QVec::zeros(6), "mano-segun-head").print("mano-segun-head en world");
	
		drawAxis("mano-segun-head", "rgbd_transform");
		
		QVec correctingPose(6,0.f);
		correctingPose[3] = -M_PI/2;
		addTransformInnerModel("mano-segun-head-ok", "mano-segun-head", correctingPose);
		innerModel->transform("world", QVec::zeros(6), "mano-segun-head-ok").print("mano-segun-head-ok en world");
		
		
		//Compute hand as felt in world
		//innerModel->transform("world",QVec::zeros(6),"ThandMesh2").print("mano through arm in world");
		innerModel->transform("world",QVec::zeros(6),"handMesh2").print("mano through arm in world");
		
		//Difference should be zero
		qDebug() << "Diferencia entre felt-hand y seen-hand (should be zero)" << innerModel->transform("world", QVec::zeros(6), "mano-segun-head")-innerModel->transform("world",QVec::zeros(6),"handMesh2");
		
		// Ponemos la marca de la mano vista desde la cámara en el sistma de coordenadas de la marca de la mano, si el target se ha alcanzado error debería ser todo cero
		QVec visualMarcaTInHandMarca = innerModel->transform("handMesh2", QVec::zeros(3), "mano-segun-head");
		qDebug() << "Marca vista por la camara en el sistema de la marca de la mano (deberia ser cero si no hay errores)" << innerModel->transform("handMesh2", QVec::zeros(6), "mano-segun-head");
		
		// Cogemos la matriz de rotación dek tHandMesh1 (marca en la mano) con respecto al padre para que las nuevas rotaciones y translaciones que 
		// hemos calculado (visualMarcaInHandMarca) sean añadidas a las ya esistentes en ThandMesh1
		QMat visualMarcaRInHandMarcaMat = innerModel->getRotationMatrixTo("handMesh2","mano-segun-head");
		//QMat handMarcaRInParentMat = innerModel->getRotationMatrixTo("ThandMesh2_pre","ThandMesh2");
		
		// Multiplicamos las matrices de rotación para sumar la nueva rotación visualMarcaRInHandMarcaMat a la ya existente con respecto al padre
		//QMat finalHandMarcaRMat = handMarcaRInParentMat * visualMarcaRInHandMarcaMat;
		QMat finalHandMarcaRMat = visualMarcaRInHandMarcaMat;
		QVec finalHandMarcaR = finalHandMarcaRMat.extractAnglesR_min();
		
		// Pasamos también las translaciones nuevas (visualMarcaTInHandMarca) al padre y las sumamos con las existentes
		//QVec handMarcaTInParent = innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2");
		//QVec finalHandMarcaT = handMarcaTInParent + (handMarcaRInParentMat* visualMarcaTInHandMarca);
		QVec finalHandMarcaT = visualMarcaTInHandMarca;
	
		// Esto es sólo para mostar como está el ThandMesh2 respecto al padre antes de las modificaciones
		//innerModel->transform("ThandMesh2_pre", QVec::zeros(3), "ThandMesh2").print("Posicion inicial del ThandMesh2 respecto al padre");
	
		// Creamos el vector final con las rotaciones y translaciones del tHandMesh1 con respecto al padre
		QVec finalHandMarca(6);
		finalHandMarca.inject(finalHandMarcaT,0);
		finalHandMarca.inject(finalHandMarcaR,3);
		qDebug() << "Posicion final corregida del ThandMesh2 respecto al padre" << finalHandMarca;
	
		//Actualizamos el transform de la marca en la mano (ThandMesh1) con las rotaciones y translaciones calculadas
	//	innerModel->updateTransformValues("handMesh2",finalHandMarca.x(), finalHandMarca.y(), finalHandMarca.z(), finalHandMarca.rx(), finalHandMarca.ry(), finalHandMarca.rz());	
	
		//Escribimos por pantalla como está el grab en el mundo despues de hacer las modificaciones
		qDebug() << "Grab en el mundo despues de modificar" << innerModel->transform("world", QVec::zeros(6), "grabPositionHandR");
		
		qDebug() << "-----------------------------------------------------------------------------------------------------\n";
		
	
		/////////////////////////////////////////////
		
		/// MODOFICAR LA POSICION DEL TIP EN EL BIK AQUI
		
		/////////////////////////////////////////////
		

		// Create mark node seen from the camera
		//Aling target with world
		//tag12Pose[3] = 0;
		//tag12Pose[4] = 0;
		//tag12Pose[5] = 0;
		
		addTransformInnerModel("marca-segun-head", "rgbd_transform", tag12Pose);
		innerModel->transform("world", QVec::zeros(6),"marca-segun-head").print("marca-segun-head en world");
		innerModel->transform("world", QVec::zeros(6),"mugTag").print("marca en world");
		qDebug() << "Differencia entre marcas. Debería ser cero" << innerModel->transform("world", QVec::zeros(6),"marca-segun-head") -
																	innerModel->transform("world", QVec::zeros(6),"mugTag"); 
		
		drawAxis("marca-segun-head", "rgbd_transform");
			
		//Build a temporary target position close to the real target. SHOULD CHANGE TO APPROXIMATE INCREMTANLLY THE OBJECT 
		QVec nearTarget(6,0.f);
		nearTarget[0] = 150; nearTarget[0] = 0 ;nearTarget[0] = 0 ;nearTarget[3] = 0;nearTarget[4] = 0;nearTarget[5] = 0;
		addTransformInnerModel("marca-cercana", "marca-segun-head", nearTarget);
		
		//Open fingers
// 		try
// 		{	
// 			qDebug() << __FUNCTION__ << "Open fingers RCIS";
// 			bodyinversekinematics_proxy->setRobot(0);
// 			bodyinversekinematics_proxy->setFingers(70);
// 			
// 			usleep(100000);
// 
// 			qDebug() << __FUNCTION__ << "Open fingers real";
// 			bodyinversekinematics_proxy->setRobot(1);
// 			bodyinversekinematics_proxy->setFingers(70);
// 		} 
// 		catch (Ice::Exception ex) {cout <<"ERROR EN ABRIR PINZA: "<< ex << endl;}
		
		QVec marcaTInWorld = innerModel->transform("world","marca-cercana");
		QMat matMarcaRInWorld = innerModel->getRotationMatrixTo("world","marca-cercana");
		
		Rot3D matZeroErrorWorld(-M_PI/2.0,0,0); // Para que las rotaciones de la marca en el mundo sean cero cuando el plano de la marca está paralelo al suelo 
		//Rot3D matLateralGrip(M_PI,-M_PI/2.0,0); // Para coger un objeto desde el lateral
		
		Rot3D matLateralGrip( M_PI, -M_PI/2, 0 ); // Para coger un objeto desde la orientación dada  OJJJJJJJJJJJJJJJJJJJJJOOOOOOOOOO
					
		QMat matResul =  matMarcaRInWorld * matZeroErrorWorld * matLateralGrip.invert();
		QVec marcaRInWorld = matResul.extractAnglesR_min();	
	
		//QVec marcaRInWorld = QVec::vec3(3.1415,-1.57,0); // Orientación fija 
		QVec marcaInWorld(6);
		marcaInWorld.inject(marcaTInWorld,0);
		marcaInWorld.inject(marcaRInWorld,3);

		QVec targetInWorld = marcaInWorld;	
		if (targetInWorld[2] < 200) targetInWorld[2] = 200;   // Para que no se peque mucho la mano al pecho
	
		//innerModel->removeNode("marca");
		
// 		try 
// 		{
// 			RoboCompBodyInverseKinematics::Pose6D pose;
// 			pose.x = targetInWorld.x();pose.y = targetInWorld.y();pose.z = targetInWorld.z();
// 			pose.rx = targetInWorld.rx();pose.ry = targetInWorld.ry();pose.rz = targetInWorld.rz();
// 					
// 			RoboCompBodyInverseKinematics::WeightVector weights;
// 			weights.x = 1; 		weights.y = 1; 		weights.z = 1;
// 			weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 
// 	
// 			qDebug() << "Sent to target in RCIS" << targetInWorld;
// 			bodyinversekinematics_proxy->setRobot(0); //Para enviar al RCIS-->0 Para enviar al robot-->1
// 			bodyinversekinematics_proxy->setTargetPose6D( "RIGHTARM", pose, weights,0);
// 			
// 	} 
// 		catch (const Ice::Exception &ex) 
// 		{
// 			std::cout << ex << endl;
// 		} 

		tag11 = false;		
		tag12 = false;
 	}
		
	//compute the error pose given a proper rotation parametrization that set the angles to zero when aligned
	
	//send the target pose to the hand controller
	
}

void SpecificWorker::drawAxis(const QString& name, const QString &parent)
{
	removeAxis(name);
	try
	{
		//float OX = 50; 
		//float OZ = 50;
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
