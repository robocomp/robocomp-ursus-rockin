/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	//Iniciamos con estado igual a IDLE:
	this->state = IDLE;
	cout<<"---------------------\nEstado inicial: "<<state<<"\n---------------------"<<endl;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	switch(this->state)
	{
		case IDLE:
			if (this->currentTarget->getState() == true)
			{
				this->state = TARGET_ARRIVE;
				cout<<"Ha llegado un TARGET"<<endl;
			}
			break;
		/*case "TARGET_ARRIVE":
			//compruebo target --> pesos
			if pesos == 0
				state.change("INIT_TRASLACION")
				else
					state.change("INIT_ROTACION");
			break;
		case "INIT_TRASLACION":
			bodyinversekinematics_proxy->setTargetPose6D(target.current.toPose6D) 
			state = waitTraslacion;
			break;
		case "INIT_ROTACION":
			//Hay que llamar metodo
			bodyinversekinematics_proxy->setTargetPose6D(target.current.toPose6D) 
			state = waitRotacion;
			break;
		case "WAIT_TRASLACION":
			if bodyinversekinematics_proxy->getState(target.currect.part)==true
				//llamamos metodo2
				
			break;*/
	}
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}*/
}


void SpecificWorker::metodo1_traslacion()
{/*
	Tag tag11;
	//Create hand as seen by head
	if( localTags.existId(11,tag11) == true)
	{
		addTransformInnerModel("mano-segun-head", "rgbd_transform", tag11.pose);
	}
	else
		qDebug() << "No veo el 11";*/
}

void SpecificWorker::metodo2_rotacion()
{
	/*
	qDebug() << __FUNCTION__;
// 	if( bikState.finish == false )
// 		return State::GRASP;			//REPLACE BY A POSITION CONTROL
		
	//AprilTags Z axis points outwards the mark, X to the right and Y upwards (left hand convention)
	Tag tag11;
	//Create hand as seen by head
	if( localTags.existId(11,tag11) == true)
	{
		addTransformInnerModel("mano-segun-head", "rgbd_transform", tag11.pose);
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
	
	Tag tag12;
	if( localTags.existId(12, tag12) == true)
	{
		addTransformInnerModel("marca-segun-head", "rgbd_transform", tag12.pose);
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
			return State::GRASP;
	}	
	else
	{
		graspFingers();
		return State::DETACH_TO_GET;
	}	*/
}

void SpecificWorker::setFingers(const float d)
{
	bodyinversekinematics_proxy->setFingers(d);
}

void SpecificWorker::setRobot(const int type)
{
	bodyinversekinematics_proxy->setRobot(type);
}

TargetState SpecificWorker::getState(const string &part)
{
	return bodyinversekinematics_proxy->getState(part);
}

void SpecificWorker::setNewTip(const string &part, const string &transform, const Pose6D &pose)
{
	bodyinversekinematics_proxy->setNewTip(part, transform, pose);
}

void SpecificWorker::stop(const string &part)
{
	bodyinversekinematics_proxy->stop(part);
}

void SpecificWorker::goHome(const string &part)
{
	bodyinversekinematics_proxy->goHome(part);
}

void SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights, const float radius)
{
	QMutex mutex;
	mutex.lock();//MUTEX
	if (this->state == IDLE)
	{
		cout<<"Recibido target"<<endl;
		//this->targetPendiente = target; //copiamos target
	}
	mutex.unlock();
}

void SpecificWorker::advanceAlongAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	bodyinversekinematics_proxy->advanceAlongAxis(bodyPart, ax, dist);
}

void SpecificWorker::pointAxisTowardsTarget(const string &bodyPart, const Pose6D &target, const Axis &ax, const bool &axisConstraint, const float axisAngleConstraint)
{
	bodyinversekinematics_proxy->pointAxisTowardsTarget(bodyPart, target, ax, axisConstraint, axisAngleConstraint);
}

void SpecificWorker::setJoint(const string &joint, const float position, const float maxSpeed)
{
	bodyinversekinematics_proxy->setJoint(joint, position, maxSpeed);
}




void SpecificWorker::newAprilTag(const tagsList &tags)
{
	//Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	std::cout<<"April Tags: "<<tags.size()<<endl;
	this->tags = tags;
}




