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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	//Iniciamos con estado igual a IDLE:
	this->stateMachine = State::IDLE;
	
 	this->innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ursus_errors.xml");

	this->rightHand = new VisualHand(this->innerModel);
	this->leftHand = new VisualHand(this->innerModel);
	
#ifdef USE_QTGUI
	this->osgView = new OsgView(this);
	this->innerViewer = new InnerModelViewer(this->innerModel, "root", this->osgView->getRootGroup(), true);
	show();
#endif
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

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										SLOTS DEL PROGRAMA													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Slot COMPUTE
 * Bucle ejecutado por el hilo del programa. Se encarga de actualizar el innermodel de la clase y de pintar la
 * posicion del target que ha llegado y la posición de la marca que esta viendo la camara.
 * MAQUINA DE ESTADOS:
 * 	-IDLE: estado en espera. Inicialmente el trueTarget se crea con estado IDLE, indicando que no tiene nada
 * 		   Pero cuando cambia a WAITING indica que ha llegado un target que espera ser ejecutado. En ese momento 
 * 		   pasamos a INIT_BIK.
 * 	-INIT_BIK: independientemente de los pesos del trueTarget, enviamos el target al BIK para que se ejecute.
 * 			   Luego pasamos al estado WAIT_BIK.
 * 	-WAIT_BIK: esperamos a que el brazo deje de moverse para empezar con la correccion. Cuando se pare pasamos
 * 			   al estado CORRECT_ROTATION o CORRECT_TRASLATION dependiendo de los pesos del target.
 * 	-CORRECT_ROTATION: corrige los errores de la pose del tip del robot, arreglando tanto traslacion como rotacion.
 * 	-CORRECT_TRASLATION: corrige solo los errores de traslacion del tip.
 */ 
void SpecificWorker::compute()
{
	this->actualizarInnermodel();
	const Pose6D tt = this->trueTarget.getPose6D();
	this->innerModel->updateTransformValues("target", tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
	const Pose6D p = *(this->rightHand);
	this->innerModel->updateTransformValues("hand", p.x, p.y, p.z, p.rx, p.ry, p.rz);

	QMutexLocker ml(&this->mutex);
	switch(this->stateMachine)
	{
		case State::IDLE:
			std::cout<<"ESTADO IDLE.\n"<<std::endl;
			// Estado inicial en espera hasta que trueTarget tenga un valor aceptable y este esperando a
			// ser ejecutado. Cambiamos a INIT_BIK
			if (this->trueTarget.getState() == Target::State::WAITING)
			{
				this->stateMachine = State::INIT_BIK;
				printf("Ha llegado un TARGET: (%f, %f, %f,   %f, %f, %f)",tt.x, tt.y, tt.z, tt.rx, tt.ry, tt.rz);
			}
		break;
		//---------------------------------------------------------------------------------------------	
		case State::INIT_BIK:
			std::cout<<"ESTADO INIT_BIK.\n"<<std::endl;
			// Nos da igual si el target tiene en cuenta las rotaciones o las traslaciones, lo enviamos al BIK
			// que ya se encargará de manejar los dos casos.
			try
			{
				this->bodyinversekinematics_proxy->setTargetPose6D(this->trueTarget.getBodyPart(), this->trueTarget.getPose6D(), this->trueTarget.getWeights(), 250);				
			}catch (const Ice::Exception &ex)
			{ 
				std::cout<<"EXCEPCION EN SET TARGET POSE 6D --> INIT BIK: "<<ex<<std::endl;
			}
			
			this->trueTarget.changeState(Target::State::IN_PROCESS); // El trueTarget pasa a estar siendo ejecutado:
			this->stateMachine = State::WAIT_BIK; // Esperamos a que el BIK termine.
		break;
		//---------------------------------------------------------------------------------------------				
		case State::WAIT_BIK:
			std::cout<<"ESTADO WAIT_BIK.\n"<<std::endl;
			// Espermos a que el BIK termine de mover el brazo para comenzar con las correcciones.
			// Dependiendo de si las rotaciones pesan o no tendremos dos metodos de correccion:
			if(this->bodyinversekinematics_proxy->getState(this->trueTarget.getBodyPart()).finish == true)
			{
				qDebug()<<"---> El BIK ha terminado.";

				const WeightVector weights = this->trueTarget.getWeights();
				if(weights.rx==0 and weights.ry==0 and weights.rz==0)
					this->stateMachine = State::CORRECT_TRASLATION;
				else
					this->stateMachine = State::CORRECT_ROTATION;
			}
		break;
		//---------------------------------------------------------------------------------------------						
		case State::CORRECT_TRASLATION:
			std::cout<<"ESTADO CORRECT_TRASLATION.\n"<<std::endl;
			
			if(this->correctTraslation()==true)
			{
				// Si la correccion ha terminado y hay un target esperando
				if (this->nextTarget.getState() == Target::State::WAITING)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					this->trueTarget = this->nextTarget;
					qDebug()<<this->trueTarget.getState();
				}
				this->stateMachine = State::IDLE;
			}	
		break;
		//---------------------------------------------------------------------------------------------	
		case State::CORRECT_ROTATION:
			std::cout<<"ESTADO CORRECT_ROTATION.\n"<<std::endl;
			
			if(this->correctTraslation()==true)
			{
				// Si la correccion ha terminado y el target ha sido resuelto, pasamos al siguiente target.
				if (this->nextTarget.getState() == Target::State::WAITING)
				{
					std::cout<<"--> Correccion completada.\nPasamos al siguiente target:\n";
					this->trueTarget = this->nextTarget;
					qDebug()<<this->trueTarget.getState();
				}
				this->stateMachine = State::IDLE;
			}
			
		break;
		//---------------------------------------------------------------------------------------------			
		default:
			break;
	}

	
#ifdef USE_QTGUI
	if (this->innerViewer)
	{
		this->innerViewer->update();
		this->osgView->autoResize();
		this->osgView->frame();
	}
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 											METODOS PRIVADOS												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo CORRECT TRASLATION.
 * Se encarga de corregir los errores de traslacion del tip del robot (donde esta realmente la mano). Calcula el
 * error entre la posicion en la que deberia estar (donde el cree que esta, internalPose) y la posicion en la que
 * verdaderamente esta (la pose que esta viendo la camara RGBD). 
 * 		- Si el error es miserable no hace nada e indica que el target ha sido resueltoy devolvemos TRUE. 
 * 		- Si el error es superior al umbral, calcula la pose a la que debe mover la mano y la manda al BIK.
 *		  Quedamos en espera de que el BIK termine de mover el brazo y devolvemos FALSE.
 * TODO MIRAR UMBRAL.
 *@return bool TRUE si el target esta perfecto o FALSE si no lo esta. 
 */ 
bool SpecificWorker::correctTraslation()
{
	float umbralError = 0.01;
	Pose6D error = this->rightHand->getError();
	error.rx = error.ry = error.rz = 0; //anumalos los errores de rotacion
	printf("ERROR: (%f, %f, %f,    %f, %f, %f)", error.x, error.y, error.z, error.rx, error.ry, error.rz);
	
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	QVec e = QVec::vec6(error.x, error.y, error.z, error.rx, error.ry, error.rz);
	if(e.norm2()<0.01)
	{
		this->trueTarget.changeState(Target::State::RESOLVED);
		return true;
	}
	
	// Corregimos error de traslacion entre la pose interna (la que cree) y el target:
	Pose6D corregir;
	corregir.x = this->rightHand->getInternalPose()+error.x;
	corregir.y = this->rightHand->getInternalPose()+error.y;
	corregir.z = this->rightHand->getInternalPose()+error.z;
	
	//Llamamos al BIK con el nuevo target corregido y esperamos
	this->bodyinversekinematics_proxy->setTargetPose6D(this->trueTarget.getBodyPart(), corregir, this->trueTarget.getWeights(), 250);
	while(this->bodyinversekinematics_proxy->getState(this->trueTarget.getBodyPart()).finish != true);
	
	return false;
}


bool SpecificWorker::correctRotation()
{
	Pose6D error = this->rightHand->getError();
	printf("ERROR: (%f, %f, %f,    %f, %f, %f)", error.x, error.y, error.z, error.rx, error.ry, error.rz);
	
	
	
	
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
 		//drawAxis("marca		if()
-segun-head", "world");
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

void SpecificWorker::actualizarInnermodel()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción en actualizar InnerModel: (i)";
	}

	try
	{
		RoboCompOmniRobot::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		try
		{
			innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		}
		catch (const Ice::Exception &ex)
		{
			cout<<"--> Exception updating transform values: "<<ex<<endl;
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Excepción reading OmniRobot: "<<ex<<endl;
	}
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								METODOS DE LA INTERFAZ DEL COMPONENTE										   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
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
	QMutexLocker ml(&mutex);
	//if (this->stateMachine == State::IDLE)
	//{
		cout<<"Recibido target"<<endl;
		if(this->trueTarget.getState()==Target::State::IDLE)	
		{
			this->trueTarget.changeBodyPart(bodyPart);
			this->trueTarget.changePose6D(target);
			this->trueTarget.changeWeights(weights);
			this->trueTarget.changeState(Target::State::WAITING);
		}
		else
		{
			this->nextTarget.changeBodyPart(bodyPart);
			this->nextTarget.changePose6D(target);
			this->nextTarget.changeWeights(weights);
			this->nextTarget.changeState(Target::State::WAITING);
		}
	//}
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


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								METODOS A LOS QUE SE SUBSCRIBE												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	//Recibimos las marcas que la camara esta viendo: marca mano y marca target.
	for (auto tag : tags)
	{
		if (tag.id == 25)
		{
			printf("RIGHT HAND SEEN: (tag id %d)\n", tag.id);
			QMutexLocker l(&this->mutex);
			this->rightHand->setVisualPose(tag);
			//Calculamos la pose interna de la mano en este momento
			innerModel->transform("root", "grabPositionHandR")
			this->rightHand->setInternalPose(pose);
		}
		else if (tag.id == 24)
		{
			printf("LEFT HAND SEEN: (tag id %d)\n", tag.id);
			QMutexLocker l(&this->mutex);
			this->leftHand->setVisualPose(tag);
		}
	}
}





