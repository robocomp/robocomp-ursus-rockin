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
	INITIALIZED		= false;
	COMPUTE_READY	= false;

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
/**
 * \brief setParams method. It stores the innermodel file
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else  	qFatal("Exiting now.");
	}catch(std::exception e) { qFatal("Error reading Innermodel param");}
	
//	QMutexLocker ml(mutex);
//	INITIALIZED = true;
	qDebug()<<"INITIALIZED: "<<INITIALIZED;
	
	timer.start(Period);
	
	return true;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief slot compute
 */ 
void SpecificWorker::compute()
{
	
	qDebug()<<"Hola desde Compute";

		
	updateInnerModel();
	

	QMutexLocker ml(mutex);
	qDebug()<<COMPUTE_READY;
	if(COMPUTE_READY == true)
	{
		qDebug()<<"READY";
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this method returns true when the motors are in their goal position or false
 * if they don't reach their goal position.
 * @param anglesOfMotors the list of the motors with the goal positions.
 * @return bool
 */ 
bool SpecificWorker::getStatePosition(const MotorAngleList &anglesOfMotors)
{
	bool todoOk = true;
    RoboCompJointMotor::MotorState motorState;
	
	for(auto motor : anglesOfMotors)
	{
		try
		{
			motorState = jointmotor_proxy->getMotorState(motor.name);
			if(fabs(motorState.pos - motor.angle)>0.1)
				todoOk = false;
		} 
		catch (const Ice::Exception &ex) {	cout<<"EXCEPTION IN GET STATE POSITION: "<<ex<<endl;}
	}
	return todoOk;
}
/**
 * \brief This method stored the motors's names, the initial position of the motors and the goal
 * position of them. When all is prepared, it raises up the flag READY.
 * @param newAnglesOfMotors an structure with the name of the motors and the value os them.
 */ 
void SpecificWorker::setJointPosition(const MotorAngleList &newAnglesOfMotors)
{
	qDebug()<<"YEAH";
	QMutexLocker ml(mutex);
	if(INITIALIZED == true)
	{
		// 1) SACAMOS VALORES ACTUALES DE LOS MOTORES: (aprovechamos y sacamos motores disponibles)
		QVec firstAngles;
		for(auto motor : newAnglesOfMotors)
		{
			float angle = innerModel->getJoint(QString::fromStdString(motor.name))->getAngle();
			firstAngles.push_back(angle);
			selectedMotors<<QString::fromStdString(motor.name);
		}
		//  2) SACAMOS VALORES FINALES DE LOS MOTORES
		QVec finalAngles;
		for(auto motor : newAnglesOfMotors)
		{
			float angle = motor.angle;
			finalAngles.push_back(angle);
		}
		jointValues.append(firstAngles);
		jointValues.append(finalAngles);
		
		COMPUTE_READY = true;
		
		qDebug()<<"||------------------------------------------------";
		qDebug()<<"|| setJointPosition: jointValues-->"<<jointValues;
		qDebug()<<"||------------------------------------------------";
	}
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this method updates the innermodel of the component
 */ 
void SpecificWorker::updateInnerModel()
{
	// ACTUALIZAMOS EL INNERMODEL
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		
	}catch (const Ice::Exception &ex){ 
		cout<<"--> Excepción en actualizar InnerModel";
	}
}



