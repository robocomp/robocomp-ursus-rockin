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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else  	qFatal("Exiting now.");
	}catch(std::exception e) { qFatal("Error reading Innermodel param");}
	
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	// ACTUALIZAMOS EL INNERMODEL
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		
	}catch (const Ice::Exception &ex){ cout<<"--> Excepción en actualizar InnerModel";}
}

/**
 * \brief 
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
 * TODO UTILIZAR LAS CLASES DE REFLEXX !!!
 */ 
void SpecificWorker::setJointPosition(const MotorAngleList &newAnglesOfMotors)
{
	// 1) SACAMOS VALORES ACTUALES DE LOS MOTORES: (aprovechamos y sacamos motores
	QVec firstAngles;
	QStringList motors;
	for(auto motor : newAnglesOfMotors)
	{
		float angle = innerModel->getJoint(QString::fromStdString(motor.name))->getAngle();
		firstAngles.push_back(angle);
		motors<<QString::fromStdString(motor.name);
	}
	//  2) SACAMOS VALORES FINALES DE LOS MOTORES
	QVec finalAngles;
	for(auto motor : newAnglesOfMotors)
	{
		float angle = motor.angle;
		finalAngles.push_back(angle);
	}
	
	QList<QVec> jointValues;
	jointValues.append(firstAngles);
	jointValues.append(finalAngles);
	
	Reflexx *reflexx = new Reflexx(jointmotor_proxy, jointValues, motors);
	reflexx->start();
	qDebug() << __FUNCTION__ << "Waiting for Reflexx...";
	reflexx->wait(5000);
	qDebug()<<"FIN DEL WAIT";
}






