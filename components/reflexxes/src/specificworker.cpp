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
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
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
	for (auto motor : newAnglesOfMotors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			
			nodo.name = motor.name;
			nodo.position = motor.angle; // posición en radianes
			nodo.maxSpeed = motor.speed; //radianes por segundo TODO Bajar velocidad.
			jointmotor_proxy->setPosition(nodo);
		} 
		catch (const Ice::Exception &ex) {	cout<<"EXCEPTION IN SET JOINT POSITION: "<<ex<<endl;}
	}
}






