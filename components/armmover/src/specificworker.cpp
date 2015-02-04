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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};


/**
 * @brief Updates an InnerModel from values read from the robot. Reads laserData too.
 * 
 * @param inner InnerModel that is to be updated
 * @return bool
 */
bool SpecificWorker::updateInnerModel(InnerModel *inner)
{
	try
	{
		RoboCompOmniRobot::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		inner->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		
		try
		{
			RoboCompJointMotor::MotorList mList;
			RoboCompJointMotor::MotorParamsList mParamsList = jointmotor_proxy->getAllMotorParams();
			
			for(auto mp: mParamsList)
			{
					mList.push_back(mp.name);
			}
			
			RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap(mList);
			for (auto mp: mParamsList)
			{
				innerModel->updateJointValue(QString::fromStdString(mp.name), mMap.at(mp.name).pos);
			}
		}
		catch (const Ice::Exception &ex)
		{
			cout << "--> Excepción en actualizar InnerModel: JointMotor: " << ex << endl;
		}
		
	}
	catch(const Ice::Exception &ex) 
	{ 
		cout << "--> Excepción en actualizar InnerModel: Omnirobot: " << ex << endl; 
		return false; 
	}
	return true;
}
