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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include "reflexxes.h"



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
			SpecificWorker		(MapPrx& mprx);	
			~SpecificWorker		();
	bool 	setParams			(RoboCompCommonBehavior::ParameterList params);

	bool 	getStatePosition	(const MotorAngleList &anglesOfMotors);
	void 	setJointPosition	(const MotorAngleList &newAnglesOfMotors);

public slots:
	void 	compute				(); 	

private:
	InnerModel						*innerModel;	
	ReflexxesAPI                	*RML;        
	RMLPositionInputParameters  	*IP;           
	RMLPositionOutputParameters 	*OP;                      
	RMLPositionFlags				Flags;   
	
	bool							INITIALIZED;
	bool							COMPUTE_READY;
	int								ResultValue;    
	int 							NUMBER_OF_DOFS;        
	QHash<QString,int> 				hashMotors;
	QList<QVec> 					jointValues;
	QStringList 					selectedMotors;
	QTimer							timer;
	
private:
	void 	setSyncPosition		(const RoboCompJointMotor::MotorGoalPositionList &listGoals);
	void 	updateMotorState	(RoboCompJointMotor::MotorStateMap motors);
	void 	updateInnerModel	();
	//void virtual run();
};

#endif

