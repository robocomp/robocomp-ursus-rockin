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


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	virtual MotorParamsList getAllMotorParams();
	virtual void getAllMotorState(MotorStateMap &mstateMap);
	virtual MotorParams getMotorParams(const string &motor);
	virtual MotorState getMotorState(const string &motor);
	virtual void setSyncVelocity(const MotorGoalVelocityList &listGoals);
	virtual void setZeroPos(const string &name);
	virtual BusParams getBusParams();
	virtual void setSyncZeroPos();
	virtual void setSyncPosition(const MotorGoalPositionList &listGoals);
	virtual MotorStateMap getMotorStateMap(const MotorList &mList);
	virtual void setPosition(const MotorGoalPosition &goal);
	virtual void setVelocity(const MotorGoalVelocity &goal);


public slots:
	void compute(); 	

private:
};

#endif

