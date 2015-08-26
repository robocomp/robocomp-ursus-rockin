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



// THIS IS AN AGENT




#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <agm.h>


class QMutexDebug : public QMutex
{
public:
	QMutexDebug(QMutex::RecursionMode recursionMode, QString name) : QMutex(recursionMode)
	{
		myname = name;
		caller = "<none yet>";
	}
	void setCallerID(QString callerID)
	{
		caller = callerID;
	}
	QString getCallerID() { return caller; }
	QString getMutexName() { return myname; }
private:
	QString caller;
	QString myname;
};

class QMutexLockerDebug
{
public:
	QMutexLockerDebug(QMutexDebug *mutex, QString callerID)
	{
		this->mutex = mutex;
// 		printf("Mutex %s is being accessed by %s\n", mutex->getMutexName().toStdString().c_str(), callerID.toStdString().c_str());
		locker = new QMutexLocker(mutex);
		mutex->setCallerID(callerID);
// 		printf("Mutex %s was blocked by %s\n", mutex->getMutexName().toStdString().c_str(), callerID.toStdString().c_str());
	}
	~QMutexLockerDebug()
	{
		mutex->setCallerID(QString("was released by ")+mutex->getCallerID());
		delete locker;
	}
private:
	QMutexDebug *mutex;
	QString caller;
	QMutexLocker *locker;
};



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	bool activateAgent(const ParameterMap& prs);
	bool deactivateAgent();
	StateStruct getAgentState();
	ParameterMap getAgentParameters();
	bool setAgentParameters(const ParameterMap& prs);
	void  killAgent();
	Ice::Int uptimeAgent();
	bool reloadConfigAgent();
	void  structuralChange(const RoboCompAGMWorldModel::Event& modification);
	void  symbolUpdated(const RoboCompAGMWorldModel::Node& modification);
	void  edgeUpdated(const RoboCompAGMWorldModel::Edge& modification);


public slots:
	void compute();

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel);


	QVec getObjectsLocationInRobot(std::map<std::string, AGMModelSymbol::SPtr> &symbols, AGMModelSymbol::SPtr &object);
	QVec fromRobotToRoom(std::map<std::string, AGMModelSymbol::SPtr> &symbols, const QVec vector);
	int sendRightArmToPose(QVec p);

	void manageReachedObjects();



	void actionExecution();
	void action_FindObjectVisuallyInTable(bool first=false);
	void action_SetObjectReach(bool first=false);
	void action_GraspObject(bool first=false);

	void directGazeTowards(AGMModelSymbol::SPtr symbol);
	void saccadic3D(QVec point, QVec axis);
	void saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz);


	bool isObjectType(AGMModel::SPtr model, AGMModelSymbol::SPtr node, const std::string &t);
	float distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr symbol);
// 	float distanceToPolygon(QVec reference, QVec position, std::string polygon_str);

	void setRightArmUp_Reflex();

private:
	
	QMutexDebug *mutex_AGM_IM;
	
	std::string action, backAction;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active;
	
};

#endif

