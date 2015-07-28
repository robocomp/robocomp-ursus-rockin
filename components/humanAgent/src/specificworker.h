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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void structuralChange(const RoboCompAGMWorldModel::Event &modification);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void newMSKBodyEvent(const PersonList &people, const long &timestamp);

public slots:
	void compute(); 	

private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active, newBodyEvent;
	PersonList personList;
	long int timeStamp;
	QTimer timerTimeStamp;
	
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	void updatePeople();
	void updatePeopleInner();
	
	map<string,QString> dictionaryNames;
	map<string,RoboCompMSKBody::JointType> dictionaryEnum;
	map<string,RTMat> mapJointRotations;
	map<int, InnerModel*> innerModelMap;
	
	void updateInnerModel( TPerson &person, int idPerson );
	void initDictionary();
	void calculateJointRotations(TPerson &p);
	
	RTMat rtMatFromJointPosition(RTMat rS, RoboCompMSKBody::SkeletonPoint p1, RoboCompMSKBody::SkeletonPoint p2, RoboCompMSKBody::SkeletonPoint translation, int axis);
	
	bool rotarTorso(const QVec & hombroizq,const QVec & hombroder);
	
};

#endif

