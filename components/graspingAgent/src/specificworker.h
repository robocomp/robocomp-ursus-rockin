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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <innermodel/innermodel.h>
#include <agm.h>

/**
       \brief
       @author authorname
*/

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
	void  modelModified(const RoboCompAGMWorldModel::Event& modification);
	void  modelUpdated(const RoboCompAGMWorldModel::Node& modification);


public slots:
 	void compute();

private:
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	bool active;
	void sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel);


	QVec getObjectsLocation(AGMModelSymbol::SPtr &object);
	void sendRightArmToTargetPosition(AGMModelSymbol::SPtr &targetObject, QVec pose=QVec::vec3(0,0,0));
	void sendRightArmToTargetFullPose(AGMModelSymbol::SPtr &targetObject, QVec pose=QVec::vec3(0,0,0));

	void manageReachedObjects();



	void actionExecution();
	void action_FindObjectVisuallyInTable(bool first=false);
	void action_SetObjectReach(bool first=false);
	void action_GraspObject(bool first=false);

	void saccadic3D(QVec point, QVec axis);
	void saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz);
	void updateInnerModel();


	void setRightArmUp_Reflex();
	void setRightArm_GRASP_0_Reflex();

private:
	std::string action, backAction;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
};

#endif