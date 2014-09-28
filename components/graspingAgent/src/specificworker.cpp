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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";

	bodyinversekinematics_proxy->goHome("RIGHTARM");

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	// STUFF
	//


	// ACTION EXECUTION
	//
	actionExecution();

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("GraspingAgent.InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
		}
		else
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}


	timer.start(Period);
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
			if (not activated)
			{
				return activate(p);
			}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::deactivateAgent()
{
		return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent()
{
}

Ice::Int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}


void SpecificWorker::modelModified(const RoboCompAGMWorldModel::Event& modification)
{
	mutex->lock();
	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	mutex->unlock();
}

void SpecificWorker::modelUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	mutex->lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	mutex->unlock();
}

bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);

		if (action == "graspobject")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic, worldModel, "graspingAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::actionExecution()
{
	static std::string previousAction = "";
	if (previousAction != action)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}

	if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable();
	}
	else if (action == "setobjectreach")
	{
		action_SetObjectReach();
	}
	else if (action == "robotmovesobjectfromcontainer")
	{
		action_RobotMovesObjectFromContainer();
	}
}

void SpecificWorker::action_FindObjectVisuallyInTable()
{
	int32_t tableId = str2int(params["container"].value);
	AGMModelSymbol::SPtr goalTable = worldModel->getSymbol(tableId);
	const float x = str2float(goalTable->getAttribute("x"));
	const float z = str2float(goalTable->getAttribute("z"));
	QVec robotRef = innerModel->transform("robot", QVec::vec3(x, 800, z), "world");
	saccadic3D(robotRef, QVec::vec3(0,-1,0));
}


void SpecificWorker::action_RobotMovesObjectFromContainer()
{
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		auto symbols = newModel->getSymbolsMap(params, "object", "c1", "c2");
		newModel->removeEdge(symbols["object"], symbols["c1"], "in");
		newModel->addEdge(   symbols["object"], symbols["c2"], "in");
		try
		{
			try
			{
				Pose6D target;
				target.x = str2float(symbols["object"]->getAttribute("x"));
				target.y = str2float(symbols["object"]->getAttribute("y"));
				target.z = str2float(symbols["object"]->getAttribute("z"));
				target.rx = str2float(symbols["object"]->getAttribute("rx"));
				target.ry = str2float(symbols["object"]->getAttribute("ry"));
				target.rz = str2float(symbols["object"]->getAttribute("rz"));
				WeightVector weights;
				weights.x = 1;
				weights.y = 1;
				weights.z = 1;
				weights.rx = 1;
				weights.ry = 0;
				weights.rz = 1;
				bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 1.);
			}
			catch (...)
			{
				printf("graspingAgent: Couldn't set RIGHTARM target (maybe a communication problem?)\n");
			}
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("graspingAgent: Couldn't publish new model\n");
		}
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}
}


void SpecificWorker::action_SetObjectReach()
{
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		auto symbols = newModel->getSymbolsMap(params, "object", "status");
		newModel->renameEdge(symbols["object"], symbols["status"], "noReach", "reach");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("graspingAgent: Couldn't publish new model\n");
		}
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}
}


void SpecificWorker::saccadic3D(QVec point, QVec axis)
{
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
	RoboCompBodyInverseKinematics::Pose6D targetSight;
	targetSight.x = tx;
	targetSight.y = ty;
	targetSight.z = tz;
	RoboCompBodyInverseKinematics::Axis axSight;
	axSight.x = axx;
	axSight.y = axy;
	axSight.z = axz;
	bool axisConstraint = false;
	float axisAngleConstraint = 0;
	try
	{
		bodyinversekinematics_proxy->stop("HEAD");
		usleep(500000);
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", targetSight, axSight, axisConstraint, axisAngleConstraint);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
}



void SpecificWorker::updateInnerModel()
{
	try
	{
		AGMModelSymbol::SPtr robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
		const float x     = str2float(robot->getAttribute("x"));
		const float z     = str2float(robot->getAttribute("z"));
		const float alpha = str2float(robot->getAttribute("alpha"));
		innerModel->updateTransformValues("robot", x, 0, z, 0, alpha, 0);
	}
	catch(...)
	{
		return;
	}
}


