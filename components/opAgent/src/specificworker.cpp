/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
	innerModel = new InnerModel();
	QObject::connect(cbReleaseObject, SIGNAL(clicked()),
                      this, SLOT(setValue(int)));
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


//       THE FOLLOWING IS JUST AN EXAMPLE for AGENTS
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("NameAgent.InnerModel") ;
// 		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel = new InnerModel(par.value);
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
// 		}
// 		else
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
// 			qFatal("Exiting now.");
// 		}
// 	}
// 	catch(std::exception e)
// 	{
// 		qFatal("Error reading config params");
// 	}

	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	int robotId = worldModel->getIdentifierByType("robot");
	if (robotId < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return;
	}
	
// 	innerModel->treePrint();

	actionExecution();

// 	printf("ae>\n");
}
void SpecificWorker::actionExecution()
{
		QMutexLocker locker(mutex);
	qDebug()<<"ACTION: "<<QString::fromStdString(action);

	static std::string previousAction = "";
	bool newAction = (previousAction != action);

	if (newAction)
	{
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());
	}


	if (action == "handobject")
	{
		action_HandObject(newAction);
	}
	


	if (newAction)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
	printf("actionExecution>>\n");
}	

//check distance and stop
void SpecificWorker::action_HandObject(bool newAction)
{
	// Get symbols' map
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/); //ALL THE SYMBOLS GIVEN IN THE RULE
	}
	catch(...)
	{
		printf("navigationAgent, action_HandObject: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0) {	exit(-1);  }
	}
	
	//
	if (cbReleaseObject->checkState())
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));

		int objectID =symbols["object"]->identifier;
		int robotID =symbols["robot"]->identifier;
		int personID =symbols["person"]->identifier;
		
		newModel->removeEdgeByIdentifiers(objectID, robotID, "in");
		newModel->addEdgeByIdentifiers(objectID, personID, "in");
		
		sendModificationProposal(worldModel, newModel);
	}

	
	
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
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

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event &modification)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
 
	delete innerModel;
	innerModel = AgmInner::extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge dst;
		AGMModelConverter::fromIceToInternal(modification,dst);
		AgmInner::updateImNodeFromEdge(worldModel, dst, innerModel);
	}
	
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker lockIM(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	AgmInner::updateImNodeFromEdge(worldModel, dst, innerModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker l(mutex);
	printf("<<symbolUpdated\n");
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
// 	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel, "room", true);
	printf("symbolUpdated>>\n");
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
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
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
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

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"opAgentAgent");
	}
	catch(...)
	{
		exit(1);
	}
}




