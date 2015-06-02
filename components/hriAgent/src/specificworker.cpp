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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	actionExecution();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("HRIAgent.InnerModel") ;
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


void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event& modification)
{
	mutex->lock();
	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	mutex->lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	mutex->unlock();
}
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
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
		//AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic, worldModel, "hriAgent");
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

	if (action == "personclassifiesmilkpot")
	{
		action_PersonClassifiesMilkPot();
	}
	else if (action == "tellhumanaboutcoffeepot")
	{
		action_TellHumanAboutCoffeePot();
	}
	else if (action == "tellhumanaboutmug")
	{
		action_TellHumanAboutMug();
	}
	else if (action == "tellhumanabouttable")
	{
		action_TellHumanAboutTable();
	}
	else if (action == "tellhumanaboutunknownobject")
	{
		action_TellHumanAboutUnknownObject();
	}


}

void SpecificWorker::action_PersonClassifiesMilkPot()
{
	static bool first = true;
	static QTime lastTime = QTime::currentTime();
	if (lastTime.elapsed() < 5000 and not first) return;
	first = false;
	lastTime = QTime::currentTime();



	speech_proxy->say("Is this milk? What? ... I can't hear you... I guess you said yes.", false);
	while(speech_proxy->isBusy())
		usleep(100000);



	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		auto symbols = newModel->getSymbolsMap(params, "objectr", "statusr", "objecth", "statush");
		newModel->renameEdge(symbols["objectr"], symbols["statusr"], "classifailed", "milkpot");
		newModel->renameEdge(symbols["objecth"], symbols["statush"], "unclassified", "milkpot");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("hriAgent: Couldn't publish new model\n");
		}
	}
	catch(...)
	{
		printf("hriAgent: Couldn't retrieve action's parameters\n");
	}
}

void SpecificWorker::action_TellHumanAboutCoffeePot()
{
	static bool first = true;
	static QTime lastTime = QTime::currentTime();
	if (lastTime.elapsed() < 5000 and not first) return;
	first = false;
	lastTime = QTime::currentTime();



	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		auto symbols = newModel->getSymbolsMap(params, "person", "objectr", "conth");
		//
		// BEFORE SUBMITTING A NEW MODIFICATION... BE SURE WE DON'T DO IT TWICE!
		{
			for (auto objEdgIt : *(symbols["objectr"]))
			{
				if (objEdgIt->getLabel() == "eq")
				{
					// We already told him!? :-?  (Maybe we're too fast! :-D)
					return;
				}
			}
		}


		speech_proxy->say("This is a coffeepot.", false);
		while(speech_proxy->isBusy())
			usleep(100000);

		//
		AGMModelSymbol::SPtr newObjH       = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newObjHStatus = newModel->newSymbol("objectSt");
		newModel->addEdge( symbols["person"],          newObjH, "know");
		newModel->addEdge(symbols["objectr"],          newObjH, "eq");
		newModel->addEdge(           newObjH, symbols["conth"], "in");
		newModel->addEdge(           newObjH,    newObjHStatus, "hasStatus");
		newModel->addEdge(           newObjH,    newObjHStatus, "position");
		newModel->addEdge(           newObjH,    newObjHStatus, "classified");
		newModel->addEdge(           newObjH,    newObjHStatus, "coffeepot");
		newModel->addEdge(           newObjH,    newObjHStatus, "see");
		newModel->addEdge(           newObjH,    newObjHStatus, "reach");
		newModel->addEdge(           newObjH,    newObjHStatus, "reachable");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("Some error occurred when publishing the new model\n");
		}
	}
	catch(...)
	{
		printf("Some error occurred when retrieving action's parameters\n");
	}
}


void SpecificWorker::action_TellHumanAboutTable()
{
	static bool first = true;
	static QTime lastTime = QTime::currentTime();
	if (lastTime.elapsed() < 5000 and not first) return;
	first = false;
	lastTime = QTime::currentTime();



	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		speech_proxy->say("This is a table.", false);
		while(speech_proxy->isBusy())
			usleep(100000);

		auto symbols = newModel->getSymbolsMap(params, "person", "objectr", "conth");
		AGMModelSymbol::SPtr newObjH       = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newObjHStatus = newModel->newSymbol("objectSt");
		newModel->addEdge( symbols["person"],          newObjH, "know");
		newModel->addEdge(symbols["objectr"],          newObjH, "eq");
		newModel->addEdge(           newObjH, symbols["conth"], "in");
		newModel->addEdge(           newObjH,    newObjHStatus, "hasStatus");
		newModel->addEdge(           newObjH,    newObjHStatus, "position");
		newModel->addEdge(           newObjH,    newObjHStatus, "classified");
		newModel->addEdge(           newObjH,    newObjHStatus, "table");
		newModel->addEdge(           newObjH,    newObjHStatus, "see");
		newModel->addEdge(           newObjH,    newObjHStatus, "reach");
		newModel->addEdge(           newObjH,    newObjHStatus, "reachable");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("Some error occurred when publishing the new model\n");
		}
	}
	catch(...)
	{
		printf("Some error occurred when retrieving action's parameters\n");
	}
}


void SpecificWorker::action_TellHumanAboutMug()
{
	static bool first = true;
	static QTime lastTime = QTime::currentTime();
	if (lastTime.elapsed() < 5000 and not first) return;
	first = false;
	lastTime = QTime::currentTime();
	printf("SpecificWorker::action_TellHumanAboutMug()\n");



	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		try
		{
			speech_proxy->say("This is a mug.", false);
			while(speech_proxy->isBusy())
				usleep(100000);
		}
		catch(...)
		{
			printf("couldn't speak!!");
		}

		auto symbols = newModel->getSymbolsMap(params, "person", "objectr", "conth");
		AGMModelSymbol::SPtr newObjH       = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newObjHStatus = newModel->newSymbol("objectSt");
		newModel->addEdge( symbols["person"],          newObjH, "know");
		newModel->addEdge(symbols["objectr"],          newObjH, "eq");
		newModel->addEdge(           newObjH, symbols["conth"], "in");
		newModel->addEdge(           newObjH,    newObjHStatus, "hasStatus");
		newModel->addEdge(           newObjH,    newObjHStatus, "position");
		newModel->addEdge(           newObjH,    newObjHStatus, "classified");
		newModel->addEdge(           newObjH,    newObjHStatus, "mug");
		newModel->addEdge(           newObjH,    newObjHStatus, "see");
		newModel->addEdge(           newObjH,    newObjHStatus, "reach");
		newModel->addEdge(           newObjH,    newObjHStatus, "reachable");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("Some error occurred when publishing the new model\n");
		}
	}
	catch(...)
	{
		printf("Some error occurred when retrieving action's parameters\n");
	}
}


void SpecificWorker::action_TellHumanAboutUnknownObject()
{
	static bool first = true;
	static QTime lastTime = QTime::currentTime();
	printf("void SpecificWorker::action_TellHumanAboutUnknownObject()%d\n", __LINE__);
	if (lastTime.elapsed() < 5000 and not first) return;
	printf("void SpecificWorker::action_TellHumanAboutUnknownObject()%d\n", __LINE__);
	first = false;
	lastTime = QTime::currentTime();


	speech_proxy->say("Look at this object I couldn't classify, please.", false);
	while(speech_proxy->isBusy())
		usleep(100000);

	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{


		auto symbols = newModel->getSymbolsMap(params, "person", "objectr", "conth");
		AGMModelSymbol::SPtr newObjH       = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newObjHStatus = newModel->newSymbol("objectSt");
		newModel->addEdge( symbols["person"],          newObjH, "know");
		newModel->addEdge(symbols["objectr"],          newObjH, "eq");
		newModel->addEdge(           newObjH, symbols["conth"], "in");
		newModel->addEdge(           newObjH,    newObjHStatus, "hasStatus");
		newModel->addEdge(           newObjH,    newObjHStatus, "unclassified");
		newModel->addEdge(           newObjH,    newObjHStatus, "position");
		newModel->addEdge(           newObjH,    newObjHStatus, "reachable");
		try
		{
			sendModificationProposal(worldModel, newModel);
		}
		catch(...)
		{
			printf("Some error occurred when publishing the new model\n");
		}
	}
	catch(...)
	{
		printf("Some error occurred when retrieving action's parameters\n");
	}
}



