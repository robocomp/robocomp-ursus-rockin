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
	static std::string previousAction = "";
	if (previousAction != action)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ObjectAgent.InnerModel") ;
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
		AGMMisc::publishModification(newModel, agmagenttopic, worldModel, "april");
	}
	catch(...)
	{
		exit(1);
	}
}


void SpecificWorker::newAprilTag(const tagsList &list)
{
	if (worldModel->numberOfSymbols() == 0) return;

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	bool publishModel = false;
	for (auto ap : list)
	{
		printf("%d  (%f, %f, %f)    (%f, %f, %f)\n", ap.id, ap.tx, ap.ty, ap.tz, ap.rx, ap.ry, ap.rz);
		switch(ap.id)
		{
			case 0: // EXPLORED TABLE
				if (updateTable(ap, newModel)) publishModel = true;
				break;
			case 1: // NON-EXPLORED TABLE
				if (updateTable(ap, newModel)) publishModel = true;
				break;
			case 12: // MUG
				if (updateMug(ap, newModel)) publishModel = true;
				break;
			case 13:
				if (updateMilk(ap, newModel)) publishModel = true;
				break;
			case 14:
				if (updateCoffee(ap, newModel)) publishModel = true;
				break;
		}
	}

	if (publishModel)
	{
		sendModificationProposal(worldModel, newModel);
	}
}

bool SpecificWorker::updateTable(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	return false;
	bool existing = false;

	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object")
		{
			try
			{
				const int32_t tag = str2int(symbol->getAttribute("tag"));
				if (t.id == tag)
				{
// 					QVec v(6);
// 					v(0) = t.tx;
// 					v(1) = t.ty;
// 					v(2) = t.tz;
// 					v(3) = t.rx;
// 					v(4) = t.ry;
// 					v(5) = t.rz;
// 					QVec worldRef = innerModel->transform("world", v, "rgbd");
					existing = true;
				}
			}
			catch (...)
			{
				printf("%s: %d\n", __FILE__, __LINE__);
			}
		}
	}

	if (not existing)
	{

	}

	return (not existing);
}

bool SpecificWorker::updateMug(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	bool existing = false;

	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object") {
			try {
				const int32_t tag = str2int(symbol->getAttribute("tag"));
				if (t.id == tag) {
// 					QVec v(6); v(0) = t.tx; v(1) = t.ty; v(2) = t.tz; v(3) = t.rx; v(4) = t.ry; v(5) = t.rz;
// 					QVec worldRef = innerModel->transform("world", v, "rgbd");
					existing = true;
				}
			}
			catch (...) { }
		}
	}

	if (not existing) {
		int32_t robotId = newModel->getIdentifierByType("robot");
		if (robotId == -1)
		{
			return false;
		}
		AGMModelSymbol::SPtr newMug = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newMugStatus = newModel->newSymbol("objectSt");
		newModel->addEdgeByIdentifiers(robotId, newMug->identifier, "know");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "hasStatus");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "see");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "position");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "reachable");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "reach");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "classified");
		newModel->addEdgeByIdentifiers(newMug->identifier, newMugStatus->identifier, "mug");

		const std::string tagIdStr = int2str(t.id);
		printf("--%s--\n", tagIdStr.c_str());
		newMug->attributes["tag"] = tagIdStr;
		newMug->attributes["tx"] = "1300";
		newMug->attributes["ty"] = "0";
		newMug->attributes["tz"] = "-1600";
		newMug->attributes["rx"] = "0";
		newMug->attributes["ry"] = "-3.1415926535";
		newMug->attributes["rz"] = "0";
// 		newMug->attributes["tx"] = float2str(t.tx);
// 		newMug->attributes["ty"] = float2str(t.ty);
// 		newMug->attributes["tz"] = float2str(t.tz);
// 		newMug->attributes["rx"] = float2str(t.rx);
// 		newMug->attributes["ry"] = float2str(t.ry);
// 		newMug->attributes["rz"] = float2str(t.rz);

// 		int32_t tableId = str2int(params["container"].value);
// 		if (tableId != -1)
		{
			newModel->addEdgeByIdentifiers(newMug->identifier,  7, "in");
		}
	}

	const bool forcePublishModel = not existing;
	printf("force publish by mug %d (%d)\n", forcePublishModel, t.id);
	return forcePublishModel;
}


bool SpecificWorker::updateMilk(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	bool existing = false;

	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object") {
			try {
				const int32_t tag = str2int(symbol->getAttribute("tag"));
				if (t.id == tag) {
// 					QVec v(6); v(0) = t.tx; v(1) = t.ty; v(2) = t.tz; v(3) = t.rx; v(4) = t.ry; v(5) = t.rz;
// 					QVec worldRef = innerModel->transform("world", v, "rgbd");
					existing = true;
				}
			}
			catch (...) { }
		}
	}

	if (not existing) {
		int32_t robotId = newModel->getIdentifierByType("robot");
		if (robotId == -1)
		{
			return false;
		}
		AGMModelSymbol::SPtr newMilk = newModel->newSymbol("object");
		AGMModelSymbol::SPtr newMilkStatus = newModel->newSymbol("objectSt");
		newModel->addEdgeByIdentifiers(robotId, newMilk->identifier, "know");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "hasStatus");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "see");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "position");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "reachable");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "reach");
		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "classifailed");
// 		newModel->addEdgeByIdentifiers(newMilk->identifier, newMilkStatus->identifier, "milk");

		const std::string tagIdStr = int2str(t.id);
		printf("--%s--\n", tagIdStr.c_str());
		newMilk->attributes["tag"] = tagIdStr;
		newMilk->attributes["tx"] = "1100";
		newMilk->attributes["ty"] = "0";
		newMilk->attributes["tz"] = "-1600";
		newMilk->attributes["rx"] = "0";
		newMilk->attributes["ry"] = "-3.1415926535";
		newMilk->attributes["rz"] = "0";
// 		newMilk->attributes["tx"] = float2str(t.tx);
// 		newMilk->attributes["ty"] = float2str(t.ty);
// 		newMilk->attributes["tz"] = float2str(t.tz);
// 		newMilk->attributes["rx"] = float2str(t.rx);
// 		newMilk->attributes["ry"] = float2str(t.ry);
// 		newMilk->attributes["rz"] = float2str(t.rz);

// 		int32_t tableId = str2int(params["container"].value);
// 		if (tableId != -1)
		{
			newModel->addEdgeByIdentifiers(newMilk->identifier,  7, "in");
		}
	}

	const bool forcePublishModel = not existing;
	printf("force publish by milk %d (%d)\n", forcePublishModel, t.id);
	return forcePublishModel;
}

bool SpecificWorker::updateCoffee(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{

	return false;
}



