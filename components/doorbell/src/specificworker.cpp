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

	connect(pushButton, SIGNAL(clicked()), this, SLOT(doorbellRang()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	printf("\n --- compute ---\n");
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	AGMModelSymbol::SPtr outside;
	try
	{
		outside = newModel->getSymbol(17);
	}
	catch(...)
	{
		printf("Can't find node 17. Waiting for executive?\n");
		return;
	}

	printf("got 17\n");
	for (AGMModelSymbol::iterator edge_itr=outside->edgesBegin(newModel); edge_itr!=outside->edgesEnd(newModel); edge_itr++)
	{
		if ((*edge_itr)->getLabel() == "in")
		{
			printf("got in\n");
			int second = (*edge_itr)->getSymbolPair().first;
			const AGMModelSymbol::SPtr &symbolPerson = newModel->getSymbolByIdentifier(second);
			if (symbolPerson->symbolType == "person")
			{
				printf("got person\n");
				for (AGMModelSymbol::iterator edge_itr2=symbolPerson->edgesBegin(newModel); edge_itr2!=symbolPerson->edgesEnd(newModel); edge_itr2++)
				{
					if ((*edge_itr2)->getLabel() == "personIs")
					{
						printf("got person type edge\n");
						int third = (*edge_itr2)->getSymbolPair().second;
						const AGMModelSymbol::SPtr &symbolPersonType = newModel->getSymbolByIdentifier(third);
						if (symbolPersonType->symbolType == "unknownPerson")
						{
							printf("got unknown!!!\n");
							
							
							string symbolPersonType_str = getPersonType();


							symbolPersonType->symbolType = symbolPersonType_str;
							
							if (symbolPersonType_str == "postman" or symbolPersonType_str == "deliMan")
							{
								AGMModelSymbol::SPtr packageObj = worldModel->newSymbol("object");
								AGMModelSymbol::SPtr packageSts = worldModel->newSymbol("unknownPerson");
								
								worldModel->addEdgeByIdentifiers(1, packageObj->identifier, "know");
								worldModel->addEdgeByIdentifiers(packageObj->identifier, symbolPerson->identifier, "in");
								worldModel->addEdgeByIdentifiers(packageObj->identifier, packageSts->identifier, "hasStatus");
							}
							else if (symbolPersonType_str == "extranger")
							{
							}
							else if (symbolPersonType_str == "medicineDoctor")
							{
							}

	
							sendModificationProposal(worldModel, newModel);
						}
					}
				}
			}
		}
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
 
	agmInner.setWorld(worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
	mutex->unlock();
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
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"doorbellAgent");
	}
	catch(...)
	{
		exit(1);
	}
}


void SpecificWorker::doorbellRang()
{
	printf("Trying to include a human in the model, given that the door bell rang...\n");

	AGMModelSymbol::SPtr newSymbolPerson =  worldModel->newSymbol("person");
	
	
	AGMModelSymbol::SPtr typeSymbolPerson = worldModel->newSymbol("unknownPerson");
	
	worldModel->addEdgeByIdentifiers(1, newSymbolPerson->identifier, "know");
	worldModel->addEdgeByIdentifiers(newSymbolPerson->identifier, 17, "in");
	worldModel->addEdgeByIdentifiers(newSymbolPerson->identifier, typeSymbolPerson->identifier,"personIs");

	AGMModel::SPtr newModel(new AGMModel(worldModel));
	sendModificationProposal(worldModel, newModel);

}



string SpecificWorker::getPersonType()
{
// 	string symbolPersonType_str = "postman";
// 	string symbolPersonType_str = "deliMan";
// 	string symbolPersonType_str = "extranger";
	string symbolPersonType_str = "medicineDoctor";
}


