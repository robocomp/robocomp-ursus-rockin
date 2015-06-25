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
	setPeriod(1000);
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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("AgmInnerAgent.InnerModel") ;
		qDebug()<<"hello";
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

void SpecificWorker::compute()
{

// 	AGMModelPrinter::printWorld(worldModel);
// 	qDebug()<<"\t\t************************";
// 	innerModel->treePrint();
// 	qDebug()<<"\t\t************************";
	
	if (worldModel->numberOfSymbols()>0)
	{
		qDebug()<<"numberOfSymbols"<<worldModel->numberOfSymbols();
		qDebug()<<"************************";
		include_im("world",20);
		qDebug()<<"************************";
		qDebug()<<"numberOfSymbols"<<worldModel->numberOfSymbols();		
		//innerModel->treePrint();
		qDebug()<<"************************";
		//AGMModelPrinter::printWorld(worldModel);
		
		
// 		printf("sending modification!\n");
// 		AGMModel::SPtr newModel(new AGMModel(worldModel));		
// 		AGMModelPrinter::printWorld(newModel);
// 		sendModificationProposal(worldModel, newModel);
		
	}
	
}	

void SpecificWorker::include_im(QString idInnerModelNode, int idSymbol)
{
	InnerModelNode *node=innerModel->getNode(idInnerModelNode);
	if (node==NULL)
	{
		qDebug()<<"node"<<idInnerModelNode<<"doesn't exist";
		qFatal("abort");
	}
	AGMModelSymbol::SPtr symbol;
	
	try
	{
		symbol =worldModel->getSymbolByIdentifier(idSymbol);
	}
	catch (AGMModelException e )	
	{
		std::cout<<e.what();
		qFatal("abort");
	}
	
	
	//int n=20;
// 	recorrer(node,n);
	int n=symbol->identifier;
	innerToAGM(node,n);
	
}
void SpecificWorker::recorrer(InnerModelNode *node, int &n)
{
	
	QList<InnerModelNode*>::iterator i;
	int p=n;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{		
		n++;
		
	 	qDebug()<<node->id<<"link"<<(*i)->id<<p<<"--"<<n;
		recorrer(*i,n);	
	}
	
	
}

void SpecificWorker::innerToAGM(InnerModelNode* node ,  int &id)
{
	QList<InnerModelNode*>::iterator i;	
	int p=id;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		id++;
		qDebug()<<node->id<<"link"<<(*i)->id<<p<<"--"<<id;
		std::map<std::string, std::string> attrs;
		attrs.insert ( std::pair<std::string,std::string>("name",(*i)->id.toStdString()) );
		AGMModelSymbol::SPtr newSym;
		if (worldModel->getIndexByIdentifier(id)==-1 )
		{
			qDebug()<<"newSymbol"<<id;
			 newSym = worldModel->newSymbol(id,"transform",attrs);		
		}
		else
		{
			qDebug()<<"exist id"<<id;
			 newSym =worldModel->getSymbolByIdentifier(id);		
		}
		
		//edge 
		std::map<std::string, std::string> linkAttrs;

		linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str((*i)->getTr().x())) );
		linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str((*i)->getTr().y())) );
		linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str((*i)->getTr().z())) );
		linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str((*i)->getRxValue())));
		linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str((*i)->getRyValue())));
		linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str((*i)->getRzValue())));
		
		if (worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs))
			qDebug()<<"addEdge"<<p<<"-- RT -->"<<newSym->identifier<<"\n";
		else
			qDebug()<<p<<"-- RT -->"<<newSym->identifier<<"el arco existe ya"<<"\n";
		innerToAGM((*i),id);
		
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
 	mutex->unlock();
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
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
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"agmInnerCompAgent");
	}
	catch(...)
	{
		exit(1);
	}
}




