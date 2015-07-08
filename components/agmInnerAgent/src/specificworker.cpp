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
		qDebug()<<"\n\n\n\n************************";
		qDebug()<<"numberOfSymbols"<<worldModel->numberOfSymbols();
		qDebug()<<"************************";
		qDebug()<<"\n\n*********** include_im *************";
// 		include_im("world",20);
		QHash<QString, int32_t>  match1;
		match1.insert("world",20);
		match1.insert("initialRobotPose",25);
		include_im(match1);
// 		qDebug()<<"\n\n******* treePrint first *****************";		
//  		innerModel->treePrint();
//  		qDebug()<<"\n\n ********** AGMModelPrinter::printWorld(worldModel) **************";
//  		AGMModelPrinter::printWorld(worldModel);
// 		
//  		qDebug()<<"\n\n****** extract innerModel and print ****************";
// 		//(extractInnerModel("world"))->treePrint();
// // 		qDebug()<<"\n\n TODO ";
// 		qDebug()<<"\n\n**********************";
		
// 		printf("sending modification!\n");
// 		AGMModel::SPtr newModel(new AGMModel(worldModel));		
// 		AGMModelPrinter::printWorld(newModel);
// 		sendModificationProposal(worldModel, newModel);
		qFatal("fary");
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
	int n=symbol->identifier;
	innerToAGM(node,n);
}

void SpecificWorker::innerToAGM(InnerModelNode* node, int &symbolID)
{
	QList<InnerModelNode*>::iterator i;	
	int p=symbolID;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		//Search name (key) of the innerModel node in AGM
		int existingID = findName ( (*i)->id ) ;
		if ( existingID == -1 )
		{	
			int32_t id =worldModel->getNewId();
			
			qDebug()<<node->id<<"link"<<(*i)->id;
			std::map<std::string, std::string> attrs;
			attrs.insert ( std::pair<std::string,std::string>("name",(*i)->id.toStdString()) );
			
			AGMModelSymbol::SPtr newSym;
			
			//TODO innerModelNode cast to the type and translate the name.
			// attribute "type" = mesh,plane,joint..
			// newSym = nodeToSymbol(InnerModelNode * node);
			newSym = worldModel->newSymbol(id,"transform",attrs);		
							
			//edge 
			std::map<std::string, std::string> linkAttrs;

			linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str((*i)->getTr().x())) );
			linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str((*i)->getTr().y())) );
			linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str((*i)->getTr().z())) );
			linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str((*i)->getRxValue())));
			linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str((*i)->getRyValue())));
			linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str((*i)->getRzValue())));
			
			worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs);
// 			if (worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs))
// 				qDebug()<<"addEdge"<<p<<"-- RT -->"<<newSym->identifier<<"\n";
// 			else
// 				qDebug()<<p<<"-- RT -->"<<newSym->identifier<<"el arco existe ya"<<"\n";
			innerToAGM((*i),id);
		}
		else
			innerToAGM((*i),existingID);
	}	
}

/**
 * @brief Search the name of the innermodel node,(the name is the unique key for innerModel ), inside de AGM Model. The innermodel id is stored in the attribute "name" of each symbol. 
 * It is found, return the id of the symbol, the unique key for AGMSymbols, -1 in other case.
 * 
 * @param n value of the attribute field name...
 * @return symbol ID, -1 if it is not found
 */
int SpecificWorker::findName(QString n)
{
	for (uint32_t i=0; i<worldModel->symbols.size(); ++i)
	{	
		if (worldModel->symbols[i]->attributes.find("name") != worldModel->symbols[i]->attributes.end() )
		{
			if (worldModel->symbols[i]->attributes["name"] == n.toStdString() )
			{
// 				qDebug()<<"findName: FOUND"<<n<<worldModel->symbols[i]->identifier;
				return worldModel->symbols[i]->identifier;
			}
		}
	}	
// 	qDebug()<<"findName: NO ENCONTRADO"<<n<<-1;
	return -1;
}

InnerModel* SpecificWorker::extractInnerModel(QString imNodeName)
{
	
	//innerModel imNew = AGM.extractInnerModel();
	//0 Go through the graph 
	//1 Find transform node transform_
	//2 Read his attribute "name" to make the ide
	InnerModel *imNew = new InnerModel() ;

	int symbolID = findName(imNodeName);
// 	symbolID=5;
	if (symbolID > -1)
		recorrer(imNew, symbolID);
	
	return imNew;
}

void SpecificWorker::recorrer(InnerModel* imNew, int& symbolID)
{
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(symbolID);
	
// 	std::cout<<symbol->toString(true)<<"\n";
// 	int first;
// 	int second;
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
// 		std::cout<<"\t"<<(*edge_itr)->getLabel()<<"\n";
		string secondSymbolType = worldModel->getSymbol( (*edge_itr)->getSymbolPair().second)->symbolType;
		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==symbolID && secondSymbolType=="transform")
		{
// 			std::cout<<"\t\t"<<(*edge_itr)->toString(worldModel)<<"\n";
			int first = (*edge_itr)->getSymbolPair().first;
			int second = (*edge_itr)->getSymbolPair().second;
// 			qDebug()<<"\t\t\tfirst<<second<<symbolID"<<first<<second<<symbolID;
			//qDebug()<<"insertar en innermodel, P->H: "<<QString::fromStdString(symbol->attributes["name"])<<"-->"<<QString::fromStdString( worldModel->getSymbol(second)->attributes["name"] );
			//symbolToImNode();
			symbolToImNode(symbol,(*edge_itr),worldModel->getSymbol(second),imNew);
			recorrer(imNew,second);
// 			qDebug()<<"---";
		}
	}

}
void SpecificWorker::symbolToImNode(AGMModelSymbol::SPtr symbol, AGMModelEdge edge, AGMModelSymbol::SPtr symbolSong, InnerModel* im)
{
	InnerModelNode* nodeA = NULL;
	InnerModelNode* nodeB = NULL;
	if (edge->getLabel() != "RT")
		qFatal("MAAAAAL symbolToImNode() ");
	
	QString nameA = QString::fromStdString(symbol->attributes["name"]);
	QString nameB = QString::fromStdString(symbolSong->attributes["name"]);
	
	//TODO hacer que por defecto si en el .xml de agm no se especifican valores de RT ponerlos a 0.
	float tx,ty,tz,rx,ry,rz;
	tx=ty=tz=rx=ry=rz=0.;
// 	tx = str2float(edge->attributes["tx"]);
// 	ty = str2float(edge->attributes["ty"]);
// 	tz = str2float(edge->attributes["tz"]);
// 
// 	rx = str2float(edge->attributes["rx"]);
// 	ry = str2float(edge->attributes["ry"]);
// 	rz = str2float(edge->attributes["rz"]);
	
	qDebug()<<"insertar"<<nameA<<"--"<< QString::fromStdString ( edge->getLabel() ) <<"-->"<<nameB;//<<tx<<ty<<tz<<rx<<ry<<rz;
	
	if (symbol->symbolType!="transform")		
		qFatal("MAAAAAL symbolToImNode() no transform ");
	//node father
	nodeA=im->getNode(nameA);

	if (nodeA==NULL)
	{
		nodeA = im->newTransform(nameA, "static",im->getRoot(),tx,ty,tz,rx,ry,rz);
		im->getRoot()->addChild(nodeA);
	}
	
	nodeB = im->newTransform (nameB, "static",nodeA,tx,ty,tz,rx,ry,rz);
	nodeA->addChild(nodeB);
	if (nodeB==NULL)
		qFatal("MAAAAAL symbolToImNode() nodeB == null ");
}


/**
 * @brief Insert innermodel in AGM graph matching nodes from innerModel to their correspondent symbols. 
 * Given the InnerModelNode ID as key in the hash, it inserts under the agm symbol specified by its ID in the value field the corresponding subtree.
 * 
 * @param matchNode hash innerModelNode symbolID. InnerModel nodes have to be inserted in order. First lower levels.
 * @return void
 */
void SpecificWorker::include_im(QHash<QString, int32_t>  match)
{
	qDebug()<<match;
	QHash<QString, int32_t>::const_iterator i = match.constBegin();
	QList<QString> lNode =match.keys();
	while (i != match.constEnd()) 
	{
		qDebug() << i.key() << ":" << i.value() << endl;
					
		InnerModelNode *node=innerModel->getNode(i.key());
		if (node==NULL)
		{
			qDebug()<<"node"<<i.key()<<"doesn't exist";
			qFatal("abort");
		}
		AGMModelSymbol::SPtr symbol;
		
		try
		{
			symbol =worldModel->getSymbolByIdentifier(i.value());
			if (symbol->symbolType!="transform")
			{
				std::cout<<symbol->toString()<<"\n";
				qFatal("AGMSymbol must be transform");
			}
		}
		catch (AGMModelException e )	
		{
			std::cout<<e.what();
			qFatal("abort");
		}
		int n=symbol->identifier;
		lNode.removeAll(i.key());		
		qDebug()<<"innerToAGM ( "<<node->id<<n<<lNode<<" ) ";
		innerToAGM(node,n,lNode);
		++i;
	}
	

}

void SpecificWorker::innerToAGM(InnerModelNode* node, int &symbolID, QList<QString>  lNode)
{
	QList<InnerModelNode*>::iterator i;	
	int p=symbolID;
	
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		if ( !lNode.contains((*i)->id) )
		{	
			//Search name (key) of the innerModel node in AGM
			int existingID = findName ( (*i)->id ) ;
			if ( existingID == -1 )
			{	
				int32_t id =worldModel->getNewId();
				
				qDebug()<<node->id<<"link"<<(*i)->id;
				std::map<std::string, std::string> attrs;
				attrs.insert ( std::pair<std::string,std::string>("name",(*i)->id.toStdString()) );
				
				AGMModelSymbol::SPtr newSym;
				
				//TODO innerModelNode cast to the type and translate the name.
				// attribute "type" = mesh,plane,joint..
				// newSym = nodeToSymbol(InnerModelNode * node);
				newSym = worldModel->newSymbol(id,"transform",attrs);		
								
				//edge 
				std::map<std::string, std::string> linkAttrs;

				linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str((*i)->getTr().x())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str((*i)->getTr().y())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str((*i)->getTr().z())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str((*i)->getRxValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str((*i)->getRyValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str((*i)->getRzValue())));
				
				worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs);
				innerToAGM((*i),id);
			}
			else
				innerToAGM((*i),existingID);
		}
	}	
}



// ****** AGENTS *******
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





