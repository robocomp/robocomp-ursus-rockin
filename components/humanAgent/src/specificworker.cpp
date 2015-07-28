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
	newBodyEvent=false;
	
	//fake
// 	for (int i=0; i<5;i++)
// 	{
// 		RoboCompMSKBody::TPerson t;
// 		if (i%2==0)
// 			t.state=RoboCompMSKBody::Tracking;	
// 		else
// 			t.state=RoboCompMSKBody::PositionOnly;	
// 		
// 		t.TrackingId=rand()% 32768;	
// 		t.Position.X=0.;
// 		t.Position.Y=(float)i+10.0;
// 		t.Position.Z=0.;
// 		std::pair<int,TPerson> p;
// 		p.first=i;
// 		p.second=t;
// 		personList.insert(p);
// 	}
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::newMSKBodyEvent(const PersonList &people, const long &timestamp)
{	
	QMutexLocker m (mutex);
	std::cout<<"\n\nnew newMSKBodyEvent, people.size() " << people.size()<<" timestamp "<<timestamp<<"\n\n";
	this->personList = people;
// 	if (timestamp!=timeStamp)
	{
		this->timeStamp = timestamp;
		timerTimeStamp.setSingleShot(true);
		timerTimeStamp.start(10000);
	}
	
	//newBodyEvent = true;	
	
	
}



void SpecificWorker::compute()
{
	QMutexLocker m (mutex);		
	qDebug()<<"timerTimeStamp.isActive() "<<timerTimeStamp.isActive();
	qDebug()<<"worldModel->numberOfSymbols()"<<worldModel->numberOfSymbols();
	std::cout<<"\tpersonList.size() "<<personList.size()<<" timeStamp "<<timeStamp<<"\n";
	
// 	srand(1000);
// 	if (worldModel->numberOfSymbols()>0)
// 	{
// 		newBodyEvent =true;
// 		static int stop =0;
// 		std::cout<<"\n\tstop: "<<stop<<"\n";
// 		if (stop==1)
// 			personList.erase(0);			
// 		if (stop==2)
// 			personList.at(2).Position.Y=100;
// 		if (stop>3)
// 			qFatal("fary");
// 		stop++;
// 	}
// 	if (newBodyEvent)
	{
		updatePeople();
		//newBodyEvent=false;		
	}
	
	//clear personList after a while without received event
	if (timerTimeStamp.isActive() ==false and personList.empty()==false  )		
	{
		std::cout<<"\t\t clear list \n\n";
		personList.clear();
	}
		
}

void SpecificWorker::updatePeople()
{
	
	
	int32_t robotID = worldModel->getIdentifierByType("robot");
	if (robotID < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return;
	}
	bool modification = false;
// 	std::cout<<"updatePeople " << personList.size()<<" timestamp "<<timeStamp<<"\n";
	//si no hay nadie mantemos lo que habia... (ya veremos)
// 	if (personList.size()<1)		
// 	{
// 		return;
// 	}
	
	
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(robotID);
	QList<int32_t> l;
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==robotID )
		{
			int second = (*edge_itr)->getSymbolPair().second;
			const AGMModelSymbol::SPtr &symbolSecond=  worldModel->getSymbolByIdentifier(second);
			if(symbolSecond->symbolType=="person")
			{
				std::cout<<" es una persona "<<symbolSecond->toString()<<"\n";
				l.append(second);
			}
			
		}
	}
	qDebug()<<"lsymbols person:"<<l;
	qDebug()<<"\n ********** \n";
			
	
	for( auto personIt : personList )
	{
		bool found = false;
		int personID = -1;
		//lista de ID de symbolos
		for (int i=0; i< l.size(); i++)
		{
			//buscar persona
			if ( str2int ( (worldModel->getSymbol(l.at(i))->getAttribute("TrackingId")) ) ==personIt.second.TrackingId ) 				
			{			
				personID=l.at(i);
				found = true;
				l.removeOne(personID);
				std::cout<<"id symbol person with TrackingId: "<<personID<<"\n";
				break;				
			}			
		}
		
		//si encuentro el id en el worldModel la actualizo con el valor de la lista, el estado no me dice nada
		if (found)
		{
			//actualizos su estado	
			std::cout<<"Actualizao el symbolo "<<personID<<"\n";
			AGMModelSymbol::SPtr  s =worldModel->getSymbol(personID);				
			s->setAttribute("State",int2str(personIt.second.state));
			AGMMisc::publishNodeUpdate(s,agmagenttopic_proxy);
			
			
// 			if (personIt.second.state== RoboCompMSKBody::stateType::Tracking )
			{
				//actualizo su arco
				std::cout<<"Actualizo su arco\n";
				AGMModelEdge &edge = worldModel->getEdgeByIdentifiers(robotID,personID,"RT");
				edge->setAttribute("tx",float2str(personIt.second.Position.X*1000));
				edge->setAttribute("ty",float2str(personIt.second.Position.Y*1000));
				edge->setAttribute("tz",float2str(personIt.second.Position.Z*1000));
				AGMMisc::publishEdgeUpdate(edge,agmagenttopic_proxy);
				
				
			}			
		}
		// añado la nueva en cualquier estado ??
		else 
		{
			
			
			AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");			
			std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<"\n";
			newSymbolPerson->setAttribute("TrackingId",int2str(personIt.second.TrackingId));
			
			std::cout<<"i: "<<personIt.first<<"\n";
			std::cout<<"personIt.second.state "<<personIt.second.state<<"\n";
			std::cout<<"personIt.second.TrackingId "<<personIt.second.TrackingId<<"\n";
			std::cout<<"personIt.second.Position ( "<<personIt.second.Position.X<<" "<<personIt.second.Position.Y<<" "<<personIt.second.Position.Z<<" )\n";
			
			//añado su arco
			std::map<string,string>att;
			att["tx"]=float2str(personIt.second.Position.X*1000);
			att["ty"]=float2str(personIt.second.Position.Y*1000);
			att["tz"]=float2str(personIt.second.Position.Z*1000);
			att["rx"]=att["ry"]=att["rz"]="0";
			worldModel->addEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT",att);
			
			
			modification = true;
			
		
		}
	}
	//removeSymbol persons
	for (int i=0; i< l.size(); i++)
	{
		std::cout<<" remove Symbol "<<worldModel->getSymbol(l.at(i))->toString()<<"\n";
		worldModel->removeSymbol(l.at(i));
		modification=true;
	}
//	enum stateType{NoTracking, PositionOnly, Tracking};

	if (modification)
	{
		qDebug()<<"-------------------------------------";
		AGMModel::SPtr newModel(new AGMModel(worldModel));			
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
 
	agmInner.setWorld(worldModel);
	
	mutex->unlock();
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	
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
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"humanCompAgent");
	}
	catch(...)
	{
		exit(1);
	}
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


