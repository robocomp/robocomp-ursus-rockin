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
#include <stdio.h>
#include <iostream>
#include <fstream>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModelVacio = new InnerModel();
	imHumanGeneric = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/person.xml");
	newBodyEvent=false;
	number=0;
	setPeriod(100);

	innerModelMap.clear();
	initDictionary();
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
	this->personList = people;
	this->timeStamp = timestamp;
	///esto es pq cuando hay 0 personas no me envia nada
	timerTimeStamp.setSingleShot(true);
	timerTimeStamp.start(50);
	newBodyEvent = true;	
}

//Dado  un innerModel creo otro, añadiendo el prefijo a cada id del nodo
void SpecificWorker::newInnerModel(InnerModel * imSrc, InnerModel *imDst, QString pre)
{
	//recorrer innermodel generico añadiendo el sufijo del nuevo symbolo e insertandolo en el map
	
	QList<InnerModelNode *>	l;
	
	imSrc->getSubTree(imSrc->getNode("root"),&l);
	
	QList<InnerModelNode*>::iterator it;
	for (it=l.begin();it!=l.end();it++)
	{
		insertNodeInnerModel(imDst,(*it),pre);
	}
	//copia perfecta con su prefijo
	//imDst->save("imDst.xml");

}
void SpecificWorker::insertNodeInnerModel(InnerModel* im, InnerModelNode* node, QString pre)
{
// 	qDebug()<<node->id;
// 	qDebug()<<"\t parent"<<node->parent->id;
	
	
	InnerModelNode * parent = NULL;
	if (node->parent->id=="root")
	{			
		parent = im->getRoot();							
	}
	else
	{
		parent = im->getNode(pre+node->parent->id);
	}
	if (parent==NULL)
		qFatal("parent null stop");

	if(  dynamic_cast<InnerModelTransform *>( node )  != NULL )
	{
		
		InnerModelTransform * tf=dynamic_cast<InnerModelTransform *>( node );
				
		if(  dynamic_cast<InnerModelJoint *>( tf )  != NULL )
		{
			qDebug()<<"insert Joint";
			InnerModelJoint * joint=dynamic_cast<InnerModelJoint *>( tf );
			
			InnerModelJoint * newJoint = im->newJoint (pre+joint->id,dynamic_cast<InnerModelTransform *>( parent),								   
								joint->backlX,joint->backlY,joint->backlZ,joint->backhX,joint->backhY,joint->backhZ,
								joint->getTr().x(),joint->getTr().y(),joint->getTr().z(),joint->getRxValue(),joint->getRyValue(),joint->getRzValue(),
								joint->min, joint->max,joint->port,joint->axis,joint->home);
			parent->addChild(newJoint);		
		}
		else
		{
			qDebug()<<"insert transform";
			InnerModelTransform * newTf = im->newTransform(pre+tf->id,tf->engine,parent,tf->getTr().x(),tf->getTr().y(),tf->getTr().z(),
								       tf->getRxValue(),tf->getRyValue(),tf->getRzValue(),tf->mass);
			parent->addChild(newTf);	
		}
	}
	else if(  dynamic_cast<InnerModelMesh *>( node )  != NULL )
	{
		qDebug()<<"insert Mesh";
		InnerModelMesh * m=dynamic_cast<InnerModelMesh *>( node );
		
		InnerModelMesh * newMesh = im->newMesh(pre+m->id,parent, m->meshPath,m->scalex,m->scaley,m->scalez,m->render,m->tx,m->ty,m->tz,m->rx,m->ry,m->rz,m->collidable);
		parent->addChild(newMesh);		
	}
	else if(  dynamic_cast<InnerModelPlane *>( node )  != NULL )
	{
		qDebug()<<"insert Plane";
		InnerModelPlane * p=dynamic_cast<InnerModelPlane *>( node );
		
		InnerModelPlane * newPlane = im->newPlane(pre+p->id,parent,p->texture,
							p->width,p->height,p->depth,p->repeat,
							p->normal(0),p->normal(1),p->normal(2),
							p->point(0),p->point(1),p->point(2),
							p->collidable);
		parent->addChild(newPlane);		
	}
	else
	{
		qDebug()<<"type not implemented, node-id: "<<node->id<<"\n";
	}
	
}

void SpecificWorker::compute()
{
	QMutexLocker m (mutex);		
	qDebug()<<"newBodyEvent"<<newBodyEvent;	
	if (newBodyEvent)
	{
		//Insertar simbolos para todo el torso		
		updatePeopleInnerFull();						
		newBodyEvent=false;	
	}
	
	//clear personList after a while without to recive any event
	if (timerTimeStamp.isActive() == false and newBodyEvent==false)		
	{		
		std::cout<<"\t clear list \n";		
		personList.clear();
		updatePeopleInnerFull();
		if (!innerModelMap.empty())
		{
			saveInnerModels("666666");
			qDebug()<<"innerModelMap.size()"<<innerModelMap.size();			
			qFatal("fary innerModelMap not empty");
		}
	}
}


//la idea es actualizar para N personas sus innerModel (completos, sin piernas) y trasladarlos al grafo
//Y COLGARLOS DEL MUNDO
void SpecificWorker::updatePeopleInnerFull()
{
	
	
	int32_t robotID = worldModel->getIdentifierByType("robot");
	if (robotID < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return;
	}
	bool modification = false;
	
	
	///CAUTION CHAPUZA PA PROBAR A COLGAR DLE MUNDO 
	 robotID = agmInner.findName("room");
	if (robotID < 0)
	{
		printf("ROOOM symbol not found, \n");
		qFatal("abort");
		return;
	}
	
	
	//extrae en una lista con los ID de los symbolos "person" que son hijos del symbolo robotID enlazados mediante "RT"
	//Qlist<int32_t> l = listaSymbolos(int symbolID, string symbolType=person,string linkType=RT);
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
				//std::cout<<" es una persona "<<symbolSecond->toString()<<"\n";
				l.append(second);
			}
		}
	}
// 	qDebug()<<"lsymbols person:"<<l;
// 	qDebug()<<"\n ********** \n";
			
	//calculo para cada strucutra personIT de TPerson mskBody.ice, su correspondientes RT en robocomp
	//<int,TPerson> personIt; jajajaja
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
		//lo suyo sería usar el innerModel update...
		if (found)
		{
			//actualizos su estado	
			std::cout<<"\n\tSimbolo encontrado: Actualizando el symbolo person: "<<personID<<"\n";
			AGMModelSymbol::SPtr  s =worldModel->getSymbol(personID);				
			if ( s->getAttribute("State")!=int2str(personIt.second.state) )
			{
				s->setAttribute("State",int2str(personIt.second.state));
				//AGMMisc::publishNodeUpdate(s,agmagenttopic_proxy);
			}
			
			//ACTUALIZO EL INNERMODEL DEL HUMANO en mi vector de InnerModel locales
			updateInnerModel(personIt.second,personID);
			static int frame = 1000;
			
			//TODO chequear esta funcion
			//agmInner.updateAgmWithInnerModel(innerModelMap.at(personID));
			saveInnerModels(QString::number(frame));
// 			agmInner.updateAgmWithInnerModelAndPublish(innerModelMap.at(personID),agmagenttopic_proxy);
			if (frame >=1002)
				qFatal("fary updateAgmWithInnerModel");
			frame++;
		
		}
		// añado la nueva en cualquier estado ??		
		else 
		{
			AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");			
			std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<"\n";
			newSymbolPerson->setAttribute("TrackingId",int2str(personIt.second.TrackingId));
			newSymbolPerson->setAttribute("State",int2str(personIt.second.state));
			
			//creo desde un innerModelGenerico un specifico para esa persona
			int id = newSymbolPerson->identifier;		
			QString pre =QString::fromStdString(int2str(id));
			innerModelMap[id] =new InnerModel();
			newInnerModel(imHumanGeneric, innerModelMap.at(id),pre);
			updateInnerModel(personIt.second,id);
					
			//lo inserto en la super estructura agmInner
			QHash<QString, int32_t>  match;			
			match.insert(pre+"XN_SKEL_TORSO",id);
			qDebug()<<"\nbefore innerModelMap.at(personID)"<<innerModelMap.at(personID)->getIDKeys()<<"\n";
			agmInner.include_im(match,innerModelMap.at(id));
			qDebug()<<"\nafterinnerModelMap.at(personID)"<<innerModelMap.at(personID)->getIDKeys()<<"\n";
// // 			//añado su arco calculado para innerModel, de la matzi kinect a la persona
			std::map<string,string>att;
			att["tx"]=float2str( innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getTr().x());				
			att["ty"]=float2str( innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getTr().y());
			att["tz"]=float2str( innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getTr().z());
 			att["rx"]=float2str(innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getRxValue());
 			att["ry"]=float2str(innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getRyValue());
 			att["rz"]=float2str(innerModelMap.at(id)->getTransform(pre+"XN_SKEL_TORSO")->getRzValue());
			worldModel->addEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT",att);
// 			
// 			///printing
// 			AGMModelEdge &edge = worldModel->getEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT");
// 			std::cout<<"\tRT [ "<<edge->getAttribute("tx")<<" , "<<edge->getAttribute("ty")<<" , "<<edge->getAttribute("tz");
// 			std::cout<<" , "<<edge->getAttribute("rx")<<" , "<<edge->getAttribute("ry")<<" , "<<edge->getAttribute("rz")<<" ]\n";

			modification = true;		
		}
	}
	//removeSymbol persons si queda alguno en la lista de symbolos
	for (int i=0; i< l.size(); i++)
	{
		std::cout<<" remove Symbol "<<worldModel->getSymbol(l.at(i))->toString()<<"\n";	
		worldModel->save("AGM_BeforeRemovePerson.xml");
		innerModelMap.at(l.at(i))->save("innerPersonToRemove.xml");
		qDebug()<<"\n innerModelMap.at(l.at(i))->getIDKeys()\n"<<innerModelMap.at(l.at(i))->getIDKeys()<<"\n";
		agmInner.remove_Im(innerModelMap.at(l.at(i)));		
		delete innerModelMap.at(l.at(i)); 
		innerModelMap.erase(l.at(i));
		
		qDebug()<<"innerModelMap.size()"<<innerModelMap.size();
		worldModel->save("AGM_AfterRemovePerson.xml");
		worldModel->removeSymbol(l.at(i));
		worldModel->save("AGM_AfterRemoveSymbol_Person.xml");
		
		qFatal("fary remove");
		modification=true;		
	}

	if (modification)
	{
		qDebug()<<"-------------------------------------";
		AGMModel::SPtr newModel(new AGMModel(worldModel));	
		//agmInner.setWorld(worldModel);
		sendModificationProposal(worldModel, newModel);					
		saveInnerModels(QString::number(number));
		number++;
	}

}



//PARA LAS PRUEBAS PRIMERAS NOS VALE
//la idea es actualizar para N personas sus innerModel y trasladarlos al grafo
//actualiza de momento el symbolo persona con los valores del torso obtenido del winKinect y transformado a InnerModel para N personas
// void SpecificWorker::updatePeopleInner()
// {
// 	
// 	
// 	int32_t robotID = worldModel->getIdentifierByType("robot");
// 	if (robotID < 0)
// 	{
// 		printf("Robot symbol not found, Waiting for the executive...\n");
// 		return;
// 	}
// 	bool modification = false;
// 	
// 	//extrae en una lista con los ID de los symbolos "person" que son hijos del symbolo robotID enlazados mediante "RT"
// 	//Qlist<int32_t> l = listaSymbolos(int symbolID, string symbolType=person,string linkType=RT);
// 	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(robotID);
// 	QList<int32_t> l;
// 	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
// 	{
// 		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
// 		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
// 		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==robotID )
// 		{
// 			int second = (*edge_itr)->getSymbolPair().second;
// 			const AGMModelSymbol::SPtr &symbolSecond=  worldModel->getSymbolByIdentifier(second);
// 			if(symbolSecond->symbolType=="person")
// 			{
// 				std::cout<<" es una persona "<<symbolSecond->toString()<<"\n";
// 				l.append(second);
// 			}
// 		}
// 	}
// 	qDebug()<<"lsymbols person:"<<l;
// 	qDebug()<<"\n ********** \n";
// 			
// 	//calculo para cada strucutra personIT de TPerson mskBody.ice, su correspondientes RT en robocomp
// 	//<int,TPerson> personIt; jajajaja
// 	for( auto personIt : personList )
// 	{
// 		bool found = false;
// 		int personID = -1;
// 		calculateJointRotations(personIt.second);
// 		//lista de ID de symbolos
// 		for (int i=0; i< l.size(); i++)
// 		{
// 			//buscar persona
// 			if ( str2int ( (worldModel->getSymbol(l.at(i))->getAttribute("TrackingId")) ) ==personIt.second.TrackingId ) 				
// 			{			
// 				personID=l.at(i);
// 				found = true;
// 				l.removeOne(personID);
// 				std::cout<<"id symbol person with TrackingId: "<<personID<<"\n";
// 				break;				
// 			}			
// 		}
// 		
// 		//si encuentro el id en el worldModel la actualizo con el valor de la lista, el estado no me dice nada
// 		if (found)
// 		{
// 			//actualizos su estado	
// 			std::cout<<"Actualizao el symbolo "<<personID<<"\n";
// 			AGMModelSymbol::SPtr  s =worldModel->getSymbol(personID);				
// 			s->setAttribute("State",int2str(personIt.second.state));
// 			AGMMisc::publishNodeUpdate(s,agmagenttopic_proxy);
// 			
// 			
// // 			if (personIt.second.state== RoboCompMSKBody::stateType::Tracking )
// 			{
// 				//actualizo su arco de momento solo con el valor SPINE ya en innerModel ya en RoboComp!!
// 				std::cout<<"Actualizo su arco\n";
// 				AGMModelEdge &edge = worldModel->getEdgeByIdentifiers(robotID,personID,"RT");
// 				std::cout<<"\tedge "<<edge.toString(worldModel)<<"\n";
// 				
// 				edge->setAttribute("tx",float2str(1000*mapJointRotations[ "Spine" ].getTr().x() ));
// 				edge->setAttribute("ty",float2str(1000*mapJointRotations[ "Spine" ].getTr().y()));
// 				edge->setAttribute("tz",float2str(1000*mapJointRotations[ "Spine" ].getTr().z()));
// 				edge->setAttribute("rx",float2str(mapJointRotations[ "Spine" ].getRxValue()));
// 				edge->setAttribute("ry",float2str(mapJointRotations[ "Spine" ].getRyValue()));
// 				edge->setAttribute("rz",float2str(mapJointRotations[ "Spine" ].getRzValue()));
// 				
// 				AGMMisc::publishEdgeUpdate(edge,agmagenttopic_proxy);
// 				
// 				///printing
// 				std::cout<<"\tRT [ "<<edge->getAttribute("tx")<<" , "<<edge->getAttribute("ty")<<" , "<<edge->getAttribute("tz");
// 				std::cout<<" , "<<edge->getAttribute("rx")<<" , "<<edge->getAttribute("ry")<<" , "<<edge->getAttribute("rz")<<" ]\n";
// 			}			
// 		}
// 		// añado la nueva en cualquier estado ??		
// 		else 
// 		{
// 			AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");			
// 			std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<"\n";
// 			newSymbolPerson->setAttribute("TrackingId",int2str(personIt.second.TrackingId));
// 			newSymbolPerson->setAttribute("imName",newSymbolPerson->toString());
// 			newSymbolPerson->setAttribute("imType","transform");
// 				
// 			//añado su arco calculado para innerModel
// 			std::map<string,string>att;
// 			att["tx"]=float2str(1000*mapJointRotations[ "Spine" ].getTr().x());				
// 			att["ty"]=float2str(1000*mapJointRotations[ "Spine" ].getTr().y());
// 			att["tz"]=float2str(1000*mapJointRotations[ "Spine" ].getTr().z());
// 			att["rx"]=float2str(mapJointRotations[ "Spine" ].getRxValue());
// 			att["ry"]=float2str(mapJointRotations[ "Spine" ].getRyValue());
// 			att["rz"]=float2str(mapJointRotations[ "Spine" ].getRzValue());;
// 			worldModel->addEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT",att);
// 			
// 			///printing
// 			AGMModelEdge &edge = worldModel->getEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT");
// 			std::cout<<"\tRT [ "<<edge->getAttribute("tx")<<" , "<<edge->getAttribute("ty")<<" , "<<edge->getAttribute("tz");
// 			std::cout<<" , "<<edge->getAttribute("rx")<<" , "<<edge->getAttribute("ry")<<" , "<<edge->getAttribute("rz")<<" ]\n";
// 
// 			modification = true;		
// 		}
// 	}
// 	//removeSymbol persons si queda alguno en la lista de symbolos
// 	for (int i=0; i< l.size(); i++)
// 	{
// 		std::cout<<" remove Symbol "<<worldModel->getSymbol(l.at(i))->toString()<<"\n";
// 		worldModel->removeSymbol(l.at(i));
// 		modification=true;
// 	}
// //	enum stateType{NoTracking, PositionOnly, Tracking};
// 
// 	if (modification)
// 	{
// 		qDebug()<<"-------------------------------------";
// 		AGMModel::SPtr newModel(new AGMModel(worldModel));			
// 		sendModificationProposal(worldModel, newModel);
// 		
// 	}
// 
// }

//PROBABLEMENTE SEA BORRADO PORQUE NO TIENE SENTIDO LO DE IR HASTA X Y Z COMO "CENTRO DE MASAS"
//DE LA PERSONA
// void SpecificWorker::updatePeople()
// {
// 	
// 	
// 	int32_t robotID = worldModel->getIdentifierByType("robot");
// 	if (robotID < 0)
// 	{
// 		printf("Robot symbol not found, Waiting for the executive...\n");
// 		return;
// 	}
// 	bool modification = false;
// // 	std::cout<<"updatePeople " << personList.size()<<" timestamp "<<timeStamp<<"\n";
// 	//si no hay nadie mantemos lo que habia... (ya veremos)
// // 	if (personList.size()<1)		
// // 	{
// // 		return;
// // 	}
// 	
// 	
// 	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(robotID);
// 	QList<int32_t> l;
// 	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
// 	{
// 		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
// 		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
// 		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==robotID )
// 		{
// 			int second = (*edge_itr)->getSymbolPair().second;
// 			const AGMModelSymbol::SPtr &symbolSecond=  worldModel->getSymbolByIdentifier(second);
// 			if(symbolSecond->symbolType=="person")
// 			{
// 				std::cout<<" es una persona "<<symbolSecond->toString()<<"\n";
// 				l.append(second);
// 			}
// 			
// 		}
// 	}
// 	qDebug()<<"lsymbols person:"<<l;
// 	qDebug()<<"\n ********** \n";
// 			
// 	//añado un simbolo persona relativo a Position (algo asi como el centro de masas)
// 	for( auto personIt : personList )
// 	{
// 		bool found = false;
// 		int personID = -1;
// 		personIt.second.Position.X=personIt.second.Position.X*1000;
// 		personIt.second.Position.Y=personIt.second.Position.Y*1000;
// 		personIt.second.Position.Z=personIt.second.Position.Z*1000;
// 		//lista de ID de symbolos
// 		for (int i=0; i< l.size(); i++)
// 		{
// 			//buscar persona
// 			if ( str2int ( (worldModel->getSymbol(l.at(i))->getAttribute("TrackingId")) ) ==personIt.second.TrackingId ) 				
// 			{			
// 				personID=l.at(i);
// 				found = true;
// 				l.removeOne(personID);
// 				std::cout<<"id symbol person with TrackingId: "<<personID<<"\n";
// 				break;				
// 			}			
// 		}
// 		
// 		//si encuentro el id en el worldModel la actualizo con el valor de la lista, el estado no me dice nada
// 		if (found)
// 		{
// 			//actualizos su estado	
// 			std::cout<<"Actualizao el symbolo "<<personID<<"\n";
// 			AGMModelSymbol::SPtr  s =worldModel->getSymbol(personID);				
// 			s->setAttribute("State",int2str(personIt.second.state));
// 			AGMMisc::publishNodeUpdate(s,agmagenttopic_proxy);
// 			
// 			
// // 			if (personIt.second.state== RoboCompMSKBody::stateType::Tracking )
// 			{
// 				//actualizo su arco
// 				std::cout<<"Actualizo su arco\n";
// 				AGMModelEdge &edge = worldModel->getEdgeByIdentifiers(robotID,personID,"RT");
// 				std::cout<<"\tedge "<<edge.toString(worldModel)<<"\n";
// 				
// 				edge->setAttribute("tx",float2str(personIt.second.Position.X));
// 				edge->setAttribute("ty",float2str(personIt.second.Position.Y));
// 				edge->setAttribute("tz",float2str(personIt.second.Position.Z));
// 				
// 				AGMMisc::publishEdgeUpdate(edge,agmagenttopic_proxy);
// 			}			
// 		}
// 		// añado la nueva en cualquier estado ??
// 		else 
// 		{
// 			
// 			
// 			AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");			
// 			std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<"\n";
// 			newSymbolPerson->setAttribute("TrackingId",int2str(personIt.second.TrackingId));
// 			
// 			std::cout<<"i: "<<personIt.first<<"\n";
// 			std::cout<<"personIt.second.state "<<personIt.second.state<<"\n";
// 			std::cout<<"personIt.second.TrackingId "<<personIt.second.TrackingId<<"\n";
// 			std::cout<<"personIt.second.Position ( "<<personIt.second.Position.X<<" "<<personIt.second.Position.Y<<" "<<personIt.second.Position.Z<<" )\n";
// 			
// 			//añado su arco relativo a Position (algo asi como el centro de masas)
// 			std::map<string,string>att;
// 			att["tx"]=float2str(personIt.second.Position.X);
// 			att["ty"]=float2str(personIt.second.Position.Y);
// 			att["tz"]=float2str(personIt.second.Position.Z);
// 			att["rx"]=att["ry"]=att["rz"]="0";
// 			worldModel->addEdgeByIdentifiers(robotID,newSymbolPerson->identifier,"RT",att);
// 			
// 			
// 			modification = true;
// 			
// 		
// 		}
// 	}
// 	//removeSymbol persons
// 	for (int i=0; i< l.size(); i++)
// 	{
// 		std::cout<<" remove Symbol "<<worldModel->getSymbol(l.at(i))->toString()<<"\n";
// 		worldModel->removeSymbol(l.at(i));
// 		modification=true;
// 	}
// //	enum stateType{NoTracking, PositionOnly, Tracking};
// 
// 	if (modification)
// 	{
// 		qDebug()<<"-------------------------------------";
// 		AGMModel::SPtr newModel(new AGMModel(worldModel));			
// 		sendModificationProposal(worldModel, newModel);
// 	}
// 
// }



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
void SpecificWorker::initDictionary()
{
	/*Cargamos el mapa de  nombres*/
	dictionaryNames[ "Spine" ] = "XN_SKEL_TORSO";
	dictionaryNames[ "Head" ] = "XN_SKEL_NECK";
	dictionaryNames[ "ShoulderLeft" ] = "XN_SKEL_LEFT_SHOULDER";
	dictionaryNames[ "ShoulderRight" ] = "XN_SKEL_RIGHT_SHOULDER";
	dictionaryNames[ "ElbowLeft" ] = "XN_SKEL_LEFT_ELBOW";
	dictionaryNames[ "ElbowRight" ] = "XN_SKEL_RIGHT_ELBOW";
// 	dictionaryNames[ "HipCenter" ] = "XN_SKEL_WAIST";
// 	dictionaryNames[ "HipLeft" ] = "XN_SKEL_LEFT_HIP";
// 	dictionaryNames[ "HipRight" ] = "XN_SKEL_RIGHT_HIP";
// 	dictionaryNames[ "KneeLeft" ] = "XN_SKEL_LEFT_KNEE";
// 	dictionaryNames[ "KneeRight" ] = "XN_SKEL_RIGHT_KNEE";
	dictionaryNames[ "HandRight" ] = "XN_SKEL_RIGHT_HAND";
// 	dictionaryNames[ "FootLeft" ] = "XN_SKEL_LEFT_FOOT";
// 	dictionaryNames[ "FootRight" ] = "XN_SKEL_RIGHT_FOOT";
	dictionaryNames[ "HandLeft" ] = "XN_SKEL_LEFT_HAND";
// 	dictionaryNames[ "WallPose" ] = "WALL_POSE";
// 	dictionaryNames[ "Wall" ] = "XN_SUCCESS_WALL";
	
	
	dictionaryEnum[ "Spine" ] = Spine;
	dictionaryEnum[ "Head" ] = Head;
	dictionaryEnum[ "ShoulderLeft" ] = ShoulderLeft;
	dictionaryEnum[ "ShoulderRight" ] = ShoulderRight;
	dictionaryEnum[ "ElbowLeft" ] = ElbowLeft;
	dictionaryEnum[ "ElbowRight" ] = ElbowRight;
	dictionaryEnum[ "HipCenter" ] = HipCenter;
	dictionaryEnum[ "HipLeft" ] = HipLeft;
	dictionaryEnum[ "HipRight" ] = HipRight;
	dictionaryEnum[ "KneeLeft" ] = KneeLeft;
	dictionaryEnum[ "HandRight" ] = HandRight;
	dictionaryEnum[ "KneeRight" ] = KneeRight;
	dictionaryEnum[ "FootLeft" ] = FootLeft;
	dictionaryEnum[ "FootRight" ] = FootRight;
	dictionaryEnum[ "HandLeft" ] = HandLeft;
}


////************** INNERMODEL ******************++
 void SpecificWorker::updateInnerModel( TPerson &person, int idPerson )
{
	float x,y,z,rx,ry,rz;
	string idJoint;

	calculateJointRotations(person);
	
	for( auto dictionaryNamesIt : dictionaryNames )
	{
		try
		{
			idJoint = dictionaryNamesIt.first;
			x=y=z=rx=ry=rz=0.0;
			
		
			x = 1000*mapJointRotations[ idJoint ].getTr().x();
			y = 1000*mapJointRotations[ idJoint ].getTr().y();
			z = 1000*mapJointRotations[ idJoint ].getTr().z();		
			
			rx = mapJointRotations[ idJoint ].getRxValue();
			ry = mapJointRotations[ idJoint ].getRyValue();
			rz = mapJointRotations[ idJoint ].getRzValue();
				
			//qDebug()<<QString::fromStdString( idJoint)<<pose.x<<pose.y<<pose.z<<"( "<<pose.rx<<pose.ry<<pose.rz<<" )";
			//nan check			
			if ( (x!=x) or (y!=y) or (z!=z) or (rx!=rx) or (ry!=ry) or (rz!=rz) )
				continue;
			string idNode = int2str(idPerson) + dictionaryNamesIt.second.toStdString();			
			if (idJoint=="Spine")
				innerModelMap[ idPerson ]->updateTransformValues( QString::fromStdString(idNode),x,y,z,rx,ry,rz );
			else
				innerModelMap[ idPerson ]->updateRotationValues( QString::fromStdString(idNode),rx,ry,rz );						
		}
		catch ( Ice::Exception e ) 
		{
			qDebug( )<<"error updateInnerModel"<<e.what( );
		}
	}
}
 /**
 * @brief Given a person, this method calculates the Rotation Matrix of all the Joints. person is in metres
 * 
 * @param person Person who rotation matrix is calculated
 * @return void
 */
 void SpecificWorker::calculateJointRotations( TPerson &person )
{
	RTMat kinect;
	RoboCompMSKBody::JointList jointList;
	
	jointList = person.joints;
	///*********************** TODO transform ***************************	
	try
	{
		InnerModel *innerModelTmp = new InnerModel();		
		innerModelTmp = agmInner.extractInnerModel("room", true);
		kinect=innerModelTmp->getTransformationMatrix("rgbdHumanPose","room");
		kinect.setTr(kinect.getTr().x()/1000.0, kinect.getTr().y()/1000.0,kinect.getTr().z()/1000.0);			
	}
	catch (...)
	{
		qDebug()<<"not found kinect using identity RTMat kinect";
	}
	//kinect.print("kinect");
	
	
	/// apunta el torso (inclinación alante/atrás y lateral del torso)
	mapJointRotations[ "Spine" ]=
	rtMatFromJointPosition( kinect,
			      jointList[ dictionaryEnum[ "Spine" ] ].Position,
			      jointList[ dictionaryEnum[ "Head" ] ].Position, 
			      jointList[ dictionaryEnum[ "Spine" ] ].Position, 2 );			

	/// alineación de hombros (rotación en Z del torso), previa al cálculo de la transformacion final de los hombros. 
	RTMat LEFT_SHOULDER_PRE_Z=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position, 
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position, 2 );
	RTMat RIGHT_SHOULDER_PRE_Z=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position, 
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position, 2 );
	
	rotarTorso ( RIGHT_SHOULDER_PRE_Z.getTr( ), LEFT_SHOULDER_PRE_Z.getTr( ) );
	

	///Cintura
// 	mapJointRotations[ "HipCenter" ]= 
// 	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
// 			      jointList[ dictionaryEnum[ "HipCenter" ] ].Position,
// 			      jointList[ dictionaryEnum[ "Spine" ] ].Position,
// 			      jointList[ dictionaryEnum[ "HipCenter" ] ].Position, 2 );

	mapJointRotations[ "HipLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "HipLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HipLeft" ] ].Position, 2);
	
	mapJointRotations[ "HipRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "HipRight" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HipRight" ] ].Position, 2);
	
	///Knee			
	mapJointRotations[ "KneeLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipLeft" ],
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position,
      			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position, 2);
				
	mapJointRotations[ "KneeRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipRight" ],
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position,
      			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position, 2);


//     ///cabeza
// 	mapJointRotations[ "Head" ]=
// 	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
// 				jointList[ dictionaryEnum[ "Head" ] ].Position,
// 				joints[ "JOINT_HEAD" ],
// 				joints[ "JOINT_NECK" ], 2);


	///brazo izquierdo
	mapJointRotations[ "ShoulderLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position, 2);		
			    
	mapJointRotations[ "ElbowLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderLeft" ],
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position, 2);		


	///brazo derecho
	///codo al hombro (p2 inicio p1 final)
	mapJointRotations[ "ShoulderRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position, 2);		
			    
	mapJointRotations[ "ElbowRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderRight" ],
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position, 2);		
 
	///Manos derecha e izquierda
	mapJointRotations[ "HandRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderRight" ]*mapJointRotations[ "ElbowRight" ],
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position, 2);
	
	mapJointRotations[ "HandLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderLeft" ]*mapJointRotations[ "ElbowLeft" ],
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position, 2);

	///Manos derecha e izquierda
	mapJointRotations[ "FootRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipRight" ]*mapJointRotations[ "KneeRight" ],
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position, 2);
	
     	mapJointRotations[ "FootLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipLeft" ]*mapJointRotations[ "KneeLeft" ],
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position, 2);
}
 
 /**
  * @brief This method calculates the rotation of a Joint given some points.
  * 
  * @param rS Indicates the rotations of the above joints. (Example: To calculate the rotation of the ElbowLeft we must know the rotation of the ShoulderLeft).
  * @param p1 Starting point of the Joint (Example: ElbowLeft)
  * @param p2 Ending point of the Joint (Example: WristLeft)
  * @param translation Traslation of the current Joint (Example: ElbowLeft)
  * @param axis Always axis Z (2)
  * @return RMat::RTMat This is the Rotation Matrix
  */
 RTMat SpecificWorker::rtMatFromJointPosition( RTMat rS, RoboCompMSKBody::SkeletonPoint p1, RoboCompMSKBody::SkeletonPoint p2, RoboCompMSKBody::SkeletonPoint translation, int axis )
{
   	bool XClockWise=true, YClockWise=true, ZClockWise=true;
	float alpha, beta, gamma;
	
	RTMat rt(XClockWise,YClockWise, ZClockWise);
	QVec p1h = QVec::vec4(p1.X, p1.Y, p1.Z, 1);
	QVec p2h = QVec::vec4(p2.X, p2.Y, p2.Z,1);
	QVec translationH = QVec::vec4(translation.X, translation.Y, translation.Z,1);
	
	QMat aux =rS;
	aux = aux.invert();
	QVec translationT = aux *translationH;
	QVec p1t = aux * p1h;
	QVec p2t = aux * p2h;
	QVec v = p2t - p1t;
	
	v= v.normalize();
	
	  ///por filas
	switch(axis){
	case 0:
		alpha = 0;
		
		if(YClockWise) beta = atan2(-v.z(),v.x());
		else beta = atan2(v.z(),v.x());
		
		if(ZClockWise) gamma = asin(-v.y());
		else gamma = asin(v.y());
		
		break;
	case 1:
		if(XClockWise) alpha = atan2(v.z(),v.y());
		else alpha = atan2(-v.z(),v.y());
		
		beta = 0;
		
		if(ZClockWise) gamma = asin(v.x());
		else gamma = asin(-v.x());
		
		break;
	case 2:
		if(XClockWise) alpha =  atan2(-v.y(),v.z());
		else alpha =  atan2(v.y(),v.z());
		
		if(YClockWise) beta = asin(v.x());
		else beta = asin(-v.x());
		
		gamma = 0;
		
		break;
	}
	
	rt.setRT(alpha,beta,gamma,translationT);
	
	return rt;
}
 
 
 /**
  * @brief This method allows to rotate the torso from the position and rotation of the shoulders
  * 
  * @param hombroizq Rotation Matrix from LeftShoulder
  * @param hombroder Rotation Matrix from RightShoulder
  * @return true if all the operations went correctly
  */
 bool SpecificWorker::rotarTorso( const QVec & hombroizq, const QVec & hombroder )
{
	QVec eje= hombroizq - hombroder;	//Calculamos el eje que va de un hombro a otro
	
	eje.normalize ();

	if( eje.x( ) == 0 ) return false;

	float angulo = atan2( eje.y( ),eje.x( ) );	//Calculamos el giro necesario para alinear los hombros con el eje
	
	mapJointRotations[ "Spine" ].setRZ( angulo ); // Aplicamos dicho giro al eje Z del torso
	
	return true;
}

void SpecificWorker::saveInnerModels(QString number)
{	
	worldModel->save(number.toStdString()+"_agmWorldModelLocal.xml");
	agmInner.getWorld()->save(number.toStdString()+"_agmWorldModel_agmInner.xml");	
	for( auto m : innerModelMap )			
	{		
		QString pre =QString::fromStdString(int2str(m.first));
		qDebug()<<"Saving innermodels : "<<pre+"innerHuman.xml";
		m.second->save(number+"_"+pre+"innerHuman.xml");			
	}		
	agmInner.extractInnerModel("room", true)->save(number+"_extractInnerModelFromRoom.xml");
	
}

///****** code for check innermodel memory
// 	std::ofstream myfile;
// 	myfile.open ("innerTestMesh.xml");
// 	myfile <<  "<innerModel>\n\n";
// 	for (int i=0;i<500;i++)
// 	{
// 		myfile <<"<transform id=\""<<QString::number(i).toStdString()<<"transform\">";	
// 		myfile <<"\n\t<mesh id=\"XN_SKEL_TORSO_mesh"<<QString::number(i).toStdString()
// 			<<"\" file=\"/home/robocomp/robocomp/files/osgModels/humanColor/pecho.osg\" scale=\"10\" rx=\"1.57\" />\n";		
// 		myfile <<"</transform>\n";
// 	}
// 	myfile <<  "\n</innerModel>";
// 	myfile.close();
// 	qFatal("fary");
	
// 	qDebug()<<"worldModel->numberOfSymbols()"<<worldModel->numberOfSymbols()<<"newBodyEvent"<<newBodyEvent;	
// 	InnerModel *innerModelTmp = new InnerModel("/home/luiky/robocomp/components/robocomp-ursus-rockin/components/humanAgent/etc/innerTestMesh.xml");
// 	InnerModel *innerModelTmp = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/autonomyLab/autonomyLabModel.xml");
// 		usleep(10000);		
// 		qDebug()<<"11";
// 		delete innerModelTmp;
// 		qDebug()<<"22";
// 		usleep(20000);
// 		return;
// 	
// 	return;

