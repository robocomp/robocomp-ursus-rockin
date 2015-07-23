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
void SpecificWorker::compute()
{
	///check remove_ImOriginal
//  	qDebug()<<"\n\nnumberOfSymbols agmInner.remove_ImOriginal: "<< (agmInner.remove_ImOriginal("/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/initialModelCOFFEE.xml",
// 				"/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml") )->numberOfSymbols();

	if (worldModel->numberOfSymbols()>0)
	{		
		
		qDebug()<<"\n\n\n\n************************";
		qDebug()<<"numberOfSymbols BEFORE insert InnerModel"<<worldModel->numberOfSymbols();
		qDebug()<<"************************";
		qDebug()<<"\n\n******* Original innerModel *****************";		
 		innerModel->treePrint();
		qDebug()<<"\n\n*********** include_im *************";
// 		
		///FIRST INNERMODEL
		QHash<QString, int32_t>  match;
		match.insert("room",7);
		match.insert("robot",1);				
		include_im(match,innerModel);
		agmInner.setWorld(worldModel);	
		worldModel->save("agmInnerFirst.xml");
		qDebug()<<"\n\n*********** FIRST INNERMODEL included ************* \n\n";
		
		qDebug()<<"\n\n******* Extract innerModel *****************";		
		QString nodeName="room";
		(agmInner.extractInnerModel(nodeName))->treePrint();
		qDebug()<<"\n\n*********** include_im *************";
// 		///SECOND INNERMODEL
// 		QHash<QString, int32_t>  match1;
// 		match1.insert("robot",1);		
// 		include_im(match1,innerModel1);
// 		qDebug()<<"\n\n*********** SECOND INNERMODEL included ************* \n\n";
// 		
// 		QString nodeName="room";
//  		qDebug()<<"\n\n****** extract innerModel from:"<<nodeName;		
// 		agmInner.setWorld(worldModel);	
// 		(agmInner.extractInnerModel(nodeName))->treePrint();		
// 		worldModel->save("agmInnerSecond.xml");
		///comprobacion agm puro es extraido correctamente
//  		qDebug()<<"\n\nnumberOfSymbols AFTER insert InnerModel"<<worldModel->numberOfSymbols();
// 		agmInner.setWorld(worldModel);
// 		worldModel=agmInner.extractAGM();
// 		qDebug()<<"numberOfSymbols extractAGM"<<worldModel->numberOfSymbols();
// 		qDebug()<<"***********  agmInner.extractAGM() print *************";
// 		AGMModelPrinter::printWorld(worldModel);
// 		qDebug()<<"************************";
		
// 		QString nodeName="world";
//  		qDebug()<<"\n\n****** extract innerModel from:"<<nodeName;
		//qDebug()<<"\n\n****** UPDATE INNERMODEL";
// 		agmInner.setWorld(worldModel);	
// 		(agmInner.extractInnerModel(nodeName))->treePrint();
// 		qDebug()<<"\n\nnumberOfSymbols AFTER setWorld"<< agmInner.getWorld()->numberOfSymbols();
// 		printf("sending modification!\n");
// 		AGMModel::SPtr newModel(new AGMModel(agmInner.getWorld()));		
// 		//AGMModelPrinter::printWorld(newModel);
		//sendModificationProposal(worldModel, newModel);
		
		
// 		(agmInner.extractInnerModel(nodeName))->treePrint();
		
// 		innerModel->updateTransformValues("floor",100.,200.,300.,.0,.0,.0);
// 		innerModel->treePrint("a",true);
// 		//agmInner.updateAgmWithInnerModel(worldModel,innerModel);
// 		agmInner.updateAgmWithInnerModel(innerModel);
// 		(agmInner.extractInnerModel(nodeName))->treePrint("d",true);
		qDebug()<<"\n\n**********************";
		
// 		
		printf("sending modification!\n");
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		qDebug()<<"-------------------------------------";
		sendModificationProposal(worldModel, newModel);
// 		int symbolID=20;
// 		string linkType ="RT";
// 		QList<int> visited;
// 		visited.append(symbolID);
//		bool loop=false;
// 		agmInner.checkLoop(symbolID,visited,"RT",loop);
		//qDebug()<<"CheckLoop from"<<symbolID<<"linkType"<<QString::fromStdString(linkType)<<"There is loop ?"<<loop<<visited;
		qFatal("fary");
	}
	
}	


/**
 * @brief Search the imName of the innermodel node,(the name is the unique key for innerModel ), inside de AGM Model. The innermodel id is stored in the attribute "imName" of each symbol. 
 * It is found, return the id of the symbol, the unique key for AGMSymbols, otherwise returns -1.
 * 
 * @param n value of the attribute field imName...
 * @return symbol ID, -1 if it is not found
 */
int SpecificWorker::findName(QString n)
{
	for (uint32_t i=0; i<worldModel->symbols.size(); ++i)
	{	
		if (worldModel->symbols[i]->attributes.find("imName") != worldModel->symbols[i]->attributes.end() )
		{
			if (worldModel->symbols[i]->attributes["imName"] == n.toStdString() )
			{
// 				qDebug()<<"findName: FOUND"<<n<<worldModel->symbols[i]->identifier;
				return worldModel->symbols[i]->identifier;
			}
		}
	}	
// 	qDebug()<<"findName: NO ENCONTRADO"<<n<<-1;
	return -1;
}

/**
 * @brief Insert innermodel in AGM graph matching nodes from innerModel to their correspondent symbols. 
 * Given the InnerModelNode ID as key in the hash, it inserts under the AGM symbol (specified by its ID as value in the hash) the counterpart subgraph.
 * If there is any relationship, parents-->children in innermodel. It is created a edge between the corresponding AGMSymbols, if it the edge exist, it is added a new RT link.
 * 
 * @param matchNode hash innerModelNode symbolID. 
 * @return void
 */
// void SpecificWorker::include_im(QHash<QString, int32_t>  match)
// {
// 	qDebug()<<match;
// 	QHash<QString, int32_t>::const_iterator i = match.constBegin();
// 	
// 	QList<QString> lNode =match.keys();
// 	
// 	//comprobar que todos los nodes existen en innerModel
// 	for (int i = 0; i < lNode.size(); ++i) 
// 	{
// 		InnerModelNode *node=innerModel->getNode(lNode.at(i));
// 		if (node==NULL)
// 		{
// 			qDebug()<<"node"<<lNode.at(i)<<"doesn't exist";
// 			qFatal("abort, not node");
// 		}
// 	}
// 	//comprobar que todos los symbolos existen en AGM y que tienen attribute name
// 	QList<int32_t> lSymbols =match.values();
// 	for (int i = 0; i < lSymbols.size(); ++i) 
// 	{
// 		try
// 		{
// 			AGMModelSymbol::SPtr symbol =worldModel->getSymbolByIdentifier(lSymbols.at(i));
// 			if (symbol->symbolType!="transform")
// 			{
// 				std::cout<<symbol->toString()<<"\n";
// 				qDebug()<<"AGMSymbol must be the attribute name, ADDING...";
// 				symbol->setAttribute("name",match.key(lSymbols.at(i)).toStdString());
// 			}
// 		}
// 		catch (AGMModelException e )	
// 		{
// 			std::cout<<e.what();
// 			qDebug()<<"symbol ID"<<lSymbols.at(i)<<"doesn't exist";
// 			qFatal("Abort, not symbol");
// 		}
// 	}
// // 	qDebug()<<"***** lNode"<< lNode;
// // 	qDebug()<<"***** lSymbols"<<lSymbols;
// 	
// 	//SI existe una relacion en innermodel entre dos nodos inserto un enlace RT en sus correspondientes symbolos
// 	for (int i = 0; i < lNode.size(); ++i) 
// 	{		
// 		InnerModelNode *node=innerModel->getNode(lNode.at(i));		
// 		for (int j = 0; j < lNode.size(); ++j) 
// 		{
// 			InnerModelNode *nodeSong=innerModel->getNode(lNode.at(j));
// 			qDebug()<<"InnerModel link"<<node->id <<"--RT-->"<<nodeSong->id;	
// 			if ( node->children.contains(nodeSong) )
// 			{
// 				qDebug()<<"crear en AGM, link"<<lSymbols.at(i)<<"--RT-->"<<lSymbols.at(j);	
// 				//edge 
// 				std::map<std::string, std::string> linkAttrs;
// 				linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str(0.0)) );
// 				linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str(0.0)) );
// 				linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str(0.0)) );
// 				linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str(0.0)) );
// 				linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str(0.0)) );
// 				linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str(0.0)) );
// 				worldModel->addEdgeByIdentifiers(lSymbols.at(i),lSymbols.at(j),"RT",linkAttrs);
// 			}
// 		}
// 	}
// 	
// 	//ya están chequeados lo que viene en match esta bien y en l
// 	while (i != match.constEnd()) 
// 	{
// // 		qDebug() << i.key() << ":" << i.value() << endl;
// 		lNode.removeAll(i.key());		
// // 		qDebug()<<"innerToAGM ( "<<i.key()<<i.value()<<lNode<<" ) ";
// 		int32_t sId=i.value();
// 		innerToAGM(innerModel->getNode(i.key()),sId,lNode);
// 		++i;
// 	}
// 	
// 
// }

///overload
/**
 * @brief Insert innermodel in AGM graph matching nodes from innerModel to their correspondent symbols. 
 * Given the InnerModelNode ID as key in the hash, it inserts under the AGM symbol (specified by its ID as value in the hash) the counterpart subgraph.
 * If there is any relationship, parents-->children in innermodel. It is created a edge between the corresponding AGMSymbols, if it the edge exist, it is added a new RT link.
 * 
 * @param matchNode hash innerModelNode symbolID. 
 * @return void
 */
void SpecificWorker::include_im(QHash<QString, int32_t>  match, InnerModel *im)
{
	qDebug()<<match;
	QHash<QString, int32_t>::const_iterator i = match.constBegin();
	
	QList<QString> lNode =match.keys();
	
	//comprobar que todos los nodes existen en im
	for (int i = 0; i < lNode.size(); ++i) 
	{
		InnerModelNode *node=im->getNode(lNode.at(i));
		if (node==NULL)
		{
			qDebug()<<"node"<<lNode.at(i)<<"doesn't exist";
			qFatal("abort, not node");
		}
	}
	//comprobar que todos los symbolos existen en AGM y que tienen attribute imName, imType y todo los de innermodel
	QList<int32_t> lSymbols =match.values();
	for (int i = 0; i < lSymbols.size(); ++i) 
	{
		try
		{
			AGMModelSymbol::SPtr symbol =worldModel->getSymbolByIdentifier(lSymbols.at(i));
			
			try
			{
				symbol->getAttribute("imName");
			}
			catch (...)			
			{
				std::cout<<"AGMSymbol "<< symbol->toString()<<" ";
				std::cout<<"must be the attribute imName, ADDING [imName , "<<match.key(lSymbols.at(i)).toStdString()<<" ]\n";
				symbol->setAttribute("imName",match.key(lSymbols.at(i)).toStdString());
			}			
			try
			{
				symbol->getAttribute("imType");
			}
			catch (...)			
			{
				std::cout<<"AGMSymbol "<< symbol->toString()<<" ";
				std::cout<<"must be the attribue imType, ADDING [imType , "<<match.key(lSymbols.at(i)).toStdString()<<" ]\n";
				symbol->setAttribute("imType",match.key(lSymbols.at(i)).toStdString());
			}
		}
		catch (AGMModelException e )	
		{
			std::cout<<e.what();
			qDebug()<<"symbol ID"<<lSymbols.at(i)<<"doesn't exist";
			qFatal("Abort, not symbol");
		}
	}
	
	//comprobar que todos los symbolos de los enlaces RT de agm tienen attribute name
	for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
	{
		//std::cout << ' ' << (*it)->toString(worldModel);
		if ((*it)->getLabel()=="RT" )
		{			
			AGMModelSymbol::SPtr symbolA = worldModel->getSymbol((*it)->getSymbolPair().first);			
			try
			{
				symbolA->getAttribute("imName");
			}
			catch (...)			
			{
				std::cout<<"AGMSymbol "<< symbolA->toString()<<" ";
				std::cout<<"must be the attribute imName, ADDING [imName , "<<symbolA->toString()<<" ]\n";
				symbolA->setAttribute("imName",symbolA->toString());
				symbolA->setAttribute("imType","transform");
			}	
			AGMModelSymbol::SPtr symbolB = worldModel->getSymbol((*it)->getSymbolPair().second);
			try
			{
				symbolB->getAttribute("imName");
			}
			catch (...)			
			{
				std::cout<<"AGMSymbol "<< symbolB->toString()<<" ";
				std::cout<<"must be the attribute name, ADDING [imName , "<<symbolB->toString()<<" ]\n";
				symbolB->setAttribute("imName",symbolB->toString());
				symbolA->setAttribute("imType","transform");
			}	
		}
		//std::cout << '\n';
	}
	
// 	qDebug()<<"***** lNode"<< lNode;
// 	qDebug()<<"***** lSymbols"<<lSymbols;
	
	//SI existe una relacion en innermodel entre dos nodos inserto un enlace RT en sus correspondientes symbolos
	for (int i = 0; i < lNode.size(); ++i) 
	{		
		InnerModelNode *node=im->getNode(lNode.at(i));		
		for (int j = 0; j < lNode.size(); ++j) 
		{
			InnerModelNode *nodeSong=im->getNode(lNode.at(j));
			qDebug()<<"InnerModel link"<<node->id <<"--RT-->"<<nodeSong->id;	
			if ( node->children.contains(nodeSong) )
			{
				//si no existe ya esta relacion
				//edge 
				try
				{
					AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(lSymbols.at(i), lSymbols.at(j), "RT");
										
				}
				catch (...)
				{
					qDebug()<<"crear en AGM, link"<<lSymbols.at(i)<<"--RT-->"<<lSymbols.at(j);	
					std::map<std::string, std::string> linkAttrs;
					linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str(0.0)) );
					linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str(0.0)) );
					linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str(0.0)) );
					linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str(0.0)) );
					linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str(0.0)) );
					linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str(0.0)) );
					worldModel->addEdgeByIdentifiers(lSymbols.at(i),lSymbols.at(j),"RT",linkAttrs);
				}
			}
		}
	}
	
	//ya están chequeados lo que viene en match esta bien y en l
	while (i != match.constEnd()) 
	{
// 		qDebug() << i.key() << ":" << i.value() << endl;
		lNode.removeAll(i.key());		
// 		qDebug()<<"innerToAGM ( "<<i.key()<<i.value()<<lNode<<" ) ";
		int32_t sId=i.value();
		innerToAGM(im->getNode(i.key()),sId,lNode);
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
				qDebug()<<node->id<<"link"<<(*i)->id;
				//symbol
				AGMModelSymbol::SPtr newSym = ImNodeToSymbol((*i));				
								
				//edge 
				std::map<std::string, std::string> linkAttrs;

				linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str((*i)->getTr().x())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str((*i)->getTr().y())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str((*i)->getTr().z())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str((*i)->getRxValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str((*i)->getRyValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str((*i)->getRzValue())));
				int32_t id = newSym->identifier;
				worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs);
				innerToAGM((*i),id,lNode);
			}
			else
				innerToAGM((*i),existingID,lNode);
		}
	}	
}

AGMModelSymbol::SPtr SpecificWorker::ImNodeToSymbol(InnerModelNode* node)
{
	
				
	
	std::map<std::string, std::string> attrs;
	attrs.insert ( std::pair<std::string,std::string>("imName",node->id.toStdString()) );
	
	AGMModelSymbol::SPtr newSym;
	
	//TODO innerModelNode cast to the type and translate the name.
	// attribute "type" = mesh,plane,joint..
	string type;
	if (dynamic_cast<InnerModelJoint*>(node) != NULL)
	{
		InnerModelJoint *joint = dynamic_cast<InnerModelJoint*>(node);
		//QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, ;
		//float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, 
// 		attrs.insert ( std::pair<std::string,std::string>("lx",float2str(0.) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("ly",float2str(0.) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("lz",float2str(0.) ) );
// 		
// 		attrs.insert ( std::pair<std::string,std::string>("hx",float2str(joint->backhX) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("hy",float2str(joint->backhY) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("hz",float2str(joint->backhZ) ) );
		
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(joint->min) ) );
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(joint->max) ) );
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(joint->port) ) );
		attrs.insert ( std::pair<std::string,std::string>("axis",joint->axis) );
		attrs.insert ( std::pair<std::string,std::string>("home",float2str(joint->home)) );
		type= "joint";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPrismaticJoint*>(node) != NULL)
	{
		InnerModelPrismaticJoint *p = dynamic_cast<InnerModelPrismaticJoint*>(node);
// 	float value, offset;
// 	float min, max;
// 	float home;
// 	uint32_t port;
// 	std::string axis;
	
		attrs.insert ( std::pair<std::string,std::string>("value",float2str(p->value) ) );
		attrs.insert ( std::pair<std::string,std::string>("offset",float2str(p->offset) ) );
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(p->min) ) );
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(p->max) ) );
		attrs.insert ( std::pair<std::string,std::string>("home",float2str(p->home)) );
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(p->port)) );
		attrs.insert ( std::pair<std::string,std::string>("axis",p->axis) );
		
		type= "prismaticJoint";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	
	else if (dynamic_cast<InnerModelTouchSensor*>(node) != NULL)
	{
		InnerModelTouchSensor *touch = dynamic_cast<InnerModelTouchSensor*>(node);
// 		float nx, ny, nz;
		attrs.insert ( std::pair<std::string,std::string>("nx",float2str(touch->nx) ));
		attrs.insert ( std::pair<std::string,std::string>("ny",float2str(touch->ny)) );
		attrs.insert ( std::pair<std::string,std::string>("nz",float2str(touch->nz)) );
		
		//float min, max;// 	float value;
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(touch->min) ));
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(touch->max)) );
		attrs.insert ( std::pair<std::string,std::string>("value",float2str(touch->value)) );
	// 	uint32_t port;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(touch->port)) );
		// QString stype;
		attrs.insert ( std::pair<std::string,std::string>("stype",touch->stype.toStdString()) );
		type = "touchSensor";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelDifferentialRobot*>(node) != NULL)
	{
		InnerModelDifferentialRobot *diff = dynamic_cast<InnerModelDifferentialRobot*>(node);
		//uint32_t port;
		//float noise;
		//bool collide;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(diff->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(diff->noise)) );
		
		string v="false";
		if (diff->collide)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collide",v) );
		
		type = "differentialRobot";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelOmniRobot*>(node) != NULL)
	{
		InnerModelOmniRobot *omni = dynamic_cast<InnerModelOmniRobot*>(node);
		//uint32_t port;
		//float noise;
		//bool collide;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(omni->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(omni->noise)) );
		
		string v="false";
		if (omni->collide)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collide",v) );
				
		type = "omniRobot";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPlane*>(node) != NULL)
	{
		//QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, 
		//float nx, float ny, float nz, float px, float py, float pz, bool collidable)

		InnerModelPlane* plane = dynamic_cast<InnerModelPlane*>(node);	
		
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(plane->width) ));
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(plane->height)) );
		attrs.insert ( std::pair<std::string,std::string>("depth",float2str(plane->depth)) );
		
		attrs.insert ( std::pair<std::string,std::string>("nx",float2str(plane->normal.x()) ));
		attrs.insert ( std::pair<std::string,std::string>("ny",float2str(plane->normal.y()) ));
		attrs.insert ( std::pair<std::string,std::string>("nz",float2str(plane->normal.z()) ));
		
		attrs.insert ( std::pair<std::string,std::string>("px",float2str(plane->point.x()) ));
		attrs.insert ( std::pair<std::string,std::string>("py",float2str(plane->point.y()) ));
		attrs.insert ( std::pair<std::string,std::string>("pz",float2str(plane->point.z()) ));
		
		string v="false";
		if (plane->collidable)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collidable",v) );
		attrs.insert ( std::pair<std::string,std::string>("repeat",int2str(plane->repeat)) );
		attrs.insert ( std::pair<std::string,std::string>("texture",plane->texture.toStdString()) );
		type = "plane";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
		
	}
	else if (dynamic_cast<InnerModelRGBD*>(node) != NULL)
	{
		InnerModelRGBD* rgbd = dynamic_cast<InnerModelRGBD*>(node);		
		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(rgbd->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(rgbd->noise) ) );
		attrs.insert ( std::pair<std::string,std::string>("focal",float2str(rgbd->focal) ) );
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(rgbd->height) ) );
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(rgbd->width) ) );
		attrs.insert ( std::pair<std::string,std::string>("ifconfig",rgbd->ifconfig.toStdString()) );
		type ="rgbd";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelCamera*>(node) != NULL)
	{
		InnerModelCamera* cam = dynamic_cast<InnerModelCamera*>(node);		
		
		attrs.insert ( std::pair<std::string,std::string>("focal",float2str(cam->focal) ) );
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(cam->height) ) );
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(cam->width) ) );
		
		type = "camera";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelIMU*>(node) != NULL)
	{
		InnerModelIMU* imu = dynamic_cast<InnerModelIMU*>(node);		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(imu->port)) );
		type = "imu";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelLaser*>(node) != NULL)
	{
		InnerModelLaser* laser = dynamic_cast<InnerModelLaser*>(node);
		//public:	uint32_t port;	uint32_t min, max;	float angle;	uint32_t measures;	QString ifconfig;
		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(laser->port)) );
		attrs.insert ( std::pair<std::string,std::string>("min",int2str(laser->min)) );
		attrs.insert ( std::pair<std::string,std::string>("max",int2str(laser->max)) );
		attrs.insert ( std::pair<std::string,std::string>("measures",int2str(laser->measures)) );
		attrs.insert ( std::pair<std::string,std::string>("angle",float2str(laser->angle)) );
		attrs.insert ( std::pair<std::string,std::string>("ifconfig",laser->ifconfig.toStdString()) );
		type = "laser";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelMesh*>(node) != NULL)
	{
		InnerModelMesh* mesh = dynamic_cast<InnerModelMesh*>(node);
		//InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable)

	     //return newMesh(id,parent,path,scale,scale,scale,render,tx,ty,tz,rx,ry,rz, collidable);
		attrs.insert ( std::pair<std::string,std::string>("path",mesh->meshPath.toStdString()) );
		attrs.insert ( std::pair<std::string,std::string>("scalex",float2str(mesh->scalex)) );
		attrs.insert ( std::pair<std::string,std::string>("scaley",float2str(mesh->scaley)) );
		attrs.insert ( std::pair<std::string,std::string>("scalez",float2str(mesh->scalez)) );
		string v="false";
		if (mesh->collidable)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collidable",v) );
		
		//enum RenderingModes { NormalRendering=0, WireframeRendering=1};
		v="NormalRendering";
		if (mesh->render==1)
			v="WireframeRendering";
		attrs.insert ( std::pair<std::string,std::string>("render",v) );
		
		type = "mesh";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPointCloud*>(node) != NULL)
	{
		//InnerModelPointCloud* pc = dynamic_cast<InnerModelPointCloud*>(node);		
		//attrs.insert ( std::pair<std::string,std::string>("generic",node->id.toStdString()) );
		type = "pointCloud";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelTransform*>(node) != NULL)
	{
		InnerModelTransform *t = dynamic_cast<InnerModelTransform*>(node);
		///InnerModelTransform *newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, 
		//float mass=0);
		type="transform";
		attrs.insert ( std::pair<std::string,std::string>("engine",t->engine.toStdString()) );
		attrs.insert ( std::pair<std::string,std::string>("mass",float2str(t->mass)) );
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	
	else
	{	
		string err;			
		err = "error: Type of node " + node->id.toStdString() + " is unknown.";
		throw err;
	}
	
	///new id for the symbol
	int32_t id =worldModel->getNewId();	
	return worldModel->newSymbol(id,type,attrs);

}



// COMMON METHODS


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	//agmInner = new AgmInner();
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
	
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("AgmInnerAgent.InnerModel1") ;
// 		qDebug()<<"hello";
// 		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel1 = new InnerModel(par.value);
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
// 		qDebug()<<"sendModificationProposal, printWorld";
// 		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"agmInnerCompAgent");
	}
	catch(...)
	{
		exit(1);
	}
}





