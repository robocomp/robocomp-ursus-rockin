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
//#########################################################################
//#########################################################################
//#########################################################################
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel(); 
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
//#########################################################################
//#########################################################################
//#########################################################################
void SpecificWorker::compute( )
{
	printf("--\n");
	static std::string previousAction = "";
	bool newAction = false;
	if (previousAction != action)
		newAction = true;

	previousAction = action;
	printf("New action: %s\n", action.c_str());

	if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}
}
//#########################################################################
//#########################################################################
//#########################################################################
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
//#########################################################################
//#########################################################################
//#########################################################################
void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event& modification)
{
	
	printf("pre <<structuralChange\n");
	QMutexLocker l(mutex);
	printf("<<structuralChange\n");

	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);
	innerModel->treePrint();

	printf("structuralChange>>\n");
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	mutex->lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);
	mutex->unlock();
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	mutex->lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);
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
// 		AGMModelPrinter::printWorld(newModel);		
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel, "objectAgent");
	}
	catch(...)
	{
		exit(1);
	}
}


// Get new apriltags!
void SpecificWorker::newAprilTag(const tagsList &list)
{
	if (worldModel->numberOfSymbols() == 0) return;

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	bool publishModel = false;
	for (auto ap : list)
	{
		switch(ap.id)
		{
			case 30:
				if (updateTable(ap, newModel))
				{
					publishModel = true;
					printf("TABLE E %d  (%f, %f, %f)    (%f, %f, %f)\n", ap.id, ap.tx, ap.ty, ap.tz, ap.rx, ap.ry, ap.rz);
// 					if (updateTable(ap,SPtr)) printf("error in newAprilTag, updateTable");
					
				}
				break;
			case 31:
				if (updateMug(ap, newModel))
				{
					printf("MUG %d  (%f, %f, %f)    (%f, %f, %f)\n", ap.id, ap.tx, ap.ty, ap.tz, ap.rx, ap.ry, ap.rz);
					publishModel = true;
				}
				break;				
		}
	}

	if (publishModel)
	{
		sendModificationProposal(worldModel, newModel);
	}
}

/**
 * \brief Metodo UPDATE TABLE: 
 * 1) Tomamos ID de la tabla en AGM y en INNER
 * 2) Calulamospadre.
 * 3) Calculamos enlace anterior E
 * 4) UPDATE AGM
 */ 
bool SpecificWorker::updateTable(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	bool existing = false;

	AGMModelSymbol::SPtr symbol;
	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		symbol = *symbol_it;
		if (symbol->symbolType == "object" and symbol->attributes.find("tag") != symbol->attributes.end())
		{
			try
			{
				const int32_t tag = str2int(symbol->getAttribute("tag")); //Identificador numerico del tag
				if (t.id == tag)
				{
					// Si el identificador del nodo coincide con el del tag entonces el simbolo existe.
					// Salimos del bucle porque ya lo hemos encontrado.
					existing = true;
					break;
				}
			}
			catch (...) { printf("%s: %d\n", __FILE__, __LINE__); }
		}
	}
	
	if (existing)
	{
		printf("TABLE Exists\n");
		QVec poseTag      = QVec::vec6(t.tx, t.ty, t.tz, t.rx, t.ry, t.rz); //sistema RGBD
		qDebug()<<"POSE TABLA VISTA POR ROBOT: "<<poseTag;
		QString symbolIMName    = QString::fromStdString(symbol->getAttribute("imName"));
		qDebug() << "Table??: "<<symbolIMName<<"--->"<< symbol->identifier;
		
		InnerModelNode *nodeSymbol   = innerModel->getNode(symbolIMName);
		if (nodeSymbol)
		{
			printf("HAY NODO %s: %d\n", __FILE__, __LINE__);
			InnerModelNode *parentNode = nodeSymbol->parent;
			if (parentNode)
			{
				printf("%s: %d\n", __FILE__, __LINE__);
				QString parentIMName = parentNode->id;
				qDebug() << "Nombre del padre: "<< parentIMName;
				
// WARNING TODO ERROR GAUUUUUUUUUUUUUUUUUUUUUUUU				poseFromParent = GIRO EN X
				
				QVec poseFromParent = innerModel->transform6D(parentIMName, poseTag, "rgbd");
				poseFromParent.print("poseFromParent");
				
				//BUSCAR EL ENLACE RT
				bool parentFound = false;
				AGMModelSymbol::SPtr symbolParent;
				for(AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
				{
					symbolParent = *symbol_it;
					if (symbolParent->symbolType == "object" and symbolParent->attributes["imName"]==parentIMName.toStdString())
					{
						parentFound = true;
						break;
					}
				}
				
				if (parentFound)
				{
					try
					{
						AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers( symbolParent->identifier, symbol->identifier, "RT");
						edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
						edgeRT->setAttribute("ty", float2str(poseFromParent.y()));
						edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
						edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
						edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
						edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));
						
						qDebug()<<"UPDATE NOW!";
	//					updateAgmWithInnerModelAndPublish(innerModel, AGMAgentTopicPrx &agmagenttopic_proxy);
						AGMMisc::publishEdgeUpdate(edgeRT, agmagenttopic_proxy);

					}
					catch(...){ qDebug()<<"Impossible to update the RT edge"; }
				}
				
				
			}
			else
			{
				qDebug() << "no hay nodo padre"; 
			}
		}
		else
		{
			qFatal("Couldn't find the table in innermodel"); 
		}
	}
	else 
	{
		qFatal("TODO: Create table symbol");
	}

	return (not existing);
}

/**
 * \brief UPDATE MUG 
 * 1) Tomamos ID de la taza en AGM y en INNER
 * 2) Calulamos padre.
 * 3) Calculamos enlace anterior E
 * 4) UPDATE AGM
 */ 
bool SpecificWorker::updateMug(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{
	bool existing = false;
	AGMModelSymbol::SPtr symbol;
	
	for (AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
	{
		symbol = *symbol_it;
		if (symbol->symbolType == "object" and symbol->attributes.find("tag") != symbol->attributes.end())
		{
			try
			{
				const int32_t tag = str2int(symbol->getAttribute("tag"));
				if (t.id == tag)
				{
					// Si el identificador del nodo coincide con el del tag entonces el simbolo existe.
					// Salimos del bucle porque ya lo hemos encontrado.
					existing = true;
					break;
				}
			}
			catch (...){ printf("||   ERROR %s: %d\n   ||", __FILE__, __LINE__);}
		}
	}

	if (existing)
	{
		QVec positionTag           = QVec::vec3(t.tx, t.ty, t.tz); //traslaciones de la aprilTag en RGBD
		QString symbolIMName       = QString::fromStdString(symbol->getAttribute("imName"));
		InnerModelNode *nodeSymbol = innerModel->getNode(symbolIMName); //sacamos nodo de innermodel
		
		qDebug() << "||   MUG EXISTS: "<<symbolIMName<<"----> "<<symbol->identifier<<"   ||";
		
		if (nodeSymbol)
		{
			InnerModelNode *parentNode = nodeSymbol->parent; // sacamos el padre del nodo en innermodel
			if (parentNode)
			{
				qDebug() <<"||   NODE PARENT: "<< parentNode->id<<"   ||";
				QString parentIMName    = parentNode->id;
				//TRANSFORMACIONES DE LA APRILTAG DESDE RGBD A PARENT NODE:
				// 1) La traslacion se realiza con un simple transform
				// 2) La rotacion es mas complicada: hay que añadir el offset de las apriltags (giradas PI/2 en x)
				QVec positionFromParent  = innerModel->transform(parentIMName, positionTag, "rgbd");
				QMat rotationOffset      = Rot3D(M_PI_2, 0, 0); //offset de las apriltags NO SE ESTA USANDO?????
				QMat rotationTag         = Rot3D(t.rx, t.ry, t.rz); //rotacion propia de la marca
				QMat rotationRGBD2Parent = innerModel->getRotationMatrixTo("rgbd", parentIMName); //matriz rotacion del nodo padre a la rgbd
				QVec rotation;
				rotation = (rotationTag * rotationRGBD2Parent).extractAnglesR_min();
				rotation.print("sin");
				rotation = (rotationTag * rotationRGBD2Parent).invert().extractAnglesR_min(); 
				rotation.print("con"); //la buena se saca de la matriz inversa. ORDEN: rgbd2Parent * tag
				// COMPONEMOS LA POSE ENTERA:
				QVec poseFromParent = QVec::zeros(6);
				poseFromParent.inject(positionFromParent, 0);
				poseFromParent.inject(rotation, 3);
				poseFromParent.print("||   POSE FROM PARENT  ||");
				innerModel->transform6D("room", poseFromParent, parentIMName).print("||   POSE FROM ROOM   ||");
				
				//BUSCAR EL ENLACE RT: NECESITAMOS EL SIMBOLO PADRE EN AGM
				bool parentFound = false;
				AGMModelSymbol::SPtr symbolParent;
				for(AGMModel::iterator symbol_it=newModel->begin(); symbol_it!=newModel->end(); symbol_it++)
				{
					symbolParent = *symbol_it;
					if (symbolParent->symbolType == "object" and symbolParent->attributes["imName"]==parentIMName.toStdString())
					{
						qDebug()<<"||   FOUND PARENT: "<< QString::fromStdString(symbolParent->attributes["imName"])<<"   ||";
						parentFound = true;
						break;
					}
				}
				if (parentFound)
				{
					// Si el padre existe en AGM sacamos el enlace RT que va desde el padre hasta el hijo y actualizamos sus valores
					try
					{
						AGMModelEdge &edgeRT  = newModel->getEdgeByIdentifiers(symbolParent->identifier, symbol->identifier, "RT");
						edgeRT->setAttribute("tx", float2str(poseFromParent.x()));
						edgeRT->setAttribute("ty", float2str(poseFromParent.y()));
						edgeRT->setAttribute("tz", float2str(poseFromParent.z()));
						edgeRT->setAttribute("rx", float2str(poseFromParent.rx()));
						edgeRT->setAttribute("ry", float2str(poseFromParent.ry()));
						edgeRT->setAttribute("rz", float2str(poseFromParent.rz()));
						
						qDebug()<<"||   UPDATE NOW!   ||";
	//					updateAgmWithInnerModelAndPublish(innerModel, AGMAgentTopicPrx &agmagenttopic_proxy);
						AGMMisc::publishEdgeUpdate(edgeRT, agmagenttopic_proxy);
					}
					catch(...){ qFatal("Impossible to update the RT edge"); }
				}
				else
					qDebug()<<"||   PARENT NODE DOESN'T EXIST INTO AGM   ||";
			}
			else
			{
				qDebug() << "||   PARENT NODE DOESN'T EXIST INTO INNERMODEL   ||"; 
			}
		}
		else
		{
			qDebug() << "||   SYMBOL NODE DOESN'T EXIST INTO INNERMODEL   ||"; 
		}
	}
	else
	{
		qFatal("||   SYMBOL NODE DOESN'T EXIST INTO AGM   ||");
		//Si el simbolo no existe, lo creamos... TODO
// 		try
// 		{
// 			int32_t objectSymbolID;
// 			int32_t objectStSymbolID;
// 			getIDsFor("mug", objectSymbolID, objectStSymbolID);
// 
// 			auto symbols = newModel->getSymbolsMap(params, "robot", "container");
// 			AGMModelSymbol::SPtr newMug = newModel->newSymbol("object", objectSymbolID);
// 			AGMModelSymbol::SPtr newMugStatus = newModel->newSymbol("objectSt", objectStSymbolID);
// 			newModel->addEdge(symbols["robot"], newMug, "know");
// 			newModel->addEdge(newMug, newMugStatus, "hasStatus");
// 			newModel->addEdge(newMug, newMugStatus, "see");
// 			newModel->addEdge(newMug, newMugStatus, "position");
// 			newModel->addEdge(newMug, newMugStatus, "reachable");
// 			newModel->addEdge(newMug, newMugStatus, "reach");
// 			newModel->addEdge(newMug, newMugStatus, "classified");
// 			newModel->addEdge(newMug, newMugStatus, "mug");
// 			newModel->addEdge(newMug, symbols["container"], "in");
// 
// 			const std::string tagIdStr = int2str(t.id);
// 			newMug->attributes["tag"] = tagIdStr;
// 			newMug->attributes["tx"] = "1300";
// 			newMug->attributes["ty"] = "800";
// 			newMug->attributes["tz"] = "-1350";
// 			newMug->attributes["rx"] = "0";
// 			newMug->attributes["ry"] = "-3.1415926535";
// 			newMug->attributes["rz"] = "0";
// 		}
// 		catch(...)
// 		{
// 			printf("(updateMug) objectAgent: Couldn't retrieve action's parameters\n");
// 		}
	}

	return not existing;
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

	if (not existing)
	{
		try
		{
			int32_t objectSymbolID;
			int32_t objectStSymbolID;
			getIDsFor("milk", objectSymbolID, objectStSymbolID);

			auto symbols = newModel->getSymbolsMap(params, "robot", "container");
			AGMModelSymbol::SPtr newMilk = newModel->newSymbol("object", objectSymbolID);
			AGMModelSymbol::SPtr newMilkStatus = newModel->newSymbol("objectSt", objectStSymbolID);
			newModel->addEdge(symbols["robot"], newMilk, "know");
			newModel->addEdge(newMilk, newMilkStatus, "hasStatus");
			newModel->addEdge(newMilk, newMilkStatus, "see");
			newModel->addEdge(newMilk, newMilkStatus, "position");
			newModel->addEdge(newMilk, newMilkStatus, "reachable");
			newModel->addEdge(newMilk, newMilkStatus, "reach");
			newModel->addEdge(newMilk, newMilkStatus, "classifailed");
			newModel->addEdge(newMilk, symbols["container"], "in");

			const std::string tagIdStr = int2str(t.id);
			newMilk->attributes["tag"] = tagIdStr;
			newMilk->attributes["tx"] = "1100";
			newMilk->attributes["ty"] = "820";
			newMilk->attributes["tz"] = "-1350";
			newMilk->attributes["rx"] = "0";
			newMilk->attributes["ry"] = "-3.1415926535";
			newMilk->attributes["rz"] = "0";
		}
		catch(...)
		{
			printf("(updateMilk) objectAgent: Couldn't retrieve action's parameters\n");
		}
	}

	const bool forcePublishModel = not existing;
// 	printf("force publish by milk %d (%d)\n", forcePublishModel, t.id);
	return forcePublishModel;
}

bool SpecificWorker::updateCoffee(const RoboCompAprilTags::tag &t, AGMModel::SPtr &newModel)
{

	return false;
}

void SpecificWorker::getIDsFor(std::string obj, int32_t &objectSymbolID, int32_t &objectStSymbolID)
{
	objectSymbolID = -1;
	objectStSymbolID = -1;

	QStringList actions = QString::fromStdString(params["plan"].value).toLower().split("\n");

	for (auto a : actions)
	{
		if (a.contains(QString::fromStdString(obj)))
		{
			QStringList parts = a.split("'");
			for (int32_t index=0; index<parts.size()-2; index++)
			{
				if      (parts[index] == "objectr") objectSymbolID   = parts[index+2].toInt();
				else if (parts[index] == "statusr") objectStSymbolID = parts[index+2].toInt();
			}
		}
	}
	printf("------------------------------->%d %d\n", objectSymbolID, objectStSymbolID);
}

void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
{
	static QTime lastTime;

	if (newAction)
		lastTime = QTime::currentTime();

	if (lastTime.elapsed() > 5000)
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		auto symbols = newModel->getSymbolsMap(params, "container");
		auto node = symbols["container"];

		for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
		{
			if ((*edge_itr)->getLabel() == "noExplored")
			{
				(*edge_itr)->setLabel("explored");
				sendModificationProposal(worldModel, newModel);
				return;
			}
		}

	}
}




