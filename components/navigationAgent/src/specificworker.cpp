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
	innerModel = new InnerModel();
	haveTarget = false;
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	printf("compute 1..\n");
	// ODOMETRY AND LOCATION-RELATED ISSUES	
	if (odometryAndLocationIssues()==false)
		return;
	printf("compute 2..\n");
// 	innerModel->treePrint();
	
	actionExecution();
	
// 	printf("ae>\n");
}


void SpecificWorker::actionExecution()
{
	QMutexLocker locker(mutex);
	printf("<<actionExecution\n");

	static std::string previousAction = "";
	bool newAction = (previousAction != action);

// 	if (newAction)
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());

// 	try
// 	{
// 		planningState = trajectoryrobot2d_proxy->getState();
// 	}
// 	catch(const Ice::Exception &ex)
// 	{
// 		std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;
// 	}
// 	catch(...)
// 	{
// 		printf("something else %d\n", __LINE__);
// 	}

	if (action == "changeroom")
	{
		action_ChangeRoom(newAction);
	}
	else if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable(newAction);
	}
	else if (action == "setobjectreach")
	{
		action_SetObjectReach(newAction);
	}
	else if (action == "graspobject")
	{
		action_GraspObject(newAction);
	}
	else
	{
		std::cout<<" "<< action;
		//action_NoAction();
	}
	
	
	printf("actionExecution>>\n");
}

void SpecificWorker::action_SetObjectReach(bool newAction)
{
	std::cout<<"action "<<action<<" 1\n";
	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = worldModel->getSymbolsMap(params/*,  "robot", "room", "object", "status"*/);
		for (auto dd : symbols)
		{
			printf("tenemos %s\n", dd.first.c_str());
		}
	}
	catch(...)
	{
		printf("navigationAgent: Couldn't retrieve action's parameters\n");
		printf("<<WORLD\n");
		AGMModelPrinter::printWorld(worldModel);
		printf("WORLD>>\n");
		if (worldModel->size() > 0)
		{
			exit(-1);
		}
	}
		
	std::cout<<"action "<<action<<" 2\n";
	// ACTION EXECUTION
	if (action=="setobjectreach")
	{
		int roomID, objectID, robotID;
		try
		{
			roomID = symbols["room"]->identifier;//7;
			objectID =symbols["object"]->identifier;//9;//
			robotID = symbols["robot"]->identifier;//9;
		}
		catch(...)
		{
			printf("bla blakedoij ewr\n");
			exit(2);
		}


		RoboCompTrajectoryRobot2D::TargetPose tp;
		///x z del modelo de agm 7--RT-->9 ,  y el angulo final del robot es el relativo al angulo con la mesa
		
		printf("%s\n", __LINE__);
		AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(roomID,objectID, "RT");
		tp.x = str2float(edge->getAttribute("tx") );
		tp.z = str2float(edge->getAttribute("tz") ) ;
		tp.y = 0.;
		tp.rx=tp.rz=0.0;
		tp.doRotation=true;
		
		//el angulo final del robot gracias a innerModel!
		QString  robotId =QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
		QString  tableId =QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));
		
		innerModel->transform6D("room","robot").print("robot pose in room");
		innerModel->transform6D("room","table").print("table pose in room");
		innerModel->transform6D("robot","table").print("table pose from robot");
		try
		{
			qDebug()<<"innerModel->transform("<<tableId<<robotId<<")";
			//innerModel->transform(tableId,robotId).print("transform");
// 				innerModel->transform6D(tableId,robotId).print("transform6D");
			tp.ry =innerModel->transform6D(robotId,tableId).ry();
		}
		catch (...)
		{
			qDebug()<<"innerModel exception";
		}
		try
		{
// 				std::cout<<"trajectoryrobot2d_proxy->go( "<<tp.x<<","<<tp.z<<","<<tp.ry<<")\n";
// 				qDebug()<<"\n***nos volvemos sin mover el robot***\n";
// 				return;
			if (!haveTarget)
			{
				try
				{
					trajectoryrobot2d_proxy->go(tp);										
					std::cout<<"trajectoryrobot2d_proxy->go( "<<tp.x<<","<<tp.z<<","<<tp.ry<<")\n";
					haveTarget=true;
				}
				catch(const Ice::Exception &ex)
				{
					std::cout <<"trajectoryrobot2d_proxy->go "<< ex << std::endl;
					throw ex;
				}
				
			}
			string state;
			try
			{
					state= trajectoryrobot2d_proxy->getState().state;						
			}
			catch(const Ice::Exception &ex)
			{
					std::cout <<"trajectoryrobot2d_proxy->getState().state "<< ex << std::endl;
					throw ex;
			}
			
			//state="IDLE";
			std::cout<<state<<" haveTarget "<<haveTarget;
			if (state=="IDLE" && haveTarget)
			{
				//std::cout<<"\ttrajectoryrobot2d_proxy->getState() "<<trajectoryrobot2d_proxy->getState().state<<"\n";
				try
				{
					AGMModel::SPtr newModel(new AGMModel(worldModel));
					//objectID has been obtained before
					int statusID =symbols["status"]->identifier;
					newModel->getEdgeByIdentifiers(objectID, statusID, "noReach").setLabel("reach");
					sendModificationProposal(worldModel, newModel);
					haveTarget=false;
				}
				catch (...)
				{
					std::cout<<"\neeeee"<< "\n";
					std::cout<<"\tedge->toString() "<<edge->toString(worldModel)<<" not exist\n";
				}
			}
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
		}
	}	


}


bool SpecificWorker::odometryAndLocationIssues()
{
	//
	// Get ODOMETRY and update it in the graph. If there's a problem talking to the robot's platform, abort
	try
	{
		omnirobot_proxy->getBaseState(bState);
	}
	catch(...)
	{
		printf("Can't connect to the robot!!\n");
		return false;
	}

	int32_t robotId, roomId;
	
	robotId = worldModel->getIdentifierByType("robot");
	if (robotId < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return false;
	}

	roomId=7;
	if (roomId < 0)
	{			
		printf("roomId not found, Waiting for Insert innerModel...\n");
		return false;
	}
	
	try
	{
		AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(roomId, robotId, "RT");
		//printf("a %d\n", __LINE__);
		
		float bStatex =str2float(edge->getAttribute("tx"));
		float bStatez = str2float(edge->getAttribute("tz"));
		float bStatealpha = str2float(edge->getAttribute("ry"));
		//to reduces the publication frequency
		if (fabs(bStatex - bState.correctedX)>50 or fabs(bStatez - bState.correctedZ)>50 or fabs(bStatealpha - bState.correctedAlpha)>0.2)
		{			
			//Publish update edge
			printf("\nUpdate odometry...\n");
			qDebug()<<"bState local :"<<bStatex<<bStatez<<bStatealpha;
			qDebug()<<"bState corrected"<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;
			
			edge->setAttribute("tx", float2str(bState.correctedX));
			edge->setAttribute("tz", float2str(bState.correctedZ));
			edge->setAttribute("ry", float2str(bState.correctedAlpha));		
			qDebug()<<"TODO: AGMMisc::publishEdgeUpdate(edge, agmagenttopic_proxy)\n\n";
			AGMMisc::publishEdgeUpdate(edge, agmagenttopic_proxy);
		}
	}
	catch (...)
	{
		printf("Can't update odometry in RT, edge does not exist? !!!\n");
		return false;
	}

// 		printf("a %d\nv", __LINE__);
	return true;
}

void SpecificWorker::updateRobotsCognitiveLocation()
{
	// If the polygons are not set yet, there's nothing to do...
	if (roomsPolygons.size()==0)
		return;

	// Get current location according to the model, if the location is not set yet, there's nothing to do either
	const int32_t currentLocation = getIdentifierOfRobotsLocation(worldModel);
	if (currentLocation == -1) return;

	// Compute the robot's location according to the odometry and the set of polygons
	// If we can't find the room where the robot is, we assume it didn't change, so there's nothing else to do
	int32_t newLocation = -1;
	for (auto &kv : roomsPolygons)
	{
		if (kv.second.containsPoint(QPointF(bState.x,  bState.z), Qt::OddEvenFill))
		{
			newLocation = kv.first;
			break;
		}
	}
	if (newLocation == -1) return;

	// If everyting is ok AND the robot changed its location, update the new location in the model and
	// propose the change to the executive
	if (newLocation != currentLocation and newLocation != -1)
	{
		printf("sending modification!\n");
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		setIdentifierOfRobotsLocation(newModel, newLocation);
		AGMModelPrinter::printWorld(newModel);
		sendModificationProposal(worldModel, newModel);
	}
}


std::map<int32_t, QPolygonF> SpecificWorker::extractPolygonsFromModel(AGMModel::SPtr &worldModel)
{
	std::map<int32_t, QPolygonF> ret;

	for (AGMModel::iterator symbol_itRR=worldModel->begin(); symbol_itRR!=worldModel->end(); symbol_itRR++)
	{
		const AGMModelSymbol::SPtr &symbolRR = *symbol_itRR;
		if (symbolRR->symbolType == "robot")
		{
			for (AGMModelSymbol::iterator edge_itRR=symbolRR->edgesBegin(worldModel); edge_itRR!=symbolRR->edgesEnd(worldModel); edge_itRR++)
			{
				AGMModelEdge edgeRR = *edge_itRR;
				if (edgeRR.linking == "know")
				{
					const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(edgeRR.symbolPair.first);
					if (symbol->symbolType == "object")
					{
						printf("object: %d\n", symbol->identifier);
						for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
						{
							AGMModelEdge edge = *edge_it;
							if (edge.linking == "room")
							{
								const QString polygonString = QString::fromStdString(symbol->getAttribute("polygon"));
								const QStringList coords = polygonString.split(";");
								printf("  it is a room\n");
								qDebug() << " " << coords.size() << " ___ " << polygonString ;
								if (coords.size() < 3)
								{
									qDebug() << coords.size() << " ___ " << polygonString ;
									qDebug() << polygonString;
									for (int32_t i=0; i<coords.size(); i++)
										qDebug() << coords[i];
									qFatal("ABORT %s %d", __FILE__, __LINE__);
								}

								QVector<QPointF> points;
								for (int32_t ci=0; ci<coords.size(); ci++)
								{
									const QString &pointStr = coords[ci];
									if (pointStr.size() < 5) qFatal("%s %d", __FILE__, __LINE__);
									const QStringList coords2 = pointStr.split(",");
									if (coords2.size() < 2) qFatal("%s %d", __FILE__, __LINE__);
									QString a = coords2[0];
									QString b = coords2[1];
									a.remove(0,1);
									b.remove(b.size()-1,1);
									float x = a.toFloat();
									float z = b.toFloat();
									points.push_back(QPointF(x, z));
								}
								if (points.size() < 3) qFatal("%s %d", __FILE__, __LINE__);
								ret[symbol->identifier] = QPolygonF(points);
							}
						}
					}
				}
			}
		}
	}

	return ret;
}

int32_t SpecificWorker::getIdentifierOfRobotsLocation(AGMModel::SPtr &worldModel)
{
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				AGMModelEdge edge = *edge_it;
				if (edge.linking == "in")
				{
					return edge.symbolPair.second;
				}
			}
		}
	}

	printf("wheres's the robot?\n");
	return -1;
}

void SpecificWorker::setIdentifierOfRobotsLocation(AGMModel::SPtr &model, int32_t identifier)
{
	bool didSomethin = false;
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "robot")
		{

			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("it was %d\n", edge_it->symbolPair.second);
				}
			}
			for (int32_t edgeIndex=0; edgeIndex<model->numberOfEdges(); edgeIndex++)
			{
				if (model->edges[edgeIndex].linking == "in")
				{
					if (model->edges[edgeIndex].symbolPair.first == symbol->identifier)
					{
						model->edges[edgeIndex].symbolPair.second = identifier;
						didSomethin = true;
					}
				}
			}
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				if (edge_it->linking == "in")
				{
					printf("now is %d\n", edge_it->symbolPair.second);
				}
			}
		}
	}
	if (not didSomethin)
		qFatal("couldn't update robot's room in the cog graph");
}


void SpecificWorker::action_ChangeRoom(bool newAction)
{
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
	const float x = str2float(goalRoom->getAttribute("tx"));
	const float z = str2float(goalRoom->getAttribute("tz"));

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("changeroom from %s to %s\n", params["r1"].value.c_str(), params["r2"].value.c_str());
		go(x, z);
	}
	else
	{
	}
}


void SpecificWorker::action_FindObjectVisuallyInTable(bool newAction)
{
	stop();


	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	AGMModelSymbol::SPtr goalTable;
	AGMModelSymbol::SPtr robot;
	int32_t tableId = -1;
	try
	{
		tableId = str2int(params["container"].value);
		goalTable = worldModel->getSymbol(tableId);
		robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
	}
	catch(...)
	{
		printf("can't access robot or table\n");
		return;
	}

	const float x = str2float(goalTable->getAttribute("tx"));
	const float z = str2float(goalTable->getAttribute("tz"));
	float alpha = tableId==7?-3.141592:0;
// printf("%s: %d\n", __FILE__, __LINE__);

	const float rx = str2float(robot->getAttribute("tx"));
	const float rz = str2float(robot->getAttribute("tz"));
	const float ralpha = str2float(robot->getAttribute("alpha"));
// printf("%s: %d\n", __FILE__, __LINE__);

	// Avoid repeating the same goal and confuse the navigator
	const float errX = abs(rx-x);
	const float errZ = abs(rz-z);
	float errAlpha = abs(ralpha-alpha);
	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
	errAlpha = abs(errAlpha);
	if (errX<20 and errZ<20 and errAlpha<0.02)
		return;
// printf("%s: %d\n", __FILE__, __LINE__);

	bool proceed = true;
	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
	{
		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
			proceed = false;
		else
			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
	}
	else
	{
		printf("proceed because it's stoped\n");
	}

	if (proceed)
	{
		lastX = x;
		lastZ = z;
		printf("find objects in table %d\n", tableId);
		go(x, tableId==7?z+550:z-550, tableId==7?-3.141592:0, true, 0, 500, 500);
	}
	else
	{
	}
}








void SpecificWorker::action_GraspObject(bool newAction)
{
	printf("SpecificWorker::action_GraspObject\n");
	int32_t objectId;
	AGMModelSymbol::SPtr goalObject;
	AGMModelSymbol::SPtr robot;

	stop();

	try
	{
		objectId = str2int(params["object"].value);
		goalObject = worldModel->getSymbol(objectId);
		robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
	}
	catch (...)
	{
		printf("do we have the model yet\n?");
		return;
	}

	const float x = str2float(goalObject->getAttribute("tx"));
	const float z = str2float(goalObject->getAttribute("tz"));
	float alpha = (objectId==7 or objectId==100)?-3.141592:0;

	QVec::vec3( x, z, alpha).print("object");

	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = x;
	tp.z = z;
	tp.y = 0;
	tp.rx = 0;
	tp.ry = alpha;
	tp.rz = 0;
	trajectoryrobot2d_proxy->goReferenced(tp, 80, 350, 50);
}

void SpecificWorker::action_NoAction(bool newAction)
{
	static QTime time;
	static bool first = true;
	if (first)
	{
		first = false;
		time = QTime::currentTime();
		trajectoryrobot2d_proxy->stop();
	}
	else if (time.elapsed()>3000)
	{
		time = QTime::currentTime();
		stop();
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("NavigationAgent.InnerModel") ;
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

bool SpecificWorker::activateAgent(const ParameterMap& prs)
{
	bool activated = false;
	printf("<<activateAgent\n");
	if (setParametersAndPossibleActivation(prs, activated))
	{
			if (not activated)
			{
				printf("activateAgent 0 >>\n");
				return activate(p);
			}
	}
	else
	{
		printf("activateAgent 1 >>\n");
		return false;
	}
	printf("activateAgent 2 >>\n");
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
	printf("pre <<structuralChange\n");
	QMutexLocker l(mutex);
	printf("<<structuralChange\n");
	
	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	//if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
		//roomsPolygons = extractPolygonsFromModel(worldModel);
	
	agmInner.setWorld(worldModel);
	innerModel = agmInner.extractInnerModel("room");
	printf("structuralChange>>\n");
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	QMutexLocker l(mutex);
	printf("<<symbolUpdated\n");
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	innerModel = agmInner.extractInnerModel("room");
	printf("symbolUpdated>>\n");
}
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	QMutexLocker l(mutex);
	printf("<<edgeUpdated\n");
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	innerModel = agmInner.extractInnerModel("room");
	printf("edgeUpdated>>\n");
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
		printf("param:%s   value:%s\n", it->first.c_str(), it->second.value.c_str());
		params[it->first] = it->second;
	}
	printf("----\n");
	
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

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		//AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel, "navigationAgent");
		newModel->save("navigationAgent.xml");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::go(float x, float z, float alpha, bool rot, float xRef, float zRef, float threshold)
{
// 	printf("go:\n   %f %f %f (%d)\n  %f\n  %f %f\n", x, z, alpha, rot, threshold, xRef, zRef);
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = x;
	tp.z = z;
	tp.y = 0;
	tp.rx = 0;
	tp.ry = 0;
	tp.rz = 0;
	if (rot)
	{
		tp.ry = alpha;
		tp.doRotation = true;
	}
	else
	{
		tp.doRotation = false;
	}
	try
	{
		trajectoryrobot2d_proxy->goReferenced(tp, 80, 350, threshold);
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("something else %d\n", __LINE__);
	}
}


void SpecificWorker::stop()
{
	try
	{
		trajectoryrobot2d_proxy->stop();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
	catch(...)
	{
		printf("something else %d\n", __LINE__);
	}
}


// printf("void SpecificWorker::action_SetObjectReach()\n");
// 	static float lastX = std::numeric_limits<float>::quiet_NaN();
// 	static float lastZ = std::numeric_limits<float>::quiet_NaN();
// 	int32_t objectId = str2int(params["object"].value);
// 	AGMModelSymbol::SPtr goalObject;
// 	try
// 	{
// 		goalObject = worldModel->getSymbol(objectId);
// 	}
// 	catch(...)
// 	{
// 		printf("object %d not in our model\n", objectId);
// 		return;
// 	}
// 	const float x = str2float(goalObject->getAttribute("tx"));
// 	const float z = str2float(goalObject->getAttribute("tz"));
// 	float alpha;
// 	switch (objectId)
// 	{
// 		case 5:
// 			alpha = -0;
// 			break;
// 		case 7:
// 			alpha = -3.141592;
// 			break;
// 		case 9:
// 			alpha = 0;
// 			break;
// 		default:
// 			qFatal("navigation: can't get orientation goal for reaching object %d\n", objectId);
// 			break;
// 	}
// 	printf("object (%f, %f, %f)\n", x, z, alpha);
// 	const int32_t robotId = worldModel->getIdentifierByType("robot");
// 	AGMModelSymbol::SPtr robot = worldModel->getSymbolByIdentifier(robotId);
// 	const float rx = str2float(robot->getAttribute("tx"));
// 	const float rz = str2float(robot->getAttribute("tz"));
// 	const float ralpha = str2float(robot->getAttribute("alpha"));
// 	printf("robot (%f, %f, %f)\n", rx, rz, ralpha);
// 
// 	// Avoid repeating the same goal and confuse the navigator
// 	const float errX = abs(rx-x);
// 	const float errZ = abs(rz-z);
// 	float errAlpha = abs(ralpha-alpha);
// 	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
// 	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
// 	errAlpha = abs(errAlpha);
// 	if (errX<20 and errZ<20 and errAlpha<0.1)
// 		return;
// 
// 	bool proceed = true;
// 	if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
// 	{
// 		if (abs(lastX-x)<10 and abs(lastZ-z)<10)
// 		{
// 			proceed = false;
// 			printf("do not proceed because the coordinates do not differ (%s)\n", planningState.state.c_str());
// 		}
// 		else
// 		{
// 			proceed = true;
// 			printf("proceed because the coordinates differ (%f, %f), (%f, %f)\n", x, z, lastX, lastZ);
// 		}
// 	}
// 	else
// 	{
// 		proceed = true;
// 		printf("proceed because it's not planning or executing\n");
// 	}
// 
// 	static bool backp = true;
// 	if (proceed)
// 	{
// 		lastX = x;
// 		lastZ = z;
// 		printf("proceed setobjectreach %d\n", objectId);
// 		float xx = x;
// 		float zz = z;
// // 		objectId==7?z+550:z-550
// 		float aa = objectId==7?-3.141592:0;
// // 		qDebug() << xx << zz << aa;
// 		go(xx, zz, aa, true, 80, 150, 50);
// 		backp = true;
// 	}
// 	else if (backp)
// 	{
// 		printf("not proceeding %s\n", planningState.state.c_str());
// 		backp = false;
// 	}
// 
// 	printf("aaAdigejr\n");




