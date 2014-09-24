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
	// ODOMETRY AND LOCATION-RELATED ISSUES
	odometryAndLocationIssues();

	// ACTION EXECUTION
	//
	actionExecution();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("NavigationAgent.InnerModel") ;
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
	if (roomsPolygons.size()==0 and worldModel->numberOfSymbols()>0)
		roomsPolygons = extractPolygonsFromModel(worldModel);
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
		//AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic, worldModel, "navigation");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::go(float x, float z, float alpha, bool rot)
{
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
		tp.onlyRot = true;
	}
	else
	{
		tp.onlyRot = false;
	}
	try
	{
		trajectoryrobot2d_proxy->go(tp);
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
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

	try
	{
		planningState = trajectoryrobot2d_proxy->getState();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;
	}
	if (action == "changeroom")
	{
		action_ChangeRoom();
	}
	else if (action == "findobjectvisuallyintable")
	{
		action_FindObjectVisuallyInTable();
	}
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


void SpecificWorker::action_ChangeRoom()
{
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
	const float x = str2float(goalRoom->getAttribute("x"));
	const float z = str2float(goalRoom->getAttribute("z"));

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


void SpecificWorker::action_FindObjectVisuallyInTable()
{
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

	int32_t tableId = str2int(params["container"].value);
	AGMModelSymbol::SPtr goalTable = worldModel->getSymbol(tableId);
	const float x = str2float(goalTable->getAttribute("x"));
	const float z = str2float(goalTable->getAttribute("z"));

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
		go(x, tableId==7?z+400:z-400, tableId==7?-3.141592:0, true);
	}
	else
	{
		printf("%s\n", planningState.state.c_str());
	}
}



void SpecificWorker::odometryAndLocationIssues()
{
	//
	// Get ODOMETRY and update it in the graph. If there's a problem talking to the robot's platform, abort
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		AGMModelSymbol::SPtr robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
		robot->setAttribute("x", float2str(bState.x));
		robot->setAttribute("z", float2str(bState.z));
		robot->setAttribute("alpha", float2str(bState.alpha));
		AGMMisc::publishNodeUpdate(robot, agmagenttopic);
	}
	catch(...)
	{
		return;
	}

	//  RELOCALIZATION
	//
	// to be done
	//
	
	//  UPDATE ROBOT'S LOCATION IN COGNITIVE MAP
	//
	updateRobotsCognitiveLocation();
}


