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
	//  RELOCALIZATION
	//
	// to be done
	//


	//  UPDATE ROBOT'S LOCATION IN COGNITIVE MAP
	//
	updateRobotsLocation();

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
	if (roomsPolygons.size()==0)
		extractPolygonsFromModel(worldModel);
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
	static float lastX = std::numeric_limits<float>::quiet_NaN();
	static float lastZ = std::numeric_limits<float>::quiet_NaN();

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
		printf("%s\n", action.c_str());
		AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
		const float x = str2float(goalRoom->getAttribute("x"));
		const float z = str2float(goalRoom->getAttribute("z"));

		bool proceed = true;
		if ( (planningState.state=="PLANNING" or planningState.state=="EXECUTING") )
		{
			if (lastX == x and lastZ == z)
				proceed = false;
		}

		if (proceed)
		{
			printf("changeroom from %s to %s\n", params["r1"].value.c_str(), params["r2"].value.c_str());
			go((lastX=x), (lastZ=z));
		}
		else
		{
			printf("%s\n", planningState.state.c_str());
		}
	}
}

void SpecificWorker::updateRobotsLocation()
{
	
}


std::map<int32_t, QPolygonF> SpecificWorker::extractPolygonsFromModel(AGMModel::SPtr &worldModel)
{
	std::map<int32_t, QPolygonF> ret;
	
	for (AGMModel::iterator symbol_it=worldModel->begin(); symbol_it!=worldModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object")
		{
			for (AGMModelSymbol::iterator edge_it=symbol->edgesBegin(worldModel); edge_it!=symbol->edgesEnd(worldModel); edge_it++)
			{
				AGMModelEdge edge = *edge_it;
				if (edge.linking == "room")
				{
					const QString polygonString = QString::fromStdString(symbol->getAttribute("polygon"));
					const QStringList coords = polygonString.split(";");
					if (coords.size() < 3)
					{
						qFatal("%s %d", __FILE__, __LINE__);
					}
					
					QVector<QPointF> points;
					for (int32_t ci=0; ci<coords.size(); ci++)
					{
						const QString &pointStr = coords[ci];
						if (pointStr.size() < 5) qFatal("%s %d", __FILE__, __LINE__);
						const QStringList coords2 = polygonString.split(",");
						if (coords2.size() < 2) qFatal("%s %d", __FILE__, __LINE__);
						QString a = coords2[0];
						QString b = coords2[1];
						a.remove(0,1);
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
	
	return ret;
}


