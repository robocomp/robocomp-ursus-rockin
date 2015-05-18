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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>



/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";

// 	bodyinversekinematics_proxy->goHome("RIGHTARM");
// 	setRightArmUp_Reflex();

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	QMutexLocker locker(mutex);

	// STUFF
	updateInnerModel();
	manageReachedObjects();

	// ACTION EXECUTION
	actionExecution();

}

#define THRESHOLD 400.

void SpecificWorker::manageReachedObjects()
{
// 	printf("<<<<<<<<<<<<<<<< REACHED OBJECTS\n");
// 	printf("<<<<<<<<<<<<<<<< REACHED OBJECTS\n");

	bool changed = false;
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	for (AGMModel::iterator symbol_itr=newModel->begin(); symbol_itr!=newModel->end(); symbol_itr++)
	{
		AGMModelSymbol::SPtr node = *symbol_itr;
		if (node->symboltype() == "object")
		{
			// Avoid working with rooms
			if (isRoom(newModel, node)) continue;

			/// Compute distance and new state
			float d2n = distanceToNode("base_head", newModel, node);
			printf("distance: %d(%s)=%f\n", node->identifier, node->symbolType.c_str(), d2n);
			for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(newModel); edge_itr!=node->edgesEnd(newModel); edge_itr++)
			{
				AGMModelEdge &edge = *edge_itr;
				if (edge->getLabel() == "reach" and d2n > THRESHOLD)
				{
					edge->setLabel("noReach");
					printf("object %d STOPS REACH\n", node->identifier);
					changed = true;
				}
				else if (edge->getLabel() == "noReach" and d2n < THRESHOLD)
				{
					edge->setLabel("reach");
					printf("___ %s ___\n", edge->getLabel().c_str());
					printf("object %d STARTS REACH\n", node->identifier);
					changed = true;
				}
			}
		}
	}

	/// Publish new model if changed
	if (changed)
	{
		printf("PUBLISH!!!!\n");
		AGMModelPrinter::printWorld(newModel);

		sendModificationProposal(newModel, worldModel);
	}


// 	printf(">>>>>>>>>>>>>>>> REACHED OBJECTS\n");
// 	printf(">>>>>>>>>>>>>>>> REACHED OBJECTS\n");


/*
	float alpha;
	switch (objectId)
	{
		case 7:
			alpha = -3.141592;
			break;
		case 9:
			alpha = 0;
			break;
		default:
			qFatal("ee");
			break;
	}
	// printf("object (%f, %f, %f)\n", x, z, alpha);

	const int32_t robotId = worldModel->getIdentifierByType("robot");
	AGMModelSymbol::SPtr robot = worldModel->getSymbolByIdentifier(robotId);
	const float rx = str2float(robot->getAttribute("tx"));
	const float rz = str2float(robot->getAttribute("tz"));
	const float ralpha = str2float(robot->getAttribute("alpha"));

	// Avoid repeating the same goal and confuse the navigator
	const float errX = abs(rx-x);
	const float errZ = abs(rz-z);
	float errAlpha = abs(ralpha-alpha);
	while (errAlpha > +M_PIl) errAlpha -= 2.*M_PIl;
	while (errAlpha < -M_PIl) errAlpha += 2.*M_PIl;
	errAlpha = abs(errAlpha);

	printf("%f %f %f\n", rx, rz, ralpha);
	printf("%f %f %f\n",  x,  z,  alpha);
	printf("%f %f %f\n", errX, errZ, errAlpha);
	if (errX<300 and errZ<300 and errAlpha<0.4)
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		std::map<std::string, AGMModelSymbol::SPtr> symbols;
		try
		{
			symbols = newModel->getSymbolsMap(params, "object", "status");
		}
		catch(...)
		{
			printf("graspingAgent: Couldn't retrieve action's parameters\n");
			return;
		}

		if (newModel->renameEdge(symbols["object"], symbols["status"], "noReach", "reach"))
		{
			try
			{
				sendModificationProposal(newModel, worldModel);
				printf("sent reach\n");
			}
			catch(const Ice::Exception& ex)
			{
				printf("graspingAgent: Couldn't publish new model\n");
				cout << "Exception: " << ex << endl;
				return ;
			}
		}
		else
		{
			printf("already reaching??\n");
		}
	}
*/
}

bool SpecificWorker::isRoom(AGMModel::SPtr model, AGMModelSymbol::SPtr node)
{
	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model); edge_itr++)
	{
		AGMModelEdge edge = *edge_itr;
		if (edge->getLabel() == "room")
		{
			return true;
		}
	}
	return false;
}

// std::vector<std::pair<float, float>> getCoordinates

float SpecificWorker::distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr node)
{
	printf("distance node %d\n", node->identifier);
	bool isPolygon = false;
	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model) and isPolygon == false; edge_itr++)
	{
		if ((*edge_itr)->getLabel() == "table") isPolygon = true;
	}

	printf("isPolygon: %d\n", isPolygon);

	const float x = str2float(node->getAttribute("tx"));
	const float y = str2float(node->getAttribute("ty"));
	const float z = str2float(node->getAttribute("tz"));

	if (isPolygon)
	{
		const std::string polygon = node->getAttribute("polygon");
		const QVec head_in_floor = innerModel->transform("world", reference_name.c_str());
		return distanceToPolygon(head_in_floor, QVec::vec3(x, y, z), polygon);
	}
	else
	{
		return innerModel->transform("base_head", QVec::vec3(x,y,z), "world").norm2();
	}
}

float SpecificWorker::distanceToPolygon(QVec reference, QVec position, std::string polygon_str)
{
	boost::geometry::model::d2::point_xy<int> point(reference(0), reference(2));
	boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float> > poly;

// 	printf("p %s\n", polygon_str.c_str());
	std::vector<std::string> strs;
	boost::split(strs, polygon_str, boost::is_any_of(";"));
// 	printf("æ %f  %f\n", position(0);
	for (auto coords : strs)
	{
// 		printf("pp %s\n", coords.c_str());
		std::vector<std::string> strs_coords;
		boost::split(strs_coords, coords, boost::is_any_of("(),"));
		if (strs_coords.size()<2)
			return std::nan("1");
// 		for (auto ss : strs_coords) printf("<%d %s\n", ddd++, ss.c_str());
// 		printf(" s %d\n", strs_coords.size());
		const float x = atof(strs_coords[1].c_str());
		const float z = atof(strs_coords[2].c_str());
		printf("< %f %f\n", x, z);
		boost::geometry::model::d2::point_xy<float> vertex(x, z);
		boost::geometry::append(poly, vertex);
	}


	return boost::geometry::distance(poly, point);
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("GraspingAgent.InnerModel") ;
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
		backAction = action;
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

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmagenttopic, worldModel, "graspingAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::actionExecution()
{
	static std::string previousAction = "";
	bool newAction = (previousAction != action);

	if (newAction)
		printf("prev:%s  new:%s\n", previousAction.c_str(), action.c_str());

	if (action == "findobjectvisuallyintable")
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

	if (newAction)
	{
		previousAction = action;
		printf("New action: %s\n", action.c_str());
	}
}


void SpecificWorker::directGazeTowards(AGMModelSymbol::SPtr symbol)
{
	try
	{
		const float x = str2float(symbol->getAttribute("tx"));
		const float y = str2float(symbol->getAttribute("ty"));
		const float z = str2float(symbol->getAttribute("tz"));
		QVec worldRef = QVec::vec3(x, y, z);
		QVec robotRef = innerModel->transform("robot", worldRef, "world");
		printf("saccadic3D\n");
		printf("\n");
		saccadic3D(robotRef, QVec::vec3(0,0,1));
	}
	catch(...)
	{
		printf("directGazeTowards\n");
		throw;
	}
}


void SpecificWorker::action_FindObjectVisuallyInTable(bool first)
{
	try
	{
		int32_t tableId = str2int(params["container"].value);
		directGazeTowards(worldModel->getSymbol(tableId));
	}
	catch(...)
	{
		printf("Can't get the symbol for the container (table)\n[%s: %d]\n", __FILE__, __LINE__);
		throw;
	}
}



void SpecificWorker::action_GraspObject(bool first)
{
	const QVec offset = QVec::vec3(150, 0, 0);
	static int32_t state = 0;
	static QTime time;
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	if (first) state = 0;
	printf("action_GraspObject: first:%d  state=%d\n", (int)first, state);

	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = newModel->getSymbolsMap(params, "object", "table", "robot");
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}


	if (state == 0) // 1º approach hand
	{
		printf("%s: %d\n", __FILE__, __LINE__);
		try
		{
			bodyinversekinematics_proxy->setFingers(80);
		}
		catch(...)
		{
			qFatal("%s: %d\n", __FILE__, __LINE__);
		}
		printf("%s: %d\n", __FILE__, __LINE__);
		usleep(100000);
		AGMModelSymbol::SPtr symbol;
		try
		{
			printf("%s: %d\n", __FILE__, __LINE__);
			symbol = symbols["object"];
			printf("%s: %d\n", __FILE__, __LINE__);
			sendRightArmToTargetFullPose(symbol, offset);
			printf("%s: %d\n", __FILE__, __LINE__);
		}
		catch (...)
		{
			printf("%s: %d\n", __FILE__, __LINE__);
		}

		time = QTime::currentTime();
		state = 1;
	}
	else if (state == 1) // wait
	{
		if (time.elapsed() > 4000) { state = 0; }
		else
		{
			auto distance = innerModel->transform("grabPositionHandR", getObjectsLocation(symbols["object"])+offset, "world").norm2();
			printf("state==1: %f\n", distance);
			if (distance < 50)
			{
				state = 2;
			}
		}
	}
	else if (state == 2) // 2º get the hand into the object
	{
		bodyinversekinematics_proxy->setFingers(100);
		usleep(100000);
		sendRightArmToTargetFullPose(symbols["object"], offset.operator*(0.4));
		time = QTime::currentTime();
		state = 3;
	}
	else if (state == 3) // wait
	{
		if (time.elapsed() > 4000) { state = 0; }
		else
		{
			auto distance = innerModel->transform("grabPositionHandR", getObjectsLocation(symbols["object"]) + offset.operator*(0.4), "world").norm2();
			printf("state==3: %f\n", distance);
			if (distance < 5)
			{
				state = 4;
			}
		}
	}
	else if (state == 4) // 3º get the hand into the object
	{
		bodyinversekinematics_proxy->setFingers(50);
		time = QTime::currentTime();
		state = 5;
	}
	else if (state == 5) // wait
	{
		if (time.elapsed() > 4000) { state = 0; }
		else
		{
			auto distance = innerModel->transform("grabPositionHandR", getObjectsLocation(symbols["object"]) + offset.operator*(0.4), "world").norm2();
			printf("state==5: %f\n", distance);
			if (distance < 5)
			{
				state = 6;
			}
		}
	}
	else if (state == 6)
	{
		try
		{
			bodyinversekinematics_proxy->setFingers(50);
			usleep(500000);
// 			qFatal("got it!!!! :-D");
			newModel->removeEdge(symbols["object"], symbols["table"], "in");
			newModel->addEdge(   symbols["object"], symbols["robot"], "in");
			sendModificationProposal(newModel, worldModel);
			time = QTime::currentTime();
			state = 7;
		}
		catch(...)
		{
			printf("graspingAgent: Couldn't publish new model\n");
		}
	}
	else
	{
		if (time.elapsed() > 4000)
		{
			state = 0;
		}
	}

}


QVec SpecificWorker::getObjectsLocation(AGMModelSymbol::SPtr &object)
{
printf("%s: %d\n", __FILE__, __LINE__);
	return QVec::vec3(
	 str2float(object->getAttribute("tx")),
	 str2float(object->getAttribute("ty")),
	 str2float(object->getAttribute("tz"))
	);
}

void SpecificWorker::sendRightArmToTargetFullPose(AGMModelSymbol::SPtr &targetObject, QVec offset)
{
	Pose6D target;
	WeightVector weights;
printf("%s: %d\n", __FILE__, __LINE__);
	try
	{
		printf("%s: %d\n", __FILE__, __LINE__);
		QVec targetPos = getObjectsLocation(targetObject) + offset;
		printf("%s: %d\n", __FILE__, __LINE__);
		targetPos.print("targetFullPose");
		printf("%s: %d\n", __FILE__, __LINE__);
		target.x = targetPos(0);
		target.y = targetPos(1);
		target.z = targetPos(2);
		weights.x = 1;
		weights.y = 1;
		weights.z = 1;
		printf("%s: %d\n", __FILE__, __LINE__);
		target.rx = str2float(targetObject->getAttribute("rx"));
		target.ry = str2float(targetObject->getAttribute("ry"));
		target.rz = str2float(targetObject->getAttribute("rz"));
		printf("%s: %d\n", __FILE__, __LINE__);
		weights.rx = 1;
		weights.ry = 1;
		weights.rz = 1;
		printf("%s: %d\n", __FILE__, __LINE__);
	}
	catch (...)
	{
		printf("graspingAgent: Error reading data from cognitive model (symbol %d): (%s:%d)\n", targetObject->identifier, __FILE__, __LINE__);
	}
	try
	{
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 1.);
	}
	catch (...)
	{
		printf("graspingAgent: Couldn't set RIGHTARM target (maybe a communication problem?)\n");
	}
}

void SpecificWorker::sendRightArmToTargetPosition(AGMModelSymbol::SPtr &targetObject, QVec offset)
{
	Pose6D target;
	WeightVector weights;
	try
	{
		QVec targetPos = getObjectsLocation(targetObject) + offset;
		targetPos.print("targetPosition");
		target.x = targetPos(0);
		target.y = targetPos(1);
		target.z = targetPos(2);
		weights.x = 1;
		weights.y = 1;
		weights.z = 1;
		target.rx = 0;
		target.ry = 0;
		target.rz = 0;
		weights.rx = 0;
		weights.ry = 0;
		weights.rz = 0;
	}
	catch (...)
	{
		printf("graspingAgent: Error reading data from cognitive model (symbol %d): (%s:%d)\n", targetObject->identifier, __FILE__, __LINE__);
	}
	try
	{
		bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 1.);
	}
	catch (...)
	{
		printf("graspingAgent: Couldn't set RIGHTARM target (maybe a communication problem?)\n");
	}
}

void SpecificWorker::action_SetObjectReach(bool first)
{
	printf("void SpecificWorker::action_SetObjectReach()\n");

	///
	///  Lift the hand if it's down, to avoid collisions
	///
	if (backAction != "setobjectreach" or innerModel->transform("root", "grabPositionHandR")(1)<1000)
	{
			backAction = action;
			printf("first time, set arm for manipulation\n");
			setRightArmUp_Reflex();
	}


	///
	/// Track the target
	///
	int32_t objectId = -1;
	try
	{
		objectId = str2int(params["object"].value);
	}
	catch (...)
	{
		printf("%s %d\n", __FILE__, __LINE__);
	}
	if (objectId > 0)
	{
		try
		{
			AGMModelSymbol::SPtr goalObject = worldModel->getSymbol(objectId);
			const float x = str2float(goalObject->getAttribute("tx"));
			const float y = str2float(goalObject->getAttribute("ty"));
			const float z = str2float(goalObject->getAttribute("tz"));
			saccadic3D(QVec::vec3(x,y,z), QVec::vec3(0,0,1));
		}
		catch (...)
		{
			printf("%s %d\n", __FILE__, __LINE__);
		}
	}
	else
	{
		printf ("don't have the object to reach in my model %d\n", objectId);
	}

	printf("--------------------\n");

	///
	/// No more work to do. The label is set passively (from this agent's point of view)
	///
}


void SpecificWorker::saccadic3D(QVec point, QVec axis)
{
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
// 	printf("saccadic3D\n");

	QVec rel = innerModel->transform("rgbd", QVec::vec3(tx, ty, tz), "world");
// 	rel.print("desde la camara");

	float errYaw   = -atan2(rel(0), rel(2));
	float errPitch = +atan2(rel(1), rel(2));
// 	printf("%f  %f\n", errYaw, errPitch);

	RoboCompJointMotor::MotorGoalPosition goal;

	goal.name = "head_yaw_joint";
	goal.maxSpeed = 0.5;
	goal.position = jointmotor_proxy->getMotorState("head_yaw_joint").pos - errYaw;
	jointmotor_proxy->setPosition(goal);

	goal.name = "head_pitch_joint";
	goal.maxSpeed = 0.5;
	goal.position = jointmotor_proxy->getMotorState("head_pitch_joint").pos - errPitch;
	jointmotor_proxy->setPosition(goal);

/*
	RoboCompBodyInverseKinematics::Pose6D targetSight;
	targetSight.x = tx;
	targetSight.y = ty;
	targetSight.z = tz;
	RoboCompBodyInverseKinematics::Axis axSight;
	axSight.x = axx;
	axSight.y = axy;
	axSight.z = axz;
	bool axisConstraint = false;
	float axisAngleConstraint = 0;
	try
	{
		bodyinversekinematics_proxy->stop("HEAD");
		usleep(500000);
		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", targetSight, axSight, axisConstraint, axisAngleConstraint);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
*/

}



void SpecificWorker::updateInnerModel()
{
	try
	{
		AGMModelSymbol::SPtr robot = worldModel->getSymbol(worldModel->getIdentifierByType("robot"));
		const float x     = str2float(robot->getAttribute("tx"));
		const float z     = str2float(robot->getAttribute("tz"));
		const float alpha = str2float(robot->getAttribute("alpha"));
		innerModel->updateTransformValues("robot", x, 0, z, 0, alpha, 0);

		MotorStateMap mstateMap;
		jointmotor_proxy->getAllMotorState(mstateMap);
		for (auto &joint : mstateMap)
		{
			innerModel->updateJointValue(QString::fromStdString(joint.first), joint.second.pos);
		}

	}
	catch(...)
	{
		return;
	}
}


void SpecificWorker::setRightArmUp_Reflex()
{
	bodyinversekinematics_proxy->setJoint("rightShoulder1", -1.6, 0.3);
	bodyinversekinematics_proxy->setJoint("rightShoulder2", -0.6, 0.3);
	bodyinversekinematics_proxy->setJoint("rightShoulder3", 0.25, 0.3);
	bodyinversekinematics_proxy->setJoint("rightElbow", 1.9, 0.5);
	bodyinversekinematics_proxy->setJoint("rightForeArm", 0.39, 0.3);
	bodyinversekinematics_proxy->setJoint("rightWrist1", 0.4, 0.3);
	bodyinversekinematics_proxy->setJoint("rightWrist2", 0.0, 0.3);
}


void SpecificWorker::setRightArm_GRASP_0_Reflex()
{
	bodyinversekinematics_proxy->setJoint("rightShoulder1", -1.6, 0.3);
	bodyinversekinematics_proxy->setJoint("rightShoulder2", -0.6, 0.3);
	bodyinversekinematics_proxy->setJoint("rightShoulder3", 0.25, 0.3);
	bodyinversekinematics_proxy->setJoint("rightElbow", 1.9, 0.5);
	bodyinversekinematics_proxy->setJoint("rightForeArm", 0.39, 0.3);
	bodyinversekinematics_proxy->setJoint("rightWrist1", 0.4, 0.3);
	bodyinversekinematics_proxy->setJoint("rightWrist2", 0.0, 0.3);
}


