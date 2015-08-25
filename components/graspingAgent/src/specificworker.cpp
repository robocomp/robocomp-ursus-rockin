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

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;

	mutex_AGM_IM = new QMutexDebug(QMutex::Recursive, QString("AGM & IM"));
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

	setRightArmUp_Reflex();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::compute( )
{
printf("%s: %d\n", __FILE__, __LINE__);
	printf("compute %d\n\n", __LINE__);
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
	{
		if (not innerModel->getNode("grabPositionHandR"))
		{
			printf("waiting for AGM*\n");
			return;
		}
	}
	printf("manageReachedObjects %d\n\n", __LINE__);
	manageReachedObjects();
	printf("manageReachedObjects %d\n\n", __LINE__);
	

	// ACTION EXECUTION
	actionExecution();
// 	printf("compute %d\n\n", __LINE__);

}


void SpecificWorker::manageReachedObjects()
{
printf("%s: %d\n", __FILE__, __LINE__);
	float THRESHOLD_object = 400;
	float THRESHOLD_table = 800;
	
	bool changed = false;
	
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
	
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	for (AGMModel::iterator symbol_itr=newModel->begin(); symbol_itr!=newModel->end(); symbol_itr++)
	{
		AGMModelSymbol::SPtr node = *symbol_itr;
		if (node->symboltype() == "object")
		{
			// Avoid working with rooms
			if (isObjectType(newModel, node, "room")) continue;

			/// Compute distance and new state
			float d2n;
			try
			{
				d2n = distanceToNode("arm_right_1", newModel, node);
			}
			catch(...)
			{
				printf("Ref: arm_right_1: %p\n", (void *)innerModel->getNode("arm_right_1"));
				printf("Obj: %s: %p\n", node->getAttribute("imName").c_str(), (void *)innerModel->getNode(node->getAttribute("imName").c_str()));
			}
			
			QVec arm = innerModel->transform("room", "arm_right_1");
			arm(1) = 0;
			QVec obj = innerModel->transformS("room", node->getAttribute("imName"));
			obj(1) = 0;

			float THRESHOLD = THRESHOLD_object;
			if (isObjectType(newModel, node, "table"))
			{
				THRESHOLD = THRESHOLD_table;
			}
			printf("%s: %f  (th:%f)\n", node->getAttribute("imName").c_str(), (arm-obj).norm2(), THRESHOLD);


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
		sendModificationProposal(newModel, worldModel);
	}
}

bool SpecificWorker::isObjectType(AGMModel::SPtr model, AGMModelSymbol::SPtr node, const std::string &t)
{
printf("%s: %d\n", __FILE__, __LINE__);
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);

	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model); edge_itr++)
	{
		AGMModelEdge edge = *edge_itr;
		if (edge->getLabel() == t)
		{
			return true;
		}
	}
	return false;
}

// std::vector<std::pair<float, float>> getCoordinates

float SpecificWorker::distanceToNode(std::string reference_name, AGMModel::SPtr model, AGMModelSymbol::SPtr node)
{
printf("%s: %d\n", __FILE__, __LINE__);

	
	// check if it's a polygon
// 	bool isPolygon = false;
// 	for (AGMModelSymbol::iterator edge_itr=node->edgesBegin(model); edge_itr!=node->edgesEnd(model) and isPolygon == false; edge_itr++)
// 	{
// 		if ((*edge_itr)->getLabel() == "table")
// 			isPolygon = true;
// 	}

// 	if (isPolygon)
// 	{
// 		const std::string polygon = node->getAttribute("polygon");
// 		const QVec head_in_floor = innerModel->transform("room", reference_name.c_str());
// 		return distanceToPolygon(head_in_floor, QVec::vec3(x, y, z), polygon);
// 	}
// 	else
// 	{
		QVec arm = innerModel->transformS("room", reference_name);
		arm(1) = 0;
		QVec obj = innerModel->transformS("room", node->getAttribute("imName"));
		obj(1) = 0;
		return (arm-obj).norm2();
// 	}
}

// float SpecificWorker::distanceToPolygon(QVec reference, QVec position, std::string polygon_str)
// {
// 	boost::geometry::model::d2::point_xy<int> point(reference(0), reference(2));
// 	boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float> > poly;
// 
// // 	printf("p %s\n", polygon_str.c_str());
// 	std::vector<std::string> strs;
// 	boost::split(strs, polygon_str, boost::is_any_of(";"));
// // 	printf("d %f  %f\n", position(0);
// 	for (auto coords : strs)
// 	{
// // 		printf("pp %s\n", coords.c_str());
// 		std::vector<std::string> strs_coords;
// 		boost::split(strs_coords, coords, boost::is_any_of("(),"));
// 		if (strs_coords.size()<2)
// 			return std::nan("1");
// // 		for (auto ss : strs_coords) printf("<%d %s\n", ddd++, ss.c_str());
// // 		printf(" s %d\n", strs_coords.size());
// 		const float x = atof(strs_coords[1].c_str());
// 		const float z = atof(strs_coords[2].c_str());
// 		printf("< %f %f\n", x, z);
// 		boost::geometry::model::d2::point_xy<float> vertex(x, z);
// 		boost::geometry::append(poly, vertex);
// 	}
// 
// 
// 	return boost::geometry::distance(poly, point);
// }

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
printf("%s: %d\n", __FILE__, __LINE__);
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("GraspingAgent.InnerModel") ;
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
printf("%s: %d\n", __FILE__, __LINE__);
	bool activated = false;
printf("%s: %d\n", __FILE__, __LINE__);
	if (setParametersAndPossibleActivation(prs, activated))
	{
printf("%s: %d\n", __FILE__, __LINE__);
			if (not activated)
			{
printf("%s: %d\n", __FILE__, __LINE__);
				return activate(p);
			}
	}
	else
	{
printf("%s: %d\n", __FILE__, __LINE__);
		return false;
	}
printf("%s: %d\n", __FILE__, __LINE__);
	return true;
}

bool SpecificWorker::deactivateAgent()
{
printf("%s: %d\n", __FILE__, __LINE__);
		return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
printf("%s: %d\n", __FILE__, __LINE__);
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
printf("%s: %d\n", __FILE__, __LINE__);
	return params;
}

bool SpecificWorker::setAgentParameters(const ParameterMap& prs)
{
printf("%s: %d\n", __FILE__, __LINE__);
	bool activated = false;
printf("%s: %d\n", __FILE__, __LINE__);
	return setParametersAndPossibleActivation(prs, activated);
}

void SpecificWorker::killAgent()
{
printf("%s: %d\n", __FILE__, __LINE__);
}

Ice::Int SpecificWorker::uptimeAgent()
{
printf("%s: %d\n", __FILE__, __LINE__);
	return 0;
}

bool SpecificWorker::reloadConfigAgent()
{
printf("%s: %d\n", __FILE__, __LINE__);
	return true;
}


void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event& modification)
{
printf("%s: %d\n", __FILE__, __LINE__);
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);

	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
printf("%s: %d\n", __FILE__, __LINE__);
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
printf("%s: %d\n", __FILE__, __LINE__);
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
	AGMModelEdge dst;
	AGMModelConverter::fromIceToInternal(modification,dst);
	agmInner.updateImNodeFromEdge(dst, innerModel);
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
printf("%s: %d\n", __FILE__, __LINE__);
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
printf("%s: %d\n", __FILE__, __LINE__);
	params.clear();
printf("%s: %d\n", __FILE__, __LINE__);
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

printf("%s: %d\n", __FILE__, __LINE__);
	try
	{
printf("%s: %d\n", __FILE__, __LINE__);
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
printf("%s: %d\n", __FILE__, __LINE__);
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}
printf("%s: %d\n", __FILE__, __LINE__);

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &newModel, AGMModel::SPtr &worldModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel, "graspingAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

void SpecificWorker::actionExecution()
{
printf("%s: %d\n", __FILE__, __LINE__);
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
printf("%s: %d\n", __FILE__, __LINE__);
	
	try
	{
		const float x = str2float(symbol->getAttribute("tx"));
		const float y = str2float(symbol->getAttribute("ty"));
		const float z = str2float(symbol->getAttribute("tz"));
		QVec worldRef = QVec::vec3(x, y, z);
		QVec robotRef = innerModel->transform("robot", worldRef, "room");
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
	qFatal("ddd");
	try
	{
		int32_t tableId = str2int(params["container"].value);
		QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
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
printf("%s: %d\n", __FILE__, __LINE__);
	static int32_t state = 0;
	static QTime time;
	QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	static int lastTargetId = 0;
	
	if (first) state = 0;
	printf("action_GraspObject: first:%d  state=%d\n", (int)first, state);

	std::map<std::string, AGMModelSymbol::SPtr> symbols;
	try
	{
		symbols = newModel->getSymbolsMap(params, "object", "room", "robot");
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
			inversekinematics_proxy->setFingers(80);
		}
		catch(...)
		{
			qFatal("%s: %d\n", __FILE__, __LINE__);
		}

		printf("%s: %d\n", __FILE__, __LINE__);
		usleep(100000);
		QVec objectsLocationInRobot;

		try
		{
			objectsLocationInRobot = getObjectsLocationInRobot(symbols, symbols["object"]);
		}
		catch (...)
		{
			printf("%s: %d\n", __FILE__, __LINE__);
		}

		objectsLocationInRobot += QVec::vec6(100, 100, 0,  0,0,0);
		objectsLocationInRobot(3) = 0;
		objectsLocationInRobot(4) = -1.5707;
		objectsLocationInRobot(5) = -3.1415926535;
		objectsLocationInRobot.print("d1");

		printf("%s: %d\n", __FILE__, __LINE__);

// 		QVec handTargetPoseInRoom;
// 		try
// 		{
// 			handTargetPoseInRoom = fromRobotToRoom(symbols, objectsLocationInRobot);
// 			handTargetPoseInRoom.print("d2");
// 		}
// 		catch (...)
// 		{
// 			printf("%s: %d\n", __FILE__, __LINE__);
// 		}
// 		printf("%s: %d\n", __FILE__, __LINE__);

		try
		{
			lastTargetId = sendRightArmToPose(objectsLocationInRobot);
			objectsLocationInRobot.print("m");
			time = QTime::currentTime();
			state = 1;
		}
		catch (...)
		{
			printf("%s: %d\n", __FILE__, __LINE__);
		}
		printf("%s: %d\n", __FILE__, __LINE__);
	}
	else if (state == 1) // wait
	{
		TargetState ikState = inversekinematics_proxy->getTargetState("RIGHTARM", lastTargetId);

		if (ikState.finish)
		{
			if (ikState.errorT < 40 and ikState.errorR < 0.5)
				state = 2;
		}
	}
	else if (state == 2) // 2º get the hand into the object
	{
		printf("yipieeeeeeeeee\n");
// 		usleep(100000);
// 		sendRightArmToPose(symbols["object"], QVec::vec3(0, -250, 0));
// 		time = QTime::currentTime();
		state = 3;
		exit(1);
	}
	else if (state == 3) // wait
	{
// 		if (time.elapsed() > 4000) { state = 0; }
// 		else
// 		{
// 			auto distance = innerModel->transform("grabPositionHandR", getObjectsLocationInRobot(symbols["object"]) + offset.operator*(0.4), "room").norm2();
// 			printf("state==3: %f\n", distance);
// 			if (distance < 5)
// 			{
// 				state = 4;
// 			}
// 		}
	}
	else if (state == 4) // 3º get the hand into the object
	{
// 		//inversekinematics_proxy->setFingers(50);
// 		time = QTime::currentTime();
// 		state = 5;
	}
	else if (state == 5) // wait
	{
// 		if (time.elapsed() > 4000) { state = 0; }
// 		else
// 		{
// 			auto distance = innerModel->transform("grabPositionHandR", getObjectsLocationInRobot(symbols["object"]) + offset.operator*(0.4), "room").norm2();
// 			printf("state==5: %f\n", distance);
// 			if (distance < 5)
// 			{
// 				state = 6;
// 			}
// 		}
	}
	else if (state == 6)
	{
// 		try
// 		{
// 			inversekinematics_proxy->setFingers(50);
// 			usleep(500000);
// // 			qFatal("got it!!!! :-D");
// 			newModel->removeEdge(symbols["object"], symbols["table"], "in");
// 			newModel->addEdge(   symbols["object"], symbols["robot"], "in");
// 			{
// 				QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
// 				sendModificationProposal(newModel, worldModel);
// 			}
// 			time = QTime::currentTime();
// 			state = 7;
// 		}
// 		catch(...)
// 		{
// 			printf("graspingAgent: Couldn't publish new model\n");
// 		}
	}
	else
	{
		if (time.elapsed() > 4000)
		{
			state = 0;
		}
	}

}


QVec SpecificWorker::getObjectsLocationInRobot(std::map<std::string, AGMModelSymbol::SPtr> &symbols, AGMModelSymbol::SPtr &object)
{
printf("%s: %d\n", __FILE__, __LINE__);
	// Get target
	int robotID, objectID;
	robotID = symbols["robot"]->identifier;
	objectID = symbols["object"]->identifier;
	
	QString  robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
	QString  objectIMID = QString::fromStdString(worldModel->getSymbol(objectID)->getAttribute("imName"));

	return innerModel->transform6D(robotIMID, objectIMID);
}

QVec SpecificWorker::fromRobotToRoom(std::map<std::string, AGMModelSymbol::SPtr> &symbols, const QVec vector)
{
printf("%s: %d\n", __FILE__, __LINE__);
	// Get target
	int roomID, robotID;
	roomID = symbols["room"]->identifier;
	robotID = symbols["robot"]->identifier;
	
	QString  robotIMID = QString::fromStdString(worldModel->getSymbol(robotID)->getAttribute("imName"));
	QString  roomIMID = QString::fromStdString(worldModel->getSymbol(roomID)->getAttribute("imName"));

	qDebug() << roomIMID << "  " << robotIMID;
	
	innerModel->getTransformationMatrix(roomIMID, robotIMID).print("robot to room");
	
	
	
	innerModel->getTransformationMatrix(roomIMID, robotIMID).extractAnglesR_min().print("angles");
	
	return innerModel->transform6D(roomIMID, vector, robotIMID);
}

int SpecificWorker::sendRightArmToPose(QVec targetPose)
{
printf("%s: %d\n", __FILE__, __LINE__);
	Pose6D target;
	WeightVector weights;
	try
	{
		target.x = targetPose.x();
		target.y = targetPose.y();
		target.z = targetPose.z();
		weights.x = 1;
		weights.y = 1;
		weights.z = 1;
		target.rx = targetPose.rx();
		target.ry = targetPose.ry();
		target.rz = targetPose.rz();
		weights.rx = 1;
		weights.ry = 1;
		weights.rz = 1;
	}
	catch (...)
	{
		printf("graspingAgent: Error reading data from cognitive model: (%s:%d)\n", __FILE__, __LINE__);
	}

	return inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);
}

void SpecificWorker::action_SetObjectReach(bool first)
{
printf("%s: %d\n", __FILE__, __LINE__);
	printf("void SpecificWorker::action_SetObjectReach()\n");

	///
	///  Lift the hand if it's down, to avoid collisions
	///
	if (first or innerModel->transform("room", "grabPositionHandR")(1)<1500)
	{
		backAction = action;
		if (first) printf("first time, set arm for manipulation\n");
		else  printf("arm is down, set arm for manipulation\n");
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
			QMutexLockerDebug locker(mutex_AGM_IM, __FUNCTION__);
			AGMModelSymbol::SPtr goalObject = worldModel->getSymbol(objectId);
			QVec pose = innerModel->transformS("robot", goalObject->getAttribute("imName"));
			const float x = pose.x();
// 			const float y = pose.y();
			const float z = pose.z();
			// In the meantime we just move the head downwards:
			inversekinematics_proxy->setJoint("head_pitch_joint", 0.9, 0.5);
			float angle = atan2(x, z);
			if (angle > +0.3) angle = +0.3;
			if (angle < -0.3) angle = -0.3;
			inversekinematics_proxy->setJoint("head_yaw_joint", angle, 0.5);
// // // // // // // // 			saccadic3D(QVec::vec3(x,y,z), QVec::vec3(0,0,1));
		}
		catch (...)
		{
			printf("%s %d\n", __FILE__, __LINE__);
		}
	}
	else
	{
		printf("don't have the object to reach in my model %d\n", objectId);
	}

	printf("--------------------\n");

	///
	/// No more work to do. The label is set passively (from this agent's point of view)
	///
}


void SpecificWorker::saccadic3D(QVec point, QVec axis)
{
printf("%s: %d\n", __FILE__, __LINE__);
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
printf("%s: %d\n", __FILE__, __LINE__);
// 	printf("saccadic3D\n");

	QVec rel = innerModel->transform("rgbd", QVec::vec3(tx, ty, tz), "room");
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
	RoboCompInverseKinematics::Pose6D targetSight;
	targetSight.x = tx;
	targetSight.y = ty;
	targetSight.z = tz;
	RoboCompInverseKinematics::Axis axSight;
	axSight.x = axx;
	axSight.y = axy;
	axSight.z = axz;
	bool axisConstraint = false;
	float axisAngleConstraint = 0;
	try
	{
		inversekinematics_proxy->stop("HEAD");
		usleep(500000);
		inversekinematics_proxy->pointAxisTowardsTarget("HEAD", targetSight, axSight, axisConstraint, axisAngleConstraint);
	}
	catch(...)
	{
		printf("IK connection error\n");
	}
*/

}




void SpecificWorker::setRightArmUp_Reflex()
{
	inversekinematics_proxy->setJoint("rightShoulder1", -3, 0.3);
	inversekinematics_proxy->setJoint("rightShoulder2", -0.2, 0.3);
	inversekinematics_proxy->setJoint("rightShoulder3", 0.5, 0.3);
	inversekinematics_proxy->setJoint("rightElbow", 0.1, 0.5);
	inversekinematics_proxy->setJoint("rightForeArm", 0.1, 0.3);
	inversekinematics_proxy->setJoint("rightWrist1", 0.0, 0.3);
	inversekinematics_proxy->setJoint("rightWrist2", 0.0, 0.3);
}



