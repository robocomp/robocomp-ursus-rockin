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

#define THRESHOLD 500.

void SpecificWorker::manageReachedObjects()
{
	printf("<<<<<<<<<<<<<<<< REACHED OBJECTS\n");
	printf("<<<<<<<<<<<<<<<< REACHED OBJECTS\n");
	printf("<<<<<<<<<<<<<<<< REACHED OBJECTS\n");

	bool changed = false;
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	for (auto node : *newModel)
	{
		if (node->symboltype() == "object")
		{
			// Avoid working with rooms
			bool isRoom = false;
			for (auto edge : *node)
			{
				if (edge->getLabel() == "room")
				{
					isRoom = true;
				}
			}
			if (isRoom)
			{
				continue;
			}
			/// Get coordinates
			const float x = str2float(node->getAttribute("x"));
			const float y = str2float(node->getAttribute("y"));
			const float z = str2float(node->getAttribute("z"));
			/// Compute distance and new state
			const float distance = innerModel->transform("base_head", QVec::vec3(x,y,z), "world").norm2();
			printf("object %d: %f\n", node->identifier, distance);
			for (auto edge : *node)
			{
				if (edge->getLabel() == "reach" and distance > THRESHOLD)
				{
					edge->setLabel("noReach");
					printf("object %d STOPS REACH\n", node->identifier);
					changed = true;
				}
				else if (edge->getLabel() == "noReach" and distance < THRESHOLD)
				{
					edge->setLabel("reach");
					printf("object %d STARTS REACH\n", node->identifier);
					changed = true;
				}
			}
		}
	}
	
	/// Publish new model if changed
	if (changed)
	{
		sendModificationProposal(worldModel, newModel);
	}


	printf(">>>>>>>>>>>>>>>> REACHED OBJECTS\n");
	printf(">>>>>>>>>>>>>>>> REACHED OBJECTS\n");
	printf(">>>>>>>>>>>>>>>> REACHED OBJECTS\n");




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
	const float rx = str2float(robot->getAttribute("x"));
	const float rz = str2float(robot->getAttribute("z"));
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
				sendModificationProposal(worldModel, newModel);
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

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
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

void SpecificWorker::action_FindObjectVisuallyInTable(bool first)
{
	int32_t tableId = str2int(params["container"].value);
	try
	{
		AGMModelSymbol::SPtr goalTable = worldModel->getSymbol(tableId);
		const float x = str2float(goalTable->getAttribute("x"));
		const float z = str2float(goalTable->getAttribute("z"));
		QVec worldRef = QVec::vec3(x, 800, z);
		QVec robotRef = innerModel->transform("robot", worldRef, "world");
		printf("saccadic3D\n");
		printf("\n");
		saccadic3D(robotRef, QVec::vec3(0,0,1));
	}
	catch(...)
	{
	}
}


// void SpecificWorker::action_RobotMovesObjectFromContainer()
// {
// 	AGMModel::SPtr newModel(new AGMModel(worldModel));
// 	try
// 	{
// 		auto symbols = newModel->getSymbolsMap(params, "object", "c1", "c2");
// 		newModel->removeEdge(symbols["object"], symbols["c1"], "in");
// 		newModel->addEdge(   symbols["object"], symbols["c2"], "in");
// 		try
// 		{
// 			Pose6D target;
// 			WeightVector weights;
// 			try
// 			{
// 				target.x = str2float(symbols["object"]->getAttribute("tx"));
// 				target.y = str2float(symbols["object"]->getAttribute("ty"));
// 				target.z = str2float(symbols["object"]->getAttribute("tz"));
// 				target.rx = str2float(symbols["object"]->getAttribute("rx"));
// 				target.ry = str2float(symbols["object"]->getAttribute("ry"));
// 				target.rz = str2float(symbols["object"]->getAttribute("rz"));
// 				weights.x = 1;
// 				weights.y = 1;
// 				weights.z = 1;
// 				weights.rx = 1;
// 				weights.ry = 0;
// 				weights.rz = 1;
// 			}
// 			catch (...)
// 			{
// 				printf("graspingAgent: Error reading data from cognitive model: (%s:%d)\n", __FILE__, __LINE__);
// 			}
// 			try
// 			{
// 				bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 1.);
// 			}
// 			catch (...)
// 			{
// 				printf("graspingAgent: Couldn't set RIGHTARM target (maybe a communication problem?)\n");
// 			}
// 			sendModificationProposal(worldModel, newModel);
// 		}
// 		catch(...)
// 		{
// 			printf("graspingAgent: Couldn't publish new model\n");
// 		}
// 	}
// 	catch(...)
// 	{
// 		printf("graspingAgent: Couldn't retrieve action's parameters\n");
// 	}
// }


void SpecificWorker::action_GraspObject(bool first)
{
	static int32_t state = 0;
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	try
	{
		auto symbols = newModel->getSymbolsMap(params, "object", "table");
		newModel->removeEdge(symbols["object"], symbols["table"], "in");
		newModel->addEdge(   symbols["object"], symbols["table"], "in");
		
		
		if (first) state = 0;
		
		if (state == 0)
		{
			
			try
			{
				Pose6D target;
				WeightVector weights;
				try
				{
					target.x = str2float(symbols["object"]->getAttribute("tx"));
					target.y = str2float(symbols["object"]->getAttribute("ty"));
					target.z = str2float(symbols["object"]->getAttribute("tz"));
					target.rx = str2float(symbols["object"]->getAttribute("rx"));
					target.ry = str2float(symbols["object"]->getAttribute("ry"));
					target.rz = str2float(symbols["object"]->getAttribute("rz"));
					weights.x = 1;
					weights.y = 1;
					weights.z = 1;
					weights.rx = 1;
					weights.ry = 0;
					weights.rz = 1;
				}
				catch (...)
				{
					printf("graspingAgent: Error reading data from cognitive model: (%s:%d)\n", __FILE__, __LINE__);
				}
				try
				{
					bodyinversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights, 1.);
				}
				catch (...)
				{
					printf("graspingAgent: Couldn't set RIGHTARM target (maybe a communication problem?)\n");
				}
				sendModificationProposal(worldModel, newModel);
			}
			catch(...)
			{
				printf("graspingAgent: Couldn't publish new model\n");
			}
		}
		
	}
	catch(...)
	{
		printf("graspingAgent: Couldn't retrieve action's parameters\n");
	}
}


void SpecificWorker::action_SetObjectReach(bool first)
{
	printf("void SpecificWorker::action_SetObjectReach()\n");

	///
	///  Lift the hand if it's down, to avoid collisions
	///
	if (backAction != "setobjectreach" or innerModel->transform("root", "finger_right_1_1_tip")(1)<1000)
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
		AGMModelSymbol::SPtr goalObject = worldModel->getSymbol(objectId);
		const float x = str2float(goalObject->getAttribute("x"));
		const float y = str2float(goalObject->getAttribute("y"));
		const float z = str2float(goalObject->getAttribute("z"));
		saccadic3D(QVec::vec3(x,y,z), QVec::vec3(0,0,1));
	}
	else
	{
		printf ("don't have the object to reach in my model %d\n", objectId);
	}

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
	printf("saccadic3D\n");
	
	QVec rel = innerModel->transform("rgbd", QVec::vec3(tx, ty, tz), "world");
	rel.print("desde la camara");

	float errYaw   = -atan2(rel(0), rel(2));
	float errPitch = +atan2(rel(1), rel(2));
	printf("%f  %f\n", errYaw, errPitch);

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
		const float x     = str2float(robot->getAttribute("x"));
		const float z     = str2float(robot->getAttribute("z"));
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


