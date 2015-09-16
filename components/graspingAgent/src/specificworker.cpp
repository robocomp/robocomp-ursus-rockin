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

	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();	
#ifdef USE_QTGUI	
	osgView = new OsgView( this );
	show();
//  	printf("%s: %d\n", __FILE__, __LINE__);
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
// 	printf("%s: %d\n", __FILE__, __LINE__);
	manipulator = new osgGA::TrackballManipulator;
	osgView->setCameraManipulator(manipulator, true);
// 	printf("%s: %d\n", __FILE__, __LINE__);
	innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
#endif
	
	

	setRightArmUp_Reflex();
	
	
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
void SpecificWorker::updateViewer()
{
	 printf("%s: %d\n", __FILE__, __LINE__);
	if (!innerModel) return;
	
	innerViewer->update();
	osgView->autoResize();		
	osgView->frame();	


}

void SpecificWorker::compute( )
{
	QMutexLocker locker(mutex);
	{
		if (not innerModel->getNode("grabPositionHandR"))
		{
			printf("waiting for AGM*\n");
			return;
		}
	}
	manageReachedObjects();

	// ACTION EXECUTION
	actionExecution();
#ifdef USE_QTGUI
	updateViewer();
#endif	
}


void SpecificWorker::manageReachedObjects()
{
	float THRESHOLD_object = 150;
	float THRESHOLD_table = 400;
	
	bool changed = false;
	
	QMutexLocker locker(mutex);
	
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
				d2n = distanceToNode("right_shoulder_grasp_pose", newModel, node);
			}
			catch(...)
			{
				printf("Ref: right_shoulder_grasp_pose: %p\n", (void *)innerModel->getNode("right_shoulder_grasp_pose"));
				printf("Obj: %s: %p\n", node->getAttribute("imName").c_str(), (void *)innerModel->getNode(node->getAttribute("imName").c_str()));
			}
			
			QVec graspPosition = innerModel->transform("room", QVec::vec3(0, 0, 0), "right_shoulder_grasp_pose");
			graspPosition(1) = 0;
			QVec obj = innerModel->transformS("room", node->getAttribute("imName"));
			obj(1) = 0;
			graspPosition.print("  graspPosition");
			obj.print("  obj");

			float THRESHOLD = THRESHOLD_object;
			if (isObjectType(newModel, node, "table"))
			{
				THRESHOLD = THRESHOLD_table;
			}
			
			
			printf("%s: %f  (th:%f)\n", node->getAttribute("imName").c_str(), (graspPosition-obj).norm2(), THRESHOLD);


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
	QMutexLocker locker(mutex);

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
	QMutexLocker locker(mutex);
	
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
	QMutexLocker locker(mutex);
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
	QMutexLocker locker(mutex);
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
	QMutexLocker locker(mutex);
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	QMutexLocker locker(mutex);
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
	QMutexLocker locker(mutex);
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

void SpecificWorker::changeInner ()
{	
	if (innerViewer)
	{
		//borra innermodel dentro de InnerModelViewer
		osgView->getRootGroup()->removeChild(innerViewer);				
	}
	innerModel = agmInner.extractInnerModel("room", false);
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
	innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
	
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event& modification)
{
	QMutexLocker locker(mutex);

	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	agmInner.setWorld(worldModel);
	
#ifdef USE_QTGUI
	changeInner( );
#else
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);	
#endif	
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	QMutexLocker locker(mutex);

	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	agmInner.setWorld(worldModel);
#ifdef USE_QTGUI
	changeInner( );
#else
	if (innerModel) delete innerModel;
	innerModel = agmInner.extractInnerModel("room", true);	
#endif	
	
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	QMutexLocker locker(mutex);
	
// // 	printf("---- %d %s %d\n" , modification.a, modification.edgeType.c_str(), modification.b);

	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	
	//worldModel->save("g.xml");
	try {
		agmInner.updateImNodeFromEdge(modification, innerModel);
	}
	catch (...)
	{
		qDebug()<<"\n";
	}
	
	agmInner.setWorld(worldModel);
	
}


bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	QMutexLocker locker(mutex);

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
	QMutexLocker locker(mutex);

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
	QMutexLocker locker(mutex);

	static std::string previousAction = "";
	bool newAction = (previousAction != action);

	qDebug()<<"---------------------------------";
	cout<<action<<endl;
	qDebug()<<"---------------------------------";
	
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
	QMutexLocker locker(mutex);
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
		QMutexLocker locker(mutex);
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
	static int32_t state = 0;
	static QTime time;

	QMutexLocker locker(mutex);
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	QVec objectsLocationInRobot;
	TargetState ikState;
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

//////////////////////////////////////////////
// <TAB>STATE MACHINE<TAB><TAB><TAB><TAB><TAB><TAB><TAB>//
// <TAB>1.- approach the hand and open fingers<TAB>//
// <TAB>2.- align in x axe<TAB><TAB><TAB><TAB><TAB><TAB>//
// <TAB>3.- grasp position<TAB><TAB><TAB><TAB><TAB><TAB>//
// <TAB>4.- close fingers<TAB><TAB><TAB><TAB><TAB><TAB>//
//////////////////////////////////////////////
	switch (state)
	{
		case 0: // APPROACH AND OPEN FINGERS
			// open fingers
			try
			{
				inversekinematics_proxy->setFingers(80); //OPEN FINGERS
				inversekinematics_proxy->setJoint("head_pitch_joint", 1.2, 0.5);
			}
			catch(...) { qFatal("%s: %d\n", __FILE__, __LINE__); }
			usleep(100000);
			// approach the hand
			try
			{
				objectsLocationInRobot = getObjectsLocationInRobot(symbols, symbols["object"]); //POSE OBJECT IN ROBOT SYSTEM
				objectsLocationInRobot.print("objectsLocationInRobot");
				innerModel->transformS("robot", QVec::vec3(0,0,0), symbols["object"]->getAttribute("imName")).print("directo");
			}
			catch (...) { printf("%s: %d\n", __FILE__, __LINE__); }
			// add offset and put rotation
			objectsLocationInRobot += QVec::vec6(150, 0, 0,  0,0,0);
			objectsLocationInRobot(3) = 0;
			objectsLocationInRobot(4) = -1.5707;
			objectsLocationInRobot(5) = 0;
			try
			{
				lastTargetId = sendRightArmToPose(objectsLocationInRobot);
				qDebug()<<"------> 0 step execution";
				time = QTime::currentTime();
				state++; // next state
			}
			catch (...)	{ printf("%s: %d\n", __FILE__, __LINE__); }
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 1: // WAITING FOR THE FIRST VIK EXECUTION
			ikState = inversekinematics_proxy->getTargetState("RIGHTARM", lastTargetId);
			if (ikState.finish)
			{
				if (ikState.errorT < 40 and ikState.errorR < 0.5)
				{
					state++; // next state
					qDebug()<<"------> 1 step execution";
				}
			}
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 2: // ALIGN TO OBJECT IN X AXE
			try
			{
				objectsLocationInRobot = getObjectsLocationInRobot(symbols, symbols["object"]); //POSE OBJECT IN ROBOT SYSTEM
			}
			catch (...)	{	printf("%s: %d\n", __FILE__, __LINE__);	}
			// add offset and put rotation
			objectsLocationInRobot += QVec::vec6(100, -50, 0,  0,0,0);
			objectsLocationInRobot(3) = 0;
			objectsLocationInRobot(4) = -1.5707;
			objectsLocationInRobot(5) = 0;
			try
			{
				lastTargetId = sendRightArmToPose(objectsLocationInRobot);
				qDebug()<<"------> 2 step execution";
				time = QTime::currentTime();
				state++; // next state
			}
			catch (...)	{ printf("%s: %d\n", __FILE__, __LINE__); }
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 3: // WAITING FOR THE SECOND VIK EXECUTION
			ikState = inversekinematics_proxy->getTargetState("RIGHTARM", lastTargetId);
			if (ikState.finish)
			{
				if (ikState.errorT < 40 and ikState.errorR < 0.5)
				{
					state++; // next state
					qDebug()<<"------> 3 step execution";
				}
			}
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 4: // GRASP POSITION
			try
			{
				objectsLocationInRobot = getObjectsLocationInRobot(symbols, symbols["object"]); //POSE OBJECT IN ROBOT SYSTEM
			}
			catch (...)	{	printf("%s: %d\n", __FILE__, __LINE__);	}
			// add offset and put rotation
			objectsLocationInRobot += QVec::vec6(80, -50, 0,  0,0,0);
			objectsLocationInRobot(3) = 0;
			objectsLocationInRobot(4) = -1.5707;
			objectsLocationInRobot(5) = 0;
			try
			{
				lastTargetId = sendRightArmToPose(objectsLocationInRobot);
				qDebug()<<"------> 4 step execution";
				time = QTime::currentTime();
				state++; // next state
			}
			catch (...)	{ printf("%s: %d\n", __FILE__, __LINE__); }
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 5: // WAITING FOR THE THIRD VIK EXECUTION
			ikState = inversekinematics_proxy->getTargetState("RIGHTARM", lastTargetId);
			if (ikState.finish)
			{
				if (ikState.errorT < 40 and ikState.errorR < 0.5)
				{
					state++; // next state
					qDebug()<<"------> 5 step execution";
				}
			}
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		case 6:
			try
			{
				inversekinematics_proxy->setFingers(50);
				usleep(500000);
// 				qFatal("got it!!!! :-D");
				newModel->removeEdge(symbols["object"], symbols["table"], "in");
				newModel->addEdge(   symbols["object"], symbols["robot"], "in");
				{
					QMutexLocker locker(mutex);
					sendModificationProposal(newModel, worldModel);
				}
				time = QTime::currentTime();
				
				state++;
			}
			catch(...)
			{
				printf("graspingAgent: Couldn't publish new model\n");
			}
			break;
		////////////////////////////////////////////////////////////////////////////////////////////
		default:
			if (time.elapsed() > 4000)
			{
				state = 0;
			}
			break;
	}
}

/**
 * \brief This method calculates the pose of the symbol OBJECT into the robot reference system.
 * @param symbols the symbols of AGM model
 * @param object the goal object
 * @return POSE 6D
 */ 
QVec SpecificWorker::getObjectsLocationInRobot(std::map<std::string, AGMModelSymbol::SPtr> &symbols, AGMModelSymbol::SPtr &object)
{
	QMutexLocker locker(mutex);

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
	QMutexLocker locker(mutex);

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
	QMutexLocker locker(mutex);
	printf("void SpecificWorker::action_SetObjectReach()\n");

	///
	///  Lift the hand if it's down, to avoid collisions
	///
	printf("altura mano %f\n", innerModel->transform("room", "grabPositionHandR")(1));
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
			AGMModelSymbol::SPtr goalObject = worldModel->getSymbol(objectId);
			// Get object's relative position from the robot's perspective
			QVec poseRRobot = innerModel->transformS("robot", goalObject->getAttribute("imName"));
			float angleRRobot = atan2(poseRRobot.x(), poseRRobot.z());
			printf("angulo relativa robot %f\n", angleRRobot);
			// Get object's relative position from the yaw's perspective
			QVec poseRYaw = innerModel->transformS("head_yaw_pose", goalObject->getAttribute("imName"));
			poseRYaw.print("relativo al yaw");
			float angleRYaw = atan2(poseRYaw.x(), poseRYaw.z());
			printf("angulo relativa a la camara %f\n", angleRYaw);
			// Compute current head's yaw
			float currentYaw = angleRRobot - angleRYaw;
			printf("current yaw: %f\n", currentYaw);
			float angle = 0.5*angleRRobot + 0.5*currentYaw;
			printf("%f -> ", angle);
			if (fabs(angle-currentYaw) > 10.*M_PI/180.)
			{
				printf(" ** ");
				if (angle>currentYaw)
					angle = currentYaw + 10.*M_PI/180.;
				else
					angle = currentYaw - 10.*M_PI/180.;
			}
			printf(" -> %f\n", angle);
			
			if (angle > +1.) angle = +1.;
			if (angle < -1.) angle = -1.;

			// In the meantime we just move the head downwards:
			inversekinematics_proxy->setJoint("head_pitch_joint", 0.9, 0.5);
			printf("Mandamos angulo %f\n", angle);
			inversekinematics_proxy->setJoint("head_yaw_joint", angle, 0.5);
// // // // // // // 			saccadic3D(QVec::vec3(x,y,z), QVec::vec3(0,0,1));
		}
		catch (...)
		{
			printf("%s %d\n", __FILE__, __LINE__);
			qFatal("d");
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
	saccadic3D(point(0), point(1), point(2), axis(0), axis(1), axis(2));
}

void SpecificWorker::saccadic3D(float tx, float ty, float tz, float axx, float axy, float axz)
{
	QVec rel = innerModel->transform("rgbd", QVec::vec3(tx, ty, tz), "room");
// 	rel.print("desde la camara");

	float errYaw   = -atan2(rel(0), rel(2));
	float errPitch = +atan2(rel(1), rel(2));

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



