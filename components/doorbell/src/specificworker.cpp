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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
        doorbells = 0;
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();


	messages_saved_pub_ = nh.advertise<std_msgs::UInt32>("/roah_rsbb/messages_saved", 1, true);

        ben_state_sub = nh.subscribe ("/roah_rsbb/benchmark/state", 1, &SpecificWorker::benchmark_state_callback, this);
	bell_sub = nh.subscribe ("/roah_rsbb/devices/bell", 1, &SpecificWorker::bell_callback, this);
}

void SpecificWorker::bell_callback(std_msgs::Empty::ConstPtr const&)
{
	qDebug() << "bell_callback";
	doorbellRang();
	doorbells++;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::benchmark_state_callback(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg)
{
    qDebug() << "benchmark_state_callback";
    switch (msg->benchmark_state) 
    {
        case roah_rsbb_comm_ros::BenchmarkState::STOP:
          //pararse
          //trajectoryrobot2d_proxy->stop();
          break;
        case roah_rsbb_comm_ros::BenchmarkState::PREPARE:
           this->prepare();
          break;
        case roah_rsbb_comm_ros::BenchmarkState::EXECUTE:
           this->execute();
        break;
    }
}

void SpecificWorker::prepare()
{
    cout<<"prepare"<<endl;
    if (ros::service::waitForService ("/roah_rsbb/end_prepare", 100)) 
    {
        std_srvs::Empty s;
        if (! ros::service::call ("/roah_rsbb/end_prepare", s)) 
        {
          ROS_ERROR ("Error calling service /roah_rsbb/end_prepare");
        }
    }
    else 
    {
        ROS_ERROR ("Could not find service /roah_rsbb/end_prepare");
    }
    
}


void SpecificWorker::execute()
{
    cout << "execute" << endl;
    //first set the log
    //log set
		
		qDebug() << __FUNCTION__;
		
		//send the log
    std_msgs::UInt32 messages_saved_msg;
    messages_saved_msg.data = 1;
    messages_saved_pub_.publish (messages_saved_msg);


    
//    doorbell_sub = nh.subscribe ("/roah_rsbb/devices/bell", 1, &SpecificWorker::doorbellCallBack, this);
}

void SpecificWorker::end_execute()
{
    cout << "end execute"<<endl;
        //fin
    if (ros::service::waitForService ("/roah_rsbb/end_execute", 100)) 
    {
        std_srvs::Empty s;
        if (! ros::service::call ("/roah_rsbb/end_execute", s)) 
        {
          ROS_ERROR ("Error calling service /roah_rsbb/end_execute");
        }
    }
    else 
    {
        ROS_ERROR ("Could not find service /roah_rsbb/end_execute");
    }
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	static string lastAction = "";

	if (action != lastAction)
	{
		printf("action changed to %s!\n", action.c_str());
		lastAction = action;
	}


	ros::spinOnce();

	QMutexLocker l(mutex);

	AGMModel::SPtr worldModel(new AGMModel(worldModel));

	AGMModelSymbol::SPtr outside, robot;
	try
	{
		outside = worldModel->getSymbol(17);
	}
	catch(...)
	{
		printf("esperando modelo habit\n");
		return;
	}
	try
	{
		robot   = worldModel->getSymbol(1);
	}
	catch(...)
	{
		printf("esperando modelo robot\n");
		return;
	}




	if (action == "changeroom")
	{
		AGMModelSymbol::SPtr goalRoom = worldModel->getSymbol(str2int(params["r2"].value));
		auto roomId = goalRoom->identifier;

		QVec target;
		if (roomId == 7) //Mesa
		{
			target = QVec::vec3(0,0,0);
		}
		else if (roomId == 17) //Puerta
		{
			target = QVec::vec3(-4400, -3200, -1.57);
			//target = QVec::vec3(0, 1000, 0);
		}
		else if (roomId == 37) //Dormitorio
		{
			target = QVec::vec3(-4800, 2000, 1.57);
			//target = QVec::vec3(0, 0, 0);
		}


		target.print("iriamos a");

		try
		{
			TargetPose pose;
			pose.x=target[0]; pose.z=target[1]; pose.ry=target[2];
			pose.y=0;pose.rz=0; pose.rx=0; 
			float dist = 0;
			try
			{
				dist = trajectoryrobot2d_proxy->go(pose);
			}
			catch(...)
			{
				printf("can\'t sent target\n");
			}
			sleep(1);
			std::string estado;
			try
			{
				estado = trajectoryrobot2d_proxy->getState().state;
			}
			catch(...)
			{
				printf("can\'t get state\n");
			}
			if(estado == "IDLE")
			{
				printf("distance %f\n", dist);
		 		trajectoryrobot2d_proxy->stop();
				try {worldModel->addEdgeByIdentifiers(1,  7, "in"); } catch(...){}
				try {worldModel->addEdgeByIdentifiers(1, 17, "in"); } catch(...){}
				try {worldModel->addEdgeByIdentifiers(1, 37, "in"); } catch(...){}
				worldModel->addEdgeByIdentifiers(1, roomId, "in");
				try {sendModificationProposal(worldModel, worldModel);  } catch(...){printf("can\'t publish\n");}
			}		
		}
		catch(Ice::Exception &e){ std::cout << e.what() << std::endl;}
		catch(...){ std::cout << "some other exception (not ice)" << std::endl;}

	}
	else if (action == "recognizehumanasextranger" or
	         action == "recognizeHumanAsDeliMan" or
	         action == "recognizeHumanAsPostman" or
	         action == "recognizeHumanAsDrKimble")
	{
		doRecognition();
	}

}


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
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	agmInner.setWorld(worldModel);
	delete innerModel;
	innerModel = agmInner.extractInnerModel();
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
		AGMMisc::publishModification(worldModel, agmagenttopic_proxy, worldModel,"doorbellAgent");
	}
	catch(...)
	{
		exit(1);
	}
}

//void SpecificWorker::doorbellCallBack(std_msgs::Empty)
//{
 //   doorbellRang();
  //  doorbells++;
//}

void SpecificWorker::doorbellRang()
{
	QMutexLocker l(mutex);


	printf("Trying to include a human in the model, given that the door bell rang...\n");

	AGMModelSymbol::SPtr newSymbolPerson =  worldModel->newSymbol("person");
	
	
	AGMModelSymbol::SPtr typeSymbolPerson = worldModel->newSymbol("unknownPerson");
	
	worldModel->addEdgeByIdentifiers(1, newSymbolPerson->identifier, "know");
	worldModel->addEdgeByIdentifiers(newSymbolPerson->identifier, 17, "in");
	worldModel->addEdgeByIdentifiers(newSymbolPerson->identifier, typeSymbolPerson->identifier,"personIs");

	sendModificationProposal(worldModel, worldModel);
}


void SpecificWorker::removeCurrentPerson()
{
    
        if (doorbells > 4)
            end_execute();    
}

string SpecificWorker::getPersonType()
{
// 	string symbolPersonType_str = "postman";
// 	string symbolPersonType_str = "deliMan";
// 	string symbolPersonType_str = "extranger";
//	string symbolPersonType_str = "medicineDoctor";


	FaceClass fc;
	Colour colour;
	welcomevisitor_proxy->getColourAndClass(colour, fc);
	if(colour == NOTHING and fc.fid!=-1)
	{
		if (fc.fclass == "medico")
			return "medicineDoctor";
		else
			return "extranger";
	}
	else if (colour== WHITE)
		return "deliMan";
	else if (colour == YELLOW)
		return "postman";
	else
		return "unknown";
}


void SpecificWorker::doRecognition()
{
	AGMModelSymbol::SPtr outside, symbolPerson, symbolPersonType;

	for (AGMModelSymbol::iterator edge_itr=outside->edgesBegin(worldModel); edge_itr!=outside->edgesEnd(worldModel); edge_itr++)
	{
		if ((*edge_itr)->getLabel() == "in")
		{
			int second = (*edge_itr)->getSymbolPair().first;
			const AGMModelSymbol::SPtr &symbolPerson = worldModel->getSymbolByIdentifier(second);
			if (symbolPerson->symbolType == "person")
			{
				for (AGMModelSymbol::iterator edge_itr2=symbolPerson->edgesBegin(worldModel); edge_itr2!=symbolPerson->edgesEnd(worldModel); edge_itr2++)
				{
					if ((*edge_itr2)->getLabel() == "personIs")
					{
						int third = (*edge_itr2)->getSymbolPair().second;
						const AGMModelSymbol::SPtr &symbolPersonType = worldModel->getSymbolByIdentifier(third);
						if (symbolPersonType->symbolType == "unknownPerson")
						{
							printf("got unknown!!!\n");
							string symbolPersonType_str = getPersonType();
							symbolPersonType->symbolType = symbolPersonType_str;
							sendModificationProposal(worldModel, worldModel);
						}
					}
				}
			}
		}
	}

}


