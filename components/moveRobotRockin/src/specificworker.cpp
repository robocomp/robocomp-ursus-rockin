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
    //published
    messages_saved_pub_ = nh.advertise<std_msgs::UInt32>
    ("/roah_rsbb/messages_saved", 1, true);
	  
    // --> If you look this. write in the terminal  ------>> rostopic echo /testPublication

    // subcribe

    subROS = nh.subscribe("/roah_rsbb/goal", 1000, &SpecificWorker::goalCallback, this);
    subROS2 = nh.subscribe ("/roah_rsbb/benchmark/state", 1, &SpecificWorker::benchmark_state_callback, this);

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
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
    //first set the log
    //log set
		
		qDebug() << __FUNCTION__;
		
		//send the log
    std_msgs::UInt32 messages_saved_msg;
    messages_saved_msg.data = 1;
    messages_saved_pub_.publish (messages_saved_msg);
		
    //ir like a champion a target
		//DENTRANCE
		
// 		target_obtained.z =  -3.2*1000.;
// 		target_obtained.x =  -4.7*1000;
// 		target_obtained.ry = 0.0;
		
		/////////////
		goto_target(target_obtained);
		
		qDebug() << __FUNCTION__ << "target called";
}

void SpecificWorker::goto_target( RoboCompTrajectoryRobot2D::TargetPose target)
{
  	try
		{
			std::string st = trajectoryrobot2d_proxy->getState().state;
			cout << st;
			if( st == "IDLE")
			{		 
				trajectoryrobot2d_proxy->go(target);
				qDebug() << "Target sent to Trajectory" << target.x<<target.z<<target.ry;
				state = State::GOING;
			}
    }
    catch(const Ice::Exception &ex)
    {
       std::cout <<"ERROR trajectoryrobot2d->go "<< ex << std::endl;
		}
}


void SpecificWorker::compute()
{
	switch(state)
	{
		case State::INIT:
			break;
		case State::GOING:
			//qDebug() << "en camino al éxito";
			try
			{
				std::string st = trajectoryrobot2d_proxy->getState().state;
				if( st == "IDLE")
				{
					qDebug() << "He acabado una";
					//speech_proxy->say("I have reached waypoint" + itoa(veces));
					if(veces == 3 )
						state = State::FINISH;
					else
					{
						veces++;
						RoboCompTrajectoryRobot2D::NavState statePos;
						::geometry_msgs::Pose2D poseROS;
						
						statePos = trajectoryrobot2d_proxy->getState();
						poseROS.x = statePos.z / 1000.;
						poseROS.y = - (statePos.x / 1000.);
						poseROS.theta = - (statePos.ang);
						
						qDebug() <<"Llegué a ( "<<statePos.z<<statePos.x<<statePos.ang<<" )";
						
						 //signal to ROS after target is reached
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
		
		//////////////////////////7
					}
				}
			}
			catch(const Ice::Exception &e)
			{
				std::cout << "Error reading from Trajectory" << e << std::endl;
			}
			break;
		case State::FINISH:
				qDebug() << "He terminado con éxito";
				//speech_proxy->say("I have finished my assignment");
			break;
	}

	ros::spinOnce();
}

///ROS Callbacks

void SpecificWorker::goalCallback(const ::geometry_msgs::Pose2D msg)
{
	 qDebug() << __FUNCTION__;
   cout<<"target_obtained ROS( "<<msg.x
																<<msg.y
																<<msg.theta
																<<" )"<<endl;
 
   target_obtained.z = msg.x*1000;
   target_obtained.x = -msg.y*1000;
   target_obtained.ry = -msg.theta;
	 target_obtained.y = target_obtained.rx = target_obtained.rz = 0.0;
	 
}

void SpecificWorker::benchmark_state_callback(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg)
{
    cout<<"benchmark_state_callback"<<endl;
    switch (msg->benchmark_state) 
    {
        case roah_rsbb_comm_ros::BenchmarkState::STOP:
          //pararse
          trajectoryrobot2d_proxy->stop();
          break;
        case roah_rsbb_comm_ros::BenchmarkState::PREPARE:
           this->prepare();
          break;
        case roah_rsbb_comm_ros::BenchmarkState::EXECUTE:
           this->execute();
        break;       
    }
}
