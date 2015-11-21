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
    messages_saved_pub_ = nh.advertise<std_msgs::UInt32> ("/roah_rsbb/messages_saved", 1, true);
    // --> If you look this. write in the terminal  ------>> rostopic echo /testPublication

    // subcribe

    subROS = nh.subscribe("/roah_rsbb/goal", 1000, &SpecificWorker::goalCallback, this);
    subROS2 = nh.subscribe ("/roah_rsbb/benchmark/state", 1, &SpecificWorker::benchmark_state_callback, this);

}

void SpecificWorker::benchmark_state_callback(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg)
{
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

    //send the log
    std_msgs::UInt32 messages_saved_msg;
    messages_saved_msg.data = 1;
    messages_saved_pub_.publish (messages_saved_msg);

    //ir like a champion a target
    goto_target( target_obtained );

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


void SpecificWorker::goto_target( RoboCompTrajectoryRobot2D::TargetPose target)
{
    bool goOk = false;
    bool errorGo = false;
    int cont = 0;
    while (!goOk && !errorGo)
    {
            try
            {
                    trajectoryrobot2d_proxy->go(target);
                    printf("Go to ( %f , %f , %f )",target.x,target.y,target.z);
                    goOk = true;
            }
            catch(const Ice::Exception &ex)
            {
                    std::cout <<"ERROR trajectoryrobot2d->go "<< ex << std::endl;
                    sleep(5000);
                    cont++;
                    if (cont == 4)
                    {
                            errorGo = true;
                    }
            }
            if (!errorGo)
            {
                    sleep(5000);
            }
    }

    if (!errorGo)
    {
            bool stateOk = false;
            bool errorState = false;
            int cont = 0;
            string state;

            while (true) // wait to idle state
            {
                    while (!stateOk)
                    {
                            try
                            {
                                    state = trajectoryrobot2d_proxy->getState().state;
                                    stateOk = true;
                            }
                            catch(const Ice::Exception &ex)
                            {
                                    std::cout <<"ERROR trajectoryrobot2d->getState "<< ex << std::endl;
                                    sleep(5000);
                                    cont++;
                                    if (cont == 4)
                                    {
                                            errorState = true;
                                    }
                            }
                    }

                    if (!errorState)
                    {
                            if (state == "IDLE")
                            {
                                    break;
                            }
                    }
                    else
                    {
                            std::cout <<"ERROR in get state!!!"<< std::endl;
                            break;
                    }
            }
    }
}


void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}

//     float64 x;
//     float64 y;
//     float64 theta;
//
//     bool goOk = false;
//     bool errorGo = false;
//     int cont = 0;
//     while (!goOk && !errorGo)
//     {
//             try
//             {
//                     trajectoryrobot2d_proxy->go(target);
//                     printf("Go to ( %f , %f , %f )",target.x,target.y,target.z);
//                     goOk = true;
//             }
//             catch(const Ice::Exception &ex)
//             {
//                     std::cout <<"ERROR trajectoryrobot2d->go "<< ex << std::endl;
//                     this->sleep(5000)
//                     cont++;
//                     if (cont == 4)
//                     {
//                             errorGo = true;
//                     }
//             }
//             if (!errorGo)
//             {
//                     this->sleep(5000)
//             }
//     }
//
//     if (!errorGo)
//     {
//             bool stateOk = false;
//             bool errorState = false;
//             int cont = 0;
//             string state;
//
//             while (true) // wait to idle state
//             {
//                     while (!stateOk)
//                     {
//                             try
//                             {
//                                     state = trajectoryrobot2d_proxy->getState().state;
//                                     stateOk = true;
//                             }
//                             catch(const Ice::Exception &ex)
//                             {
//                                     std::cout <<"ERROR trajectoryrobot2d->getState "<< ex << std::endl;
//                                     this->sleep(5000)
//                                     cont++;
//                                     if (cont == 4)
//                                     {
//                                             errorState = true;
//                                     }
//                             }
//                     }
//
//                     if (!errorState)
//                     {
//                             if (state == "IDLE")
//                             {
//                                     break;
//                             }
//                     }
//                     else
//                     {
//                             std::cout <<"ERROR in get state!!!"<< std::endl;
//                             break;
//                     }
//             }
//     }

}


void SpecificWorker::goalCallback(const ::geometry_msgs::Pose2D msg)
{
        cout<<msg.x<<endl;
        cout<<msg.y<<endl;
        cout<<msg.theta<<endl;

        target_obtained.x = msg.x;
        target_obtained.y = msg.y;
        target_obtained.ry = msg.theta;

// 	ROS_INFO("I heard: [%s]", msg->data.c_str());
// 	///////////// Aqui tiene que venir las n poses
//
// 	// asignar la lista de target de ROS a la de robocomp
// 	// 	target.x = pose2D.x;
// 	// 	target.z = pose2D.y;
// 	// 	target.y = 0; // no cambiar! siempre 0
// 	// 	target.ry = pose2D.teta;
// 	// añadir la modificacion de nueva lista
// 	targetList = poses2D;
}
