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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Empty.h>
#include <roah_rsbb_comm_ros/BenchmarkState.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    
      ros::Subscriber subROS;
      ros::Subscriber subROS2;
	    ros::Publisher  messages_saved_pub_;
		    
        RoboCompTrajectoryRobot2D::TargetPose target_obtained;
        
        geometry_msgs::Pose2D goal_msg;

				ros::NodeHandle nh;
        std_msgs::String str;
        
        QMutex mutex_pos;

	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
        void prepare();
        void execute();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
        void goto_target( RoboCompTrajectoryRobot2D::TargetPose target);
        void benchmark_state_callback(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg);
	void goalCallback(const ::geometry_msgs::Pose2D msg);
	enum State {INIT, GOING, FINISH};
	State state = State::INIT;
	uint veces = 0;
	
public slots:
	void compute();

      private:
      void nextTarget();
};

#endif

