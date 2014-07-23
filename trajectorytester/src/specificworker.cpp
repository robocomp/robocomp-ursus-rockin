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
#include <boost/graph/graph_concepts.hpp>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx,QWidget *parent) : GenericWorker(mprx)
{
	connect(goPushButton, SIGNAL(clicked()), this, SLOT(go()));
	connect(bedroomPushButton, SIGNAL(clicked()), this, SLOT(goBedRoom()));
	connect(kitchenPushButton, SIGNAL(clicked()), this, SLOT(goKitchen()));
	connect(hallPushButton, SIGNAL(clicked()), this, SLOT(goHall()));
	connect(diningPushButton, SIGNAL(clicked()), this, SLOT(goDining()));
	connect(livingPushButton, SIGNAL(clicked()), this, SLOT(goLiving()));
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};

void SpecificWorker::go()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = xSpinBox->value();
	tp.z = zSpinBox->value();
	tp.y = 0;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}


void SpecificWorker::goDining()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.y=0;
	tp.x = 6000;
	tp.z = -9100;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		
}

void SpecificWorker::goLiving()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.y=0;
	tp.x = 3000;
	tp.z = -8100;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		
}

void SpecificWorker::goKitchen()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.y=0;
	tp.x = 6000;
	tp.z = -6100;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		
}

void SpecificWorker::goBedRoom()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.y=0;
	tp.x = 7000;
	tp.z = -1500;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		
}

void SpecificWorker::goHall()
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.y=0;
	tp.x = 800;
	tp.z = -1000;
	
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		qDebug() << __FUNCTION__ << "Target sent";
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}		
}
