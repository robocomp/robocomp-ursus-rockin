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
#include </home/pbustos/robocomp/components/robocomp-ursus-rockin/trajectoryrobot2d/src/specificworker.h>
#include <TrajectoryRobot2D.ice>
#include <boost/graph/graph_concepts.hpp>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx,QWidget *parent) : GenericWorker(mprx)
{
//	connect(goPushButton, SIGNAL(clicked()), this, SLOT(goButton()));
	connect(bedroomPushButton, SIGNAL(clicked()), this, SLOT(goBedRoom()));
	connect(kitchenPushButton, SIGNAL(clicked()), this, SLOT(goKitchen()));
	connect(kitchenPushButton_2, SIGNAL(clicked()), this, SLOT(goKitchen2()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(goKitchen()));
	connect(hallPushButton, SIGNAL(clicked()), this, SLOT(goHall()));
	connect(livingPushButton, SIGNAL(clicked()), this, SLOT(goLiving()));
	connect(entrancePushButton, SIGNAL(clicked()), this, SLOT(goEntrance()));
	connect(livingPushButton_2, SIGNAL(clicked()), this, SLOT(goLiving2()));
	connect(livingPushButton_3, SIGNAL(clicked()), this, SLOT(goLiving3()));
	connect(doorPushButton, SIGNAL(clicked()), this, SLOT(goDoor()));
	connect(diningPushButton, SIGNAL(clicked()), this, SLOT(goDining()));
	connect(diningPushButton_2, SIGNAL(clicked()), this, SLOT(goDining2()));
	connect(diningPushButton_3, SIGNAL(clicked()), this, SLOT(goDining3()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(stopRobot()));
	
	connect(t1Button, SIGNAL(clicked()), this, SLOT(step1()));
	connect(t2Button, SIGNAL(clicked()), this, SLOT(step2()));
	connect(t3Button, SIGNAL(clicked()), this, SLOT(step3()));
	
	plantWidget = new PlantWidget(frame, QPointF(82,457), QPointF(0,10000), QPointF(65,387), QPointF(0,-10000));
	plantWidget->show();
	statusLabel->setText("");
	tag11 = false;
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockinSimple.xml");
	
	connect(plantWidget, SIGNAL(mouseMove(QVec)), this, SLOT(setTargetCoorFromPlant(QVec)));
	connect(plantWidget, SIGNAL(mousePress(QVec)), this, SLOT(setNewTargetFromPlant(QVec)));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	state = State::IDLE;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(250);
	return true;
};

void SpecificWorker::compute( )
{
	try
	{
		planningState = trajectoryrobot2d_proxy->getState();
		statusLabel->setText(QString::fromStdString( planningState.state ));
		if( planningState.state == "PLANNING" )
		{
			segsLcd->display(reloj.elapsed() );
		}
		if( planningState.state == "EXECUTING" )
		{
			float distance = (target - current).norm2();
			//estimatedDurationLcd->display((int)state.estimatedTime);
			estimatedDurationLcd->display(distance);
			executionTimeLcd->display(distance / 300);			
		}
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << "Error talking to TrajectoryRobot2D" <<  std::endl;
	}
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		current = QVec::vec3(bState.x, 0, bState.z);
		plantWidget->moveRobot( bState.x, bState.z, bState.alpha);	
		innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);
		innerModel->updateRotationValues("robot", 0, bState.alpha, 0);
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << "Error talking to Differentialrobot" << std::endl;
	}

	
	doStateMachine();
	
}

void SpecificWorker::doStateMachine()
{
	switch (state) 
	{
		case State::IDLE:
				qDebug() << "IDLE";
			break;
		case State::GO_KITCHEN:
				qDebug() << "GO_KITCHEN";
				state = go_kitchen();
			break;
		case State::SERVOING:
			qDebug() << "SERVOING";
			state = servoing();
			break;
		default:
			break;
	}
}

SpecificWorker::State SpecificWorker::go_kitchen()
{
	static bool initiated = false;
	
	if( planningState.state == "IDLE" and not initiated)
	{
		try 
		{	
			RoboCompBodyInverseKinematics::Pose6D target;
			target.x = 0; target.y=10; target.z = 1000; target.rx=0; target.ry=0; target.rz=0;
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = 0; axis.y = 0; axis.z = 1;
			bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, false, false);
		} 
		catch (const RoboCompBodyInverseKinematics::BIKException &ex) 
			{ std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
	
		go(QVec::vec3(5500,0,-5000), QVec::vec3(0,0,0));
		initiated = true;
		return State::GO_KITCHEN;
	}
	if( planningState.state == "IDLE" and (QVec::vec3(5500,0,-5000) - QVec::vec3(bState.x,0,bState.z)).norm2() < 100)
	{
		qDebug() << "Made it...";
		initiated = false;
		stopRobot();
		return State::SERVOING;
	}
	if( tag11 == true) 
	{
		qDebug() << "TAG11";
		initiated = false;
		stopRobot();
		return State::SERVOING;
	}
	if( planningState.state == "EXECUTING" )
	{
			qDebug() << "Working...";
			return State::GO_KITCHEN;
	}
}

SpecificWorker::State SpecificWorker::servoing()
{
	qDebug() << tag.id << tag.tx << tag.ty << tag.tz;
	//compute new final target 
	
	go(QVec::vec3(,0,), QVec::vec3(0,0,0));
	return State::IDLE;
}

/////////////////////////////////////////////////////////////
///  TASK DEVELOPMENT
////////////////////////////////////////////////////////////

void SpecificWorker::step1()
{
	state = State::GO_KITCHEN;
	
// 	try 
// 	{
// 		bodyinversekinematics_proxy->setRobot(0);	
// 	} catch (const Ice::Exception &ex) { std::cout << ex << "in setRobot" << std::endl;}
// 	
// 	try 
// 	{
// 		RoboCompBodyInverseKinematics::Pose6D target;
// 		target.x = 0; target.y=10; target.z = 1000; target.rx=0; target.ry=0; target.rz=0;
// 		RoboCompBodyInverseKinematics::Axis axis;
// 		axis.x = 0; axis.y = 0; axis.z = 1;
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget("HEAD", target, axis, false, false);
// 		
// 	} catch (const RoboCompBodyInverseKinematics::BIKException &ex) { std::cout << ex.text << "in pointAxisTowardsTarget" << std::endl;}
// 	
// 	goKitchen2();
}

/**
 * @brief Reposition de robot servoing the mark
 * 
 * @return void
 */

void SpecificWorker::step2()
{
	
	
}

void SpecificWorker::step3()
{

}


////////////////////////////////////////////////////////////////////

void SpecificWorker::go(const QVec& t, const QVec &r)
{
	RoboCompTrajectoryRobot2D::TargetPose tp;
	tp.x = t.x();
	tp.z = t.z();
	tp.y = 0;
	if( r.size() == 3 )
	{
		tp.rx = r.x(); tp.ry = r.y(); tp.rz = r.z();
		tp.onlyRot = true;
	}
	else
		tp.onlyRot = false;
	target = t;
	try
	{
		trajectoryrobot2d_proxy->go(tp);
		//qDebug() << __FUNCTION__ << "Target " << t << " sent";
		reloj.restart();
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}	
}

void SpecificWorker::goButton()
{
	go(QVec::vec3(xSpinBox->value(),0, zSpinBox->value()));
}

void SpecificWorker::goDining()
{
	go(QVec::vec3(6000,0,-9100));
}

void SpecificWorker::goDining2()
{
	go(QVec::vec3(4600,0,-7800));
}

void SpecificWorker::goDining3()
{
	go(QVec::vec3(6200,0,-7100));
}

void SpecificWorker::goLiving()
{
	go(QVec::vec3(3000,0,-8100));
}

void SpecificWorker::goLiving2()
{
	go(QVec::vec3(666,0,-7861));
}

void SpecificWorker::goLiving3()
{
	go(QVec::vec3(790,0,-5062));
}

void SpecificWorker::goEntrance()
{
	go(QVec::vec3(2700,0,-4000));
}

void SpecificWorker::goDoor()
{
	go(QVec::vec3(4500,0,-3300));
}

void SpecificWorker::goKitchen()  
{
	go(QVec::vec3(6000,0,-5900), QVec::vec3(0,0,0));	
}

void SpecificWorker::goKitchen2()//Stove table
{
	go(QVec::vec3(5500,0,-5000), QVec::vec3(0,0,0));
}

void SpecificWorker::goBedRoom()
{
	go(QVec::vec3(7000,0,-1500));

}

void SpecificWorker::goHall()
{
	go(QVec::vec3(800,0,-1000));
}

void SpecificWorker::setTargetCoorFromPlant(QVec t)
{
	xSpinBox->display(t.x());
	zSpinBox->display(t.z());
}


void SpecificWorker::setNewTargetFromPlant(QVec t)
{
	xLcd->display(t.x());
	yLcd->display(t.z());
	go(t);
}

void SpecificWorker::stopRobot()
{
	try
	{	trajectoryrobot2d_proxy->stop();	}
	catch(const Ice::Exception &ex)
	{		std::cout << ex << "Error talking to TrajectoryRobot2D" << std::endl;	}
}


/////////////////////////////////////////////////////////////////
//// AprilTags subscription
/////////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	for(auto i: tags)
	{
		if( i.id == 11) 
		{
			tag1LineEdit->setText("Id: " + QString::number(i.id));	
			//qDebug() << __FUNCTION__ << "God damn got it!";
			tag11 = true;
			tag = i;
		}
		else
		{
			tag1LineEdit->setText("");
			tag11 = false;
		}
	}
}



