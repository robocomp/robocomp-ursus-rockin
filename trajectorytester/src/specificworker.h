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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "plantwidget.h"
#include <innermodel/innermodel.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QWidget *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const tagsList& tags); 

	enum class State {IDLE, INIT_GO_KITCHEN, GO_KITCHEN, SERVOING, MOVE_ARM, INIT_MOVE_ARM, GRASP, INIT_PREPARE_ARM, PREPARE_ARM, INIT_APPROACH, APPROACH,
										DETACH_TO_GET, INIT_REDRAW_ARM, REDRAW_ARM, INIT_BACKUP, BACKUP, 
										INIT_GO_OTHER_TABLE, GO_OTHER_TABLE, DETACH_TO_PUT, PUT_MUG_ON_TABLE, INIT_REDRAW_ARM2, REDRAW_ARM2, BACKUP2, INIT_GO_CENTER, GO_CENTER} ;
	
public slots:
 	void compute(); 	
	void goButton();
	void goBedRoom();
	void goLiving();
	void goKitchen();
	void goKitchen2();
	void goHall();
	void goDining();
	void goDining2();
	void goDining3();
	void goLiving2();
	void goLiving3();
	void goDoor();
	void goEntrance();
	void stopRobot();
	void setNewTargetFromPlant(QVec);
	void setTargetCoorFromPlant(QVec t);
	void goHome();
	void step1();
	void step2();
	void step3();
	void step4();
	void step5();
	
	
private:
	PlantWidget *plantWidget;
	void go(const QVec& t, const QVec& r=QVec());
	void goBackwards(const QVec& t, const QVec &r=QVec());
	QTime reloj, relojE;
	QVec target, current;
	void doStateMachine();
	State init_go_kitchen();
	State go_kitchen();
	State servoing();
	State moveArm();
	State initMoveArm();
	State grasp();
	State detachToGet();
	State initRedrawArm();
	State initPrepareArm();
	State prepareArm();
	State redrawArm();
	State initApproach();
	State approach();
	State initBackUp();
	State backUp();
	State initGoOtherTable();
	State goOtherTable();
	State putMugOntable();
	State detachToPut();
	State initRedrawArm2();
	State redrawArm2();
	State backUp2();
	State initGoCenter();
	State goCenter();
	State state;
	
	void attachMug();
	void openFingers();
	void closeFingers();
	void graspFingers();
	
	RoboCompTrajectoryRobot2D::NavState planningState;
	RoboCompDifferentialRobot::TBaseState bState;
	bool tag11; 	//mano	
	bool tag12;		//marca mug mesa 1
	bool tag0; 		//marca mesa humano
	QVec tag12Pose, tag11Pose, tag0Pose;
	InnerModel *innerModel;
	QStringList listaMotores;
	void actualizarInnermodel(const QStringList &listaJoints);
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
	void drawAxis(const QString &name, const QString &parent);
	void removeAxis(const QString &name);
	float initialDistance; //to grab the mug
	RoboCompBodyInverseKinematics::TargetState bikState;
};

#endif