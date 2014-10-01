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

	enum class State {IDLE, GO_KITCHEN, SERVOING, MOVE_ARM, INIT_MOVE_ARM, GRASP, CLOSE_FINGERS, OPEN_FINGERS, DETACH, INIT_REDRAW_ARM, REDRAW_ARM, INIT_BACKUP, BACKUP, 
										INIT_GO_OTHER_TABLE, GO_OTHER_TABLE} ;
	
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
	QTime reloj, relojE;
	QVec target, current;
	void doStateMachine();
	State go_kitchen();
	State servoing();
	State moveArm();
	State initMoveArm();
	State openFingers();
	State closeFingers();
	State grasp();
	State detach();
	State initRedrawArm();
	State redrawArm();
	State initBackUp();
	State backUp();
	State initGoOtherTable();
	State goOtherTable();
	State state;
	
	void attachMug();
	RoboCompTrajectoryRobot2D::NavState planningState;
	RoboCompDifferentialRobot::TBaseState bState;
	bool tag11, tag12;
	QVec tag12Pose, tag11Pose;
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