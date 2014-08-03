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
#include <boost/graph/graph_concepts.hpp>

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
	void goLiving2();
	void goLiving3();
	void goDoor();
	void goEntrance();
	void stopRobot();
	void setNewTargetFromPlant(QVec);
	void setTargetCoorFromPlant(QVec t);
	
private:
	PlantWidget *plantWidget;
	void go(const QVec &t);
	QTime reloj;
	
};

#endif