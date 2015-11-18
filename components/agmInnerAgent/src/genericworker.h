/*
 *    Copyright (C) 
<@@<2015>@@>
 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <ui_mainUI.h>

#include <CommonBehavior.h>
#include <AGMAgent.h>
#include <Planning.h>
#include <AGMExecutive.h>
#include <AGMCommonBehavior.h>
#include <AGMWorldModel.h>

#include <agm.h>
#include <agmInner/agmInner.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompAGMAgent;
using namespace RoboCompPlanning;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMWorldModel;


struct BehaviorParameters 
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};



class GenericWorker : 
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
<TABHERE>bool activate(const BehaviorParameters& parameters);
<TABHERE>bool deactivate();
<TABHERE>bool isActive() { return active; }
<TABHERE>RoboCompAGMWorldModel::BehaviorResultType status();
	

<TABHERE>AGMAgentTopicPrx agmagenttopic_proxy;

