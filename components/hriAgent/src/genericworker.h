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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>
#include <ui_guiDlg.h>
#include "config.h"
#include <agm.h>
#include <Speech.h>
#include <AGMAgent.h>
#include <AGMCommonBehavior.h>
#include <AGMExecutive.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompSpeech;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAGMAgent;
struct BehaviorNavegacionParameters 
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
	QMutex *mutex;                //Shared mutex with servant

		
	bool activate(const BehaviorNavegacionParameters& parameters);
	bool deactivate();
	bool isActive() { return active; }
	RoboCompAGMWorldModel::BehaviorResultType status();
	SpeechPrx speech_proxy;
	AGMAgentTopicPrx agmagenttopic;
	virtual bool activateAgent(const ParameterMap& prs) = 0;
	virtual bool deactivateAgent() = 0;
	virtual StateStruct getAgentState() = 0;
	virtual ParameterMap getAgentParameters() = 0;
	virtual bool setAgentParameters(const ParameterMap& prs) = 0;
	virtual void  killAgent() = 0;
	virtual Ice::Int uptimeAgent() = 0;
	virtual bool reloadConfigAgent() = 0;
	virtual void  modelModified(const RoboCompAGMWorldModel::Event& modification) = 0;
	virtual void  modelUpdated(const RoboCompAGMWorldModel::Node& modification) = 0;

protected:
	QTimer timer;
	int Period;
	int iter;
	bool active;
	AGMModel::SPtr worldModel;
	ParameterMap params;
	BehaviorNavegacionParameters p;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	RoboCompPlanning::Action createAction(std::string s);
public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif