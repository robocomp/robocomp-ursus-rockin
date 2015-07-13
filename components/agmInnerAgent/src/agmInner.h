/*
 * 
 *  *    Copyright (C) 2015 by YOUR NAME HERE
 *  *
 *  *    This file is part of RoboComp
 *  *
 *  *    RoboComp is free software: you can redistribute it and/or modify
 *  *    it under the terms of the GNU General Public License as published by
 *  *    the Free Software Foundation, either version 3 of the License, or
 *  *    (at your option) any later version.
 *  *
 *  *    RoboComp is distributed in the hope that it will be useful,
 *  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  *    GNU General Public License for more details.
 *  *
 *  *    You should have received a copy of the GNU General Public License
 *  *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
 

#ifndef AGMINNER_H
#define AGMINNER_H

#include <innermodel/innermodel.h>
#include <agm.h>
#include <qt4/QtCore/QHash>
#include <qt4/QtCore/QList>
#include <qt4/QtCore/QString>
#include <iostream>
#include <string>

using namespace std;


class AgmInner
{
public:
	AgmInner();
	~AgmInner();
	//const AGMModel::SPtr &src
	void setWorld(AGMModel::SPtr model);
	AGMModel::SPtr getWorld();
	int findName(QString n);	
	InnerModel* extractInnerModel(QString imNodeName);
	void recorrer(InnerModel* imNew, int& symbolID);
	void edgeToInnerModel(AGMModelEdge edge, InnerModel* imNew);
	void checkLoop(int& symbolID, QList< int >& visited, string linkType, bool& loop);


private:
    AGMModel::SPtr worldModel;
    InnerModel *innerModel;
};

#endif // AGMINNER_H
