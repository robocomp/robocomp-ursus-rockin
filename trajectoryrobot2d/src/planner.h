/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef PLANNER_H
#define PLANNER_H

#include <QtCore>
#include <qmat/QMatAll>
#include "tree.hh"
#include <innermodel/innermodel.h>
#include <InnerModelManager.h>
#include "rcisdraw.h"
#include "qline2d.h"
#include "waypoints.h"

using namespace RoboCompInnerModelManager;

class Planner : public QObject
{
Q_OBJECT
	public:
		Planner(InnerModel *innerModel_, QObject *parent=0);
		~Planner(){};
  
		void setMaxIter( int v) 						{ MAX_ITER = v;}
		bool computePath(const QVec &target);
		WayPoints smoothRoad( WayPoints road);
		void drawTree( InnerModelManagerPrx innermodelmanager_proxy );
		// 	 void drawPath( qWorld *world, const QList<QVec> & path , const QColor & color);
		// 	 void drawPath(qWorld *world);
		// 	 void drawSmoothedPath(qWorld *world);

		WayPoints getPath() { WayPoints road; road.readRoadFromList(currentSmoothedPath); return road;}; 
		//QVec tryBezierToTarget(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec>  *arbol , tree<QVec>::iterator & nodeCurrentPos);
	 
  private:
		InnerModel *innerModel;
		tree<QVec> *arbol, *arbolGoal, *aux;
		tree<QVec>::pre_order_iterator CURRENT_LEAF, CURRENT_LEAF_GOAL;
		QVec fsX, fsZ;
		int ind;
		QVec finalTarget;
		bool  PATH_FOUND;
		QVec p1,p2,origin,target;
		int MAX_ITER;
		QList<QVec> currentPath, currentSmoothedPath, currentAdaptiveSmoothedPath;
	    QVec trySegmentToTarget(const QVec & origin, const QVec & target, bool & reachEnd, tree<QVec>  *arbol, tree<QVec>::iterator & nodeCurrentPos);
		QList<QVec> recoverPath(tree<QVec> *arbol , const tree<QVec>::pre_order_iterator & _current, tree<QVec> *arbolGoal, const tree<QVec>::pre_order_iterator & _currentGoal);
		bool isThereAnObstacleAtPosition(const QVec & pCenter, float rX, float rZ);

		QVec chooseRandomPointInFreeSpace(const QVec &currentTarget);
		void computeRandomSequence(const QVec & currentTarget);
		bool equal(const QVec & p1, const QVec & p2);
		void smoothPath( const QList< QVec >& list);
		void smoothPathStochastic( QList<QVec> & list);
		tree<QVec>::iterator findClosestPointInTree( tree<QVec> *arb , const QVec & currentTarget);
		void getCollisionObjects(InnerModelNode* node); 										//Obtains a list of id's of planes and meshes for collision detection
		bool collisionDetector( const QVec &point,  InnerModel *innerModel);	
		QStringList listCollisionObjects;
		
  public:
		
};

#endif // PLANNER_H











