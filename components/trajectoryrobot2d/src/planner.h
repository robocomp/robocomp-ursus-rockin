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

#include "qline2d.h"
#include "waypoints.h"

#define MAX_WAYPOINT_CACHE 500
#define MAX_RANDOM_R2_CACHE 10000
#define MAX_ITER_INIT 5000



class Planner : public QObject
{
Q_OBJECT
public:
	Planner(const InnerModel &innerModel_, QObject *parent=0);
	~Planner(){}

	void setMaxIter( int v) { MAX_ITER = v; }
	bool computePath(const QVec &target, InnerModel *inner);
	WayPoints smoothRoad( WayPoints road);
	void drawTree(InnerModel *innerVisual);
	QList<QVec> getPath() { return currentSmoothedPath; }

	bool collisionDetector(const QVec &position, const QVec &rotation, InnerModel *im);
	void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
	
private:
	InnerModel *innerModel;
	std::vector<QString> robotNodes;
	std::vector<QString> restNodes;
	tree<QVec> *arbol, *arbolGoal, *aux;
	tree<QVec>::pre_order_iterator CURRENT_LEAF, CURRENT_LEAF_GOAL;
	QVec fsX, fsZ;
	int ind;
	int MAX_ITER;
	QVec finalTarget;
	bool  PATH_FOUND;
	QVec p1,p2,origin,target;
	QList<QVec> currentSmoothedPath;
	
	//Caches
	QList<QVec> wayPointCache;									// To store succesful plans
	void initializeWayPointsCache();
	QVector<QVec> randomR2Cache;
	void initializeRandomR2Cache();
	QVec getPointFromRandomR2Cache();
	
	QVec trySegmentToTarget(const QVec & origin, const QVec & target, bool & reachEnd, tree<QVec>  *arbol, tree<QVec>::iterator & nodeCurrentPos);
	QVec trySegmentToTargetBinarySearch(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec> * arbol , tree<QVec>::iterator & nodeCurrentPos);
	QList<QVec> recoverPath(tree<QVec> *arbol , const tree<QVec>::pre_order_iterator & _current, tree<QVec> *arbolGoal, const tree<QVec>::pre_order_iterator & _currentGoal);
	bool isThereAnObstacleAtPosition(const QVec & pCenter, float rX, float rZ);

	QVec sampleFreeSpaceR2(const QVec &currentTarget, InnerModel *inner);
	bool equal(const QVec & p1, const QVec & p2);
	void smoothPath( const QList< QVec >& list);
	void smoothPathStochastic( QList<QVec> & list);
	tree<QVec>::iterator findClosestPointInTree( tree<QVec> *arb , const QVec & currentTarget);
	void storePlanInCache(QList< QVec > currentPath);

};

#endif // PLANNER_H











