/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef PLANNERPRM_H
#define PLANNERPRM_H

#include <QObject>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <iostream>
#include <deque>
#include <iterator>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>
#include "nabo/nabo.h"

struct Payload
		{	
			QVec pose; 
			uint32_t index;
			Payload(){};
			Payload(uint i, const QVec &point){ index = i; pose = point; };
		};		
typedef boost::adjacency_list<boost::vecS,boost::vecS, boost::undirectedS, Payload, boost::no_property, boost::listS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > IndexIteratorMap;

class PlannerPRM : public QObject
{
    Q_OBJECT
    
	public:
		PlannerPRM(){};
		PlannerPRM(const InnerModel &innerModel_, QObject *parent=0);
		bool computePath(const QVec &target, InnerModel *inner);
		void setInnerModel(const InnerModel &innerModel_);
		QList<QVec> getPath() { return currentPath; }
		void setSpaceLimits(float xmin, float xmax, float zmin, float zmax)		{xMin = xmin; xMax = xmax, zMin = zmin; zMax = zMax;};
		
	private:
		Graph graph;
		void addPointToGraph(const QVec &point);
		void createGraph(InnerModel *inner);
		QVec sampleFreeSpaceR2(const QPointF& XZLimits);
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
		InnerModel *innerModel;
		QList<QVec> currentPath;   			//Results will be saved here
		std::vector<QString> robotNodes;
		std::vector<QString> restNodes;
		float xMin, xMax, zMin, zMax; 		//Limits of environmnent
		QVec sampleFreeSpaceR2(InnerModel *inner, const QPointF &XZLimits);
		bool collisionDetector(const QVec &position, const QVec &rotation, InnerModel *im);
		QVec trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd);
		void readGraphFromFile(QString name);
		//Libnabo fast KDTree for low dimension
		Nabo::NNSearchF *nabo;

};

#endif // PLANNERPRM_H
