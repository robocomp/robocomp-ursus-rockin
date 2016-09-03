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
#include <iostream>
#include <ostream>
#include <vector>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/dynamic_property_map.hpp> 
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <nabo/nabo.h>
#include <innermodeldraw.h>
#include "sampler.h"
#include <qlog/qlog.h>

#include "plannerompl.h"

struct VertexPayload
{	
	QVec pose; //3D
	std::size_t vertex_id;
	VertexPayload() {}
	VertexPayload(std::size_t i, const QVec &p) { vertex_id = i; pose=p; }
};	
struct EdgePayload
{
	float dist;
	EdgePayload(){ dist = -1;}
	EdgePayload(float d) { dist = d;}
};
typedef boost::adjacency_list<boost::listS,boost::listS, boost::undirectedS, VertexPayload, EdgePayload, boost::listS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
//typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef boost::component_index<VertexIndex> Components;
typedef std::vector<Graph::edge_descriptor> PathType;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;
typedef std::pair<EdgeIterator, EdgeIterator> EdgePair;
typedef std::pair<int,std::vector<Vertex> > CComponent; //Size of comp and correspondng list of vertices
typedef std::map<Vertex, int32_t> ComponentMap; 
typedef std::vector<CComponent> ConnectedComponents;


class PlannerPRM : public QObject
{
	public:
		PlannerPRM();
		void initialize(Sampler *sampler_, int nPointsInGraph, int nNeighboursInGraph);
		bool computePath(QVec& target,  InnerModel* inner);
		void initialize(const Sampler &sampler);
		QList<QVec> getPath() { return currentPath; }
		void setSpaceLimits(float xmin, float xmax, float zmin, float zmax) {xMin = xmin; xMax = xmax, zMin = zmin; zMax = zMax;}
		bool drawGraph(InnerModelViewer *innerViewer);
		void cleanGraph(InnerModelViewer *innerViewer);
		void removeGraph(InnerModelViewer *innerViewer);
			
		// Learning
		bool learnPath(const QList<QVec> &path);
		bool learnForAWhile(uint maxGraphNodes = 150, bool print = true, bool save = true);
		
		//Sampler
		Sampler *sampler;
		
	private:
		InnerModel innerPlanner;
		
		Graph graph;
		int32_t constructGraph(const QList<QVec> &pointList, uint NEIGHBOORS=20, float MAX_DISTANTE_TO_CHECK=2000.f, uint robotSize=400);
		bool searchGraph(const Vertex& originVertex, const Vertex& targetVertex,  std::vector<Vertex> &vertexPath);
		bool rebuildExternalData();
		void readGraphFromFile(QString name);
		void writeGraphToFile(const QString& fileName = "");
		void searchClosestPoints(const QVec& origin, const QVec& target, Vertex& originVertex, Vertex& targetVertex);
		void connectedComponents( ComponentMap &componentMap, ConnectedComponents &comps, bool print = false) const;
		
		//smoothers
		bool pathSmoother(QList<QVec> & pointlist);
		void smoothPath( const QList<QVec> & list);
		void smoothPathIter( QList<QVec> & list);
		QList<QVec> currentSmoothedPath;
		QList<QVec> currentPath;   			//Results will be saved here
		
		//learners
		int32_t removeSmallComponents(int32_t minSize = 3);
		bool connectIsolatedComponents(int32_t& numConnections);
		int32_t connectCloseElements(float thoseCloserThan = 500);  //mm
	
		float xMin, xMax, zMin, zMax; 		//Limits of environmnent QUITAR
		
		//Libnabo fast KDTree for low dimension
		Nabo::NNSearchF *nabo;
		//Hash mapping nodes index in MatrixXf and vertex_descriptor in the graph
		QHash<int,Vertex> vertexMap;
		//Eigen matrix holding the initial points: size(2,NUM_POINTS)
		Eigen::MatrixXf data;
		//number of points given in constructor
		uint NUM_POINTS;
		uint NEIGHBOORS;
		
		//embedded RRTConnect Planner
		PlannerOMPL plannerRRT;
		bool planWithRRT(const QVec& origin, const QVec& target, QList<QVec> &path);
		
		bool graphDirtyBit;
		uint graphNumPoints;
		QString graphFileName = "grafo.dot";
		uint graphNeighPoints;
};

//Graph writing classes
template <class IndexMap,class PoseMap> class vertex_writer 
{
	public:
		vertex_writer(IndexMap i, PoseMap p) : indexMap(i),poseMap(p) {}
		template <class VertexPayload> void operator()(std::ostream &out, const VertexPayload& vertex) const 
		{
 			out << "[Index=\"" << indexMap[vertex] << "\", Pose=\"" << poseMap[vertex] << "\"]";		
		}
	private:
		IndexMap indexMap;
		PoseMap poseMap;
};

template <class IndexMap, class PoseMap> inline vertex_writer<IndexMap,PoseMap> make_vertex_writer(IndexMap i,PoseMap p) 
{  
	return vertex_writer<IndexMap,PoseMap>(i,p);
}

template <class DistanceMap> class edge_writer 
{
	public:
		edge_writer(DistanceMap d) : distanceMap(d) {}
		template <class EdgePayload> void operator()(std::ostream &out, const EdgePayload& edge) const 
		{
 			out << "[Distance=\"" << distanceMap[edge] <<  "\"]";		
		}
	private:
		DistanceMap distanceMap;
};

template <class DistanceMap> inline edge_writer<DistanceMap> make_edge_writer(DistanceMap d) 
{  
	return edge_writer<DistanceMap>(d);
}
#endif // PLANNERPRM_H
// struct Point3D
// {
// 	float x;
// 	float y;
// 	float z;
// 	Point3D(){};
// 	Point3D(const Point3D &p) { x = p.x; y = p.y; z=p.z;};
// 	Point3D(float i, float j, float k) { x = i; y = j; z = k;};
// 	operator QVec() const { return QVec::vec3(x,y,z);}
// 	Point3D operator =(const QVec &v) const { return Point3D(v.x(),v.y(),v.z()); };		
// 	inline friend std::istream& operator >> ( std::istream& i, Point3D& p )
//     {
//         i >> p.x;
// 		if((i.flags() & std::ios_base::skipws) == 0) 
// 		{
// 			char whitespace;
// 			i >> whitespace;
// 		}
// 		i >> p.y;
// 		if((i.flags() & std::ios_base::skipws) == 0)
// 		{
// 			char whitespace;
// 			i >> whitespace;
// 		}
// 		i >> p.z; 	
//         return i;
//     }
// 
//     inline friend std::ostream& operator << ( std::ostream& o, const Point3D& p )
//     {
//         o << p.x << " " << p.y << " " << p.z;
//         return o;
//     } 
// };