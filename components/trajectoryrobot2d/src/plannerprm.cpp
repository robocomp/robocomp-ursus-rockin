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

#include "plannerprm.h"

/**
 * @brief Probabilistic Road Map planner constructor. Computes and updates a free-path graph and trajectories on it. If no path can be found on graph, it calls RRT
 * 
 * @param innerModel_ ...
 * @param nPoints ...
 * @param neigh ...
 * @param parent ...
 */
PlannerPRM::PlannerPRM(InnerModel *innerModel_, uint nPoints, uint neigh,  QObject *parent)
{	
	// 	innerModel = new InnerModel(*innerModel_);
	innerModel = innerModel_;
	
	//Get outerRegion extension from floor definition
	QRectF outerRegion;
	InnerModelPlane *floor = NULL;
	try
	{
		floor = innerModel->getPlane("floor_plane");  ///TIENE QUE HABER UN FLOOR_PLANE
	}
	catch (QString err)
	{
		printf("We need a plane named 'floor_plane'\n");
		throw err;
	}

	if (floor != NULL)
	{
		QVec center = innerModel->transform("world", QVec::zeros(3), "floor");
		QVec upperLeft = innerModel->transform("world",QVec::vec3(center.x() - floor->width/2, center.y(), center.z() + floor->height/2), "floor");
		QVec downRight = innerModel->transform("world",QVec::vec3(center.x() + floor->width/2, center.y(), center.z() - floor->height/2), "floor");
		upperLeft.print("UL");
		downRight.print("DR");
		outerRegion.setLeft( upperLeft.x() );
		outerRegion.setRight( downRight.x() );
		outerRegion.setBottom( downRight.z() );
		outerRegion.setTop( upperLeft.z() );
		
		qDebug() << __FUNCTION__ << "OuterRegion" << outerRegion;
	}
	else
		qFatal("Aborting. Cannot determine the size of the world. Please define a floor_plane");
	
	QList<QRectF> innerRegions;
	//QRectF outerRegion(-1920,3500,  4000,-7000);
	//QRectF outerRegion(-2500,-2500,  5000, 5000);
	
	// for Rocking apartment
	// innerRegions << QRectF(1500, 0, 4000, -3000) <<	QRectF(0, -8500, 4000, -1500) << QRectF(7500, -4000, 2500, -6000);
	// QRectF outerRegion(0, 0, 10000, -10000);

	sampler.initialize(innerModel, outerRegion, innerRegions);

	if( QFile("grafo.dot").exists())
	{
		qDebug() << __FUNCTION__ << "Graph file exits. Loading";
		readGraphFromFile("grafo.dot");
	}
	else
	{
		qDebug() << __FUNCTION__ << "Graph file DOES NOT exit. Creating with " << nPoints << "nodes and " << neigh << "neighboors";
		//createGraph(nPoints, neigh, 2500.f);  //MAX distance apart for two points to be considered.
		QList<QVec> pointList = sampler.sampleFreeSpaceR2(nPoints);
		constructGraph(pointList, neigh, 2500.f, 400);
		std::ofstream fout("grafo.dot");
		writeGraphToStream(fout);
	}
	graphDirtyBit = true;
}

/**
 * @brief Main callable method. Computes a path for the robot given a target pose and a point to Innermodel. inner is used to synchronize the current copy of Innermodel.
 * This is done so the planner can ran as a thread in the future.
 * Should de extended to return the best path found given a certain time limit
 * @param target QVec for pose translation. Should be within world limits
 * @param inner Pointer to Innermodel
 * @return true if a path of at least two waypoints have been found. Path is written to class variable currentPath.
 * Additional chekcs on max and min lenght for the path should be applied
 */
bool PlannerPRM::computePath(QVec& target, InnerModel* inner)
{
// 	qDebug() << __FUNCTION__ << "Starting planning with robot at:" << inner->transform("world","robot") <<  "and target at:" << target;

	QVec currentTarget = target;  //local variable to get samples from free space

	//We need to resynchronize here with calling version of IM because in subsequent call the robot will be "Dios sabe dónde"
	QVec robot = inner->transform("world", "robot");
	QVec robotRotation = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min();
	innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());

	currentPath.clear();

	//If target on obstacle find a point close to it on the robot-target line
	if( sampler.searchRobotValidStateCloseToTarget(target) == false )
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
		return false;
	}
	//Check if the target is in "plain sight"
	QVec point;
	//qDebug() << __LINE__;
	if ( sampler.checkRobotValidDirectionToTargetOneShot( robot, target) )
	{
 		qDebug() << __FILE__ << __FUNCTION__ << "-------- Target on sight. Proceeding";
		currentPath << robot << target;
		return true;
	}
	
	//Now search in KD-tree for closest points to origin and target
	Vertex robotVertex, targetVertex;
	searchClosestPoints(robot, target, robotVertex, targetVertex);

	//Obtain a free path from [robot] to [robotVertex] using RRTConnect. Return if fail.
 	QList<QVec> path;
// 	if (planWithRRT(robot, graph[robotVertex].pose, path) )
//  	{
//  		if(path.size() > 1)  //has to be. We trim the last element to avoid duplicating it
//  		{
//  			path.removeLast();
//  			currentPath += path;
//  			qDebug() << __FUNCTION__ << "RRTConnect succeeded for ROBOT with a " << currentPath.size() << "plan." << " So far" << path;
//  		}
//  		else
// 	if (path.size() == 1)
//  				qFatal("Fary en path");
//  	}
//  	else
//  		 return false;

	//Now we are in the graph
	//Search in graph minimun path. Return if fail
	if( robotVertex != targetVertex )  //Same node for both. We should skip searchGraph
	{
		std::vector<Vertex> vertexPath;
		if ( searchGraph(robotVertex, targetVertex, vertexPath) )
			for( Vertex v : vertexPath )
				currentPath.append(graph[v].pose);
		else //No path found. The path does not connect to target's closest point in graph
		{
// 			qDebug() << __FUNCTION__ << "No path through graph. Starting RRTConnect from " << graph[robotVertex].pose << " to" <<  target;
		 if ( planWithRRT(graph[robotVertex].pose, target, path) )
			{
				path.removeFirst();
				currentPath += path;
			}
			else
				return false;
		}
	}
	else	//add the only node
		currentPath += graph[robotVertex].pose;

	//Obtain a free path from [target] to [targetVertex] using RRTConnect. Return if fail.
//	path.clear();
// 	if (planWithRRT(graph[targetVertex].pose, target, path) )
// 	{
// 		if( path.size() > 1) //Should be !!  We trimm the first elemen to avoid duplicating it since it already came in searchGraph
// 		{
// 			path.removeFirst();
// 			currentPath += path;
// // 			qDebug() << __FUNCTION__ << "RRTConnect succeeded for TARGET with a " << path.size() << "plan" << ". So end" << path;
// 		}
// 		else
// 			if(path.size() == 1)
// 				qFatal("Fary en path target");
// 	}
// 	else
// 		 return false;

	if( currentPath.size() < 2 )
		return false;
	else
	{
		currentSmoothedPath.clear();
// 		qDebug() << __FUNCTION__ << "Smoothing";
		smoothPath(currentPath);
		currentPath = currentSmoothedPath;
// 		qDebug() << __FUNCTION__ << "Final path size " << currentPath.size();

		return true;
	}

}

///PRIVATE
////////////////////////////////////////////////////////////////////////
/// One query planner to be used when no path found un PRM
////////////////////////////////////////////////////////////////////////


bool PlannerPRM::planWithRRT(const QVec &origin, const QVec &target, QList<QVec> &path)
{
// 	qDebug() << __FUNCTION__ << "RRTConnect start...";

	//bool reachEnd;
	const float diffV = (origin-target).norm2();
	if (diffV < 200) // HALF ROBOT RADIOUS
	{
		printf("plannerprm.cpp:%d Origin and target too close. %f returning true", __LINE__, diffV);
		return true;
	}

	QVec point;
	qDebug() << __LINE__;
	if (sampler.checkRobotValidDirectionToTargetOneShot( origin, target ))
	{
		printf("plannerprm.cpp:%d Found target directly in line of sight\n", __LINE__);
		fflush(stdout);
		path << origin << target;
		return true;
	}

/*
	printf("%s Calling Full Power of RRTConnect OMPL planner. This may take a while\n", __FUNCTION__);
	fflush(stdout);
	try
	{
		plannerRRT.initialize(&sampler);  //QUITAR DE AQUI
		if (plannerRRT.computePath(origin, target, 60))
		{
			path += plannerRRT.getPath();
			return true;
		}
	}
	catch (...)
	{
	}
*/
	printf("%s: %d\n", __FILE__, __LINE__);
	return false;
}

void PlannerPRM::searchClosestPoints(const QVec& origin, const QVec& target, Vertex& originVertex, Vertex& targetVertex)
{
// 	qDebug() << __FUNCTION__ << "Searching from " << origin << "to " << target;

	//prepare the query
	Eigen::MatrixXi indices;
	Eigen::MatrixXf distsTo;
	Eigen::MatrixXf query(3,2);
	indices.resize(1, query.cols());
	distsTo.resize(1, query.cols());
	query(0,0) = origin.x();query(1,0) = origin.y();query(2,0) = origin.z();
	query(0,1) = target.x();query(1,1) = target.y();query(2,1) = target.z();

	nabo->knn(query, indices, distsTo, 1);

	originVertex = vertexMap.value(indices(0,0));
	targetVertex = vertexMap.value(indices(0,1));

// 	qDebug() << __FUNCTION__ << "Closest point to origin is at" << data(0,indices(0,0)) << data(1,indices(0,0)) << data(2,indices(0,0)) << " and corresponds to " << graph[originVertex].pose;
// 	qDebug() << __FUNCTION__ << "Closest point to target is at" << data(0,indices(0,1)) << data(1,indices(0,1)) << data(2,indices(0,1)) << " and corresponds to " << graph[targetVertex].pose;

}

/**
 * @brief Search the graph for a minimun path between originVertex and targetVertex. Returns the path
 *
 * @param originVertex ...
 * @param targetVertex ...
 * @param vertexPath std::vector of Vertex with the path
 * @return bool
 */
bool PlannerPRM::searchGraph(const Vertex &originVertex, const Vertex &targetVertex, std::vector<Vertex> &vertexPath)
{
	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<float> distances(boost::num_vertices(graph));    // To store distances

	//Create a vertex_index property map, since VertexList is listS
	typedef std::map<Vertex, size_t>IndexMap;
	IndexMap indexMap;
	boost::associative_property_map<IndexMap> propmapIndex(indexMap);
    //indexing the vertices
	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
		boost::put(propmapIndex, v, i++);

	auto predecessorMap = boost::make_iterator_property_map(&predecessors[0], propmapIndex);
	auto distanceMap = boost::make_iterator_property_map(&distances[0], propmapIndex);

	boost::dijkstra_shortest_paths(graph, originVertex, boost::weight_map(boost::get(&EdgePayload::dist, graph))
															.vertex_index_map(propmapIndex)
															.predecessor_map(predecessorMap)
 															.distance_map(distanceMap));

	// Extract a shortest path
	PathType path;
	Vertex v = targetVertex;

	// Start by setting 'u' to the destintaion node's predecessor   |||// Keep tracking the path until we get to the source
	for( Vertex u = predecessorMap[v]; u != v; v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
	{
		std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph);
		Graph::edge_descriptor edge = edgePair.first;
		path.push_back( edge );
	}

 	std::cout << __FUNCTION__ << " Path found with length: " << path.size() <<std::endl;
	Vertex lastVertex;
	if(path.size() > 0)
	{
		// Save shortest path
		vertexPath.clear();
		std::cout << __FUNCTION__ << "Shortest path from origin to target:" << std::endl;
		for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
		{
			vertexPath.push_back(boost::source(*pathIterator, graph));
			lastVertex = boost::target(*pathIterator, graph);
		}
		vertexPath.push_back(lastVertex);
		std::cout << std::endl;
		std::cout << __FUNCTION__ << "Distance: " << distanceMap[targetVertex] << std::endl;
		return true;
	}
	else
		return false;
}



ConnectedComponents PlannerPRM::connectedComponents(ComponentMap &componentMap, bool print)
{
	//qDebug() << __FUNCTION__;

	//create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
 	typedef std::map<Vertex, size_t> IMap;
 	typedef boost::associative_property_map<IMap> IndexMap;
	IMap indexMap;
	IndexMap index( indexMap );

 	//indexing the vertices
 	int i=0;
 	BGL_FORALL_VERTICES(v, graph, Graph)
 		boost::put(index, v, i++);

	std::vector<int> components(boost::num_vertices(graph));
	boost::iterator_property_map< std::vector<int>::iterator, IndexMap> compIterMap( components.begin(), index );

    int numComps = boost::connected_components(graph, compIterMap, vertex_index_map(index));

	//std::cout << __FUNCTION__ << " Number of connected components " << numComps << std::endl;

	//Create a nice data structure to return info
	ConnectedComponents compList(numComps);
	for(auto it : compList)
		it = std::make_pair(0, std::vector<Vertex>());

	componentMap.clear();
	BGL_FORALL_VERTICES(v, graph, Graph)
	{
		//qDebug() << graph[v].index << "is in comp" << get(compIterMap, v);
		compList[get(compIterMap, v)].second.push_back( v );
		compList[get(compIterMap, v)].first++;
		componentMap.insert( std::pair<Vertex,int32_t>( v, get(compIterMap, v )));
	}

	if( print )
		for(auto it : compList)
		{
			qDebug() << __FUNCTION__ << "Num comps: " << it.first ;
			for(auto itt : it.second)
				std::cout <<  graph[itt].vertex_id << " " ;
			std::cout << std::endl;
		}

	return compList;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Elementary learning operations
/////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Given a list of free space points, tries to connect them to graph. The number of vertices changes
 *
 * @return void
 */
int32_t PlannerPRM::constructGraph(const QList<QVec> &pointList, uint NEIGHBOORS, float MAX_DISTANTE_TO_CHECK, uint robotSize)
{

// 	qDebug() << __FUNCTION__ << "Constructing graph with " << pointList.size() << " new points and " <<  NEIGHBOORS << "neighboors";

	int ROBOT_SIZE_SQR = robotSize*robotSize; //mm  OBTAIN FROM ROBOT'S BOUNDING BOX!!!!!
	float MAX_DISTANTE_TO_CHECK_SQR = MAX_DISTANTE_TO_CHECK * MAX_DISTANTE_TO_CHECK;

	//Expand the matrix with new points, insert new vertices and update de <int,vertex> map
	int lastCol = data.cols();
	data.conservativeResize(3, data.cols() + pointList.size() );

	//create the query matrix
	Eigen::MatrixXf query(data.rows(), pointList.size());
	//qDebug() << __FUNCTION__ << "Data matrix resized" << data.rows() << data.cols() << "lastCol" << lastCol;

	for(int i=lastCol, k=0; i< data.cols(); i++, k++)
	{
		data(0,i) = pointList[k].x();
		data(1,i) = pointList[k].y();
		data(2,i) = pointList[k].z();
		query(0,k) = pointList[k].x();
		query(1,k) = pointList[k].y();
		query(2,k) = pointList[k].z();
		Vertex vertex = boost::add_vertex(graph);
		vertexMap.insert(i,vertex);   			//Hash<row,Vertex> to go back and forth between data matrix and graph
		graph[vertex].pose = QVec::vec3(data(0,i), data(1,i), data(2,i));
		graph[vertex].vertex_id = i;
		//qDebug() << __FUNCTION__ << "Inserted vertex " << i;
	}

	//Compute KdTre

	try{ nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data); }
	catch(std::exception &ex){ std::cout << ex.what() << std::endl; qFatal("fary en Nabo");}

	//Query for sorted distances to NEIGHBOORS neighboors
	Eigen::MatrixXi indices(NEIGHBOORS, pointList.size() );
	Eigen::MatrixXf distsTo(NEIGHBOORS, pointList.size() );
	if( NEIGHBOORS >= data.cols())
		NEIGHBOORS = data.cols()-1;
	nabo->knn(query, indices, distsTo, NEIGHBOORS, 0, Nabo::NNSearchF::SORT_RESULTS);

	//Connect the new points
	bool reachEnd;
	int32_t numExit = 0;
	bool connected = false;
	//qDebug() << __FUNCTION__ << lastCol + pointList.size() << indices.rows() << indices.cols();
	for(int i=lastCol, j=0; i<lastCol+pointList.size(); i++, j++)
	{
		Vertex vertexNew = vertexMap.value(i);
		for(uint k=0; k<NEIGHBOORS; k++)
		{
			int indKI = indices(k,j);
			Vertex vertexOld = vertexMap.value(indKI);
			//qDebug() << __FUNCTION__ << "Trying" << i << j <<k << "dist" << distsTo(k,j) << "indKI" << indKI;
			if( (distsTo(k,j) < MAX_DISTANTE_TO_CHECK_SQR) and (distsTo(k,j) > ROBOT_SIZE_SQR ))  //check distance to be lower that threshold and greater than robot size
			{
				QVec lastPoint;
				const QVec iiv = QVec::vec3(data(0,i), data(1,i), data(2,i));
				const QVec ijdie = QVec::vec3(data(0,indKI), data(1,indKI), data(2,indKI));
				qDebug() << __LINE__;
				reachEnd = sampler.checkRobotValidDirectionToTargetOneShot(iiv, ijdie);
				qDebug() << __FUNCTION__ << "i" << i << "to k" << k << indKI << "at dist " << distsTo(k,j) << "reaches the end:" << reachEnd;

				// If free path to neighboor, insert it in the graph
				if( reachEnd == true)
				{
					//Compute con comps to check it tne vertices belong to the same comp.
					ComponentMap componentMap;
					ConnectedComponents components = connectedComponents(componentMap);

					//Reject connecting elements belonging to the same connected component and already existing edges between vertices
					if( (boost::edge(vertexOld,vertexNew,graph).second == false )
							and (componentMap.at(vertexOld) != componentMap.at(vertexNew)) )
					{
						EdgePayload edge;
						edge.dist = (QVec(graph[vertexNew].pose) - QVec(graph[vertexOld].pose)).norm2();
						boost::add_edge(vertexOld, vertexNew, edge, graph);
						numExit++;
						connected = true;
					}
				}
			}
		}
		if( connected == false )
		{
// 			qDebug() << "---------- Could not connect to any of the existing ----------";
			connected = false;
		}
	}

	if( numExit > 0 )
		graphDirtyBit = true;

	return numExit;
}


/**
 * @brief Selects the two biggest con comps and try to connect their closest points. The number of vertices does not change
 * @return void
 */
bool PlannerPRM::connectIsolatedComponents(int32_t &numConnections)
{
// 	qDebug() << __FUNCTION__ << "Expanding graph...";

	//compute connected components
	ComponentMap cm;
	ConnectedComponents components = connectedComponents(cm);

	//If only one component, ntohing to do here
	if( components.size() < 2)
		return false;

	//Sort the components by size
	std::sort( components.begin(), components.end(),
			   []( CComponent a, CComponent b){ return a.first > b.first; });

	//Recover the two first indexes
	uint32_t largestSize  = components.front().first;
	uint32_t largest2Size = (*(components.cbegin() + 1)).first;

	// Compute closest points between both comps using KdTree.
	// Fist, build a Matrix with the elements of the largest to create a KdTree

// 	qDebug() << __FUNCTION__ << "Sizes: " << largestSize << largest2Size;
	const uint32_t neighboors = 1;
	Eigen::MatrixXf dataL( 3 , largestSize );   //Watch R3 dependant
	Eigen::MatrixXf queryL(3 , largest2Size );
	Eigen::MatrixXi indicesL(neighboors, largest2Size );
	Eigen::MatrixXf distsToL(neighboors, largest2Size );

 	QHash<uint32_t,Vertex> vertexMapL, vertexMapLL;
	uint32_t i = 0;
	//qDebug()  << __FUNCTION__ << "comps in C1";
	for( auto it : components[0].second)
	{
		dataL(0,i) = graph[it].pose.x();
		dataL(1,i) = graph[it].pose.y();
		dataL(2,i) = graph[it].pose.z();
		vertexMapL.insert(i,it);
		i++;
		//qDebug() << graph[it].pose;
	}
	//build the query matrix qith the second comp
	//pair = components[largest2];
	//qDebug()  << __FUNCTION__ << "comps in C2";
	i=0;
	for( auto it : components[1].second)
	{
		queryL(0,i) = graph[it].pose.x();
		queryL(1,i) = graph[it].pose.y();
		queryL(2,i) = graph[it].pose.z();
		vertexMapLL.insert(i,it);
		i++;
		//qDebug() << graph[it].pose;
	}
	// Build the KdTree is valid here.
	Nabo::NNSearchF *naboL = Nabo::NNSearchF::createKDTreeTreeHeap(dataL);

	// Query for sorted distances from one elements in C2  to elements in C1
	naboL->knn(queryL, indicesL, distsToL, neighboors, 0, 0);

	// Pick 5 elements from indicesa and try to connect them
	QVec inds = QVec::uniformVector(largest2Size, 0, largest2Size-1);

	int nExit = 0;
	for (uint i=0; i<largest2Size; i++)
	{
		Vertex v = vertexMapL.value(indicesL(0,i));
		Vertex vv = vertexMapLL.value(i);
		qDebug() << __LINE__;
		if( sampler.checkRobotValidDirectionToTargetOneShot( graph[v].pose , graph[vv].pose) )
		{
			qDebug() << "aqui";
			EdgePayload edge;
			edge.dist = distsToL(i);
			boost::add_edge(v, vv, edge, graph);
// 			qDebug() << __FUNCTION__ << "Exito" << graph[v].vertex_id << "to " << graph[vv].vertex_id;
			nExit++;
		}
	}

// 	qDebug() << __FUNCTION__  << "After expand, " << nExit << " new connections";
	numConnections = nExit;

	if( numConnections > 0)
		graphDirtyBit = true;

// 	//sample a few points aronud closestPose. a Gaussian sample would be better
// 	const uint32_t MAX_POINTS = 5;
// 	//QList<QVec> pList = sampler.sampleFreeSpaceR2Uniform( QRectF(10, 20, 50, 70), MAX_POINTS);
// 	QList<QVec> pList = sampler.sampleFreeSpaceR2Gaussian(closestPose.x(), closestPose.z(), sqrt(minDist),sqrt(minDist), MAX_POINTS);

	//try to join random points form both components


	//call construct
	//constructGraph(pList);

	return true;
}

/**
 * @brief Remove all small components in the graph with a size smaller thatn minSize.
 *
 * @param minSize ...
 * @return number of componets removed
 */
int32_t PlannerPRM::removeSmallComponents(int32_t minSize)
{
	//qDebug() << __FUNCTION__;

	ComponentMap cm;
	ConnectedComponents components = connectedComponents(cm);
	uint32_t nc = 0;

 	for( auto p : components )
	{
		if( p.first < minSize )
		{
			for( auto q : p.second )
			{
				boost::clear_vertex( q, graph);
				boost::remove_vertex(q, graph);
			}
			nc++;
		}
	}
	//qDebug() << __FUNCTION__  << "Graph after removing has " << boost::num_vertices(graph) << " vertices and" << components.size() << "components" ;

	if( nc > 0 )
	{
		rebuildExternalData();
		graphDirtyBit = true;
	}

	return nc;
}

/**
 * @brief Remove elements that are too closed
 *
 * @return void
 */
int32_t PlannerPRM::removeTooCloseElements( int32_t maxDist)
{
	std::vector<Vertex> toDelete;

	//libNabo uses squared dist
	maxDist = maxDist*maxDist;

	// We use the KDTree structure to compute distances
	Eigen::MatrixXi indices(1, data.cols() );
	Eigen::MatrixXf distsTo(1, data.cols() );

	nabo->knn(data, indices, distsTo, 1, 0, Nabo::NNSearchF::SORT_RESULTS);

	for(int i=0; i<data.cols(); i++)
		if( distsTo(0,i) < maxDist )
			toDelete.push_back( vertexMap.value(i) );

	for( auto it : toDelete )				//CHECK THAT NO NEW DISC COMPS ARE CREATED
	{
		boost::clear_vertex( it, graph);
 		boost::remove_vertex(it, graph);
	}

	if( toDelete.size() > 0 )
	{
		rebuildExternalData();
		graphDirtyBit = true;
	}
	return toDelete.size();
}
////////////////////////////////////////////////////////////////////////
/// UTILITIES
////////////////////////////////////////////////////////////////////////

/**
 * @brief Rebuilds de data matrix used for KDtree and the QHash that links Vertex to data indices
 *
 * @return bool
 */
bool PlannerPRM::rebuildExternalData()
{

	//get size of graph
	int nPoints = boost::num_vertices(graph);

	if( nPoints == 0 )
	{
		// qDebug() << __FUNCTION__ << "Empty graph. Returning";
		return false;
	}

	//Expand the matrix
	data.resize( 3, nPoints );

	//Clean the vertex-to-data hash
	vertexMap.clear();

	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
	{
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		vertexMap.insert(i,v);
		i++;
	}

	//Compute KdTre
	if(nabo != nullptr)
		delete nabo;
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);

	//Ready to go!!
	return true;
}


void PlannerPRM::writeGraphToStream(std::ostream &stream)
{
	// Create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
 	typedef std::map<Vertex, size_t> IMap;
 	typedef boost::associative_property_map<IMap> IndexMap;
	IMap indexMap;
	IndexMap index( indexMap );

 	//indexing the vertices
 	BGL_FORALL_VERTICES(v, graph, Graph)
 		boost::put(index, v, graph[v].vertex_id);

 	std::vector<int> components(boost::num_vertices(graph));
 	boost::iterator_property_map< std::vector<int>::iterator, IndexMap> compIterMap( components.begin(), index );

  	boost::write_graphviz(stream, graph, make_vertex_writer(boost::get(&VertexPayload::vertex_id, graph),boost::get(&VertexPayload::pose, graph)),
  										 make_edge_writer(boost::get(&EdgePayload::dist, graph)), boost::default_writer(), index);
}

void PlannerPRM::readGraphFromFile(QString name)
{
 	std::ifstream fin(name.toStdString().c_str());

	boost::dynamic_properties dynamicProperties;
	dynamicProperties.property("Index", boost::get(&VertexPayload::vertex_id, graph));
    dynamicProperties.property("Pose", boost::get(&VertexPayload::pose, graph));
    dynamicProperties.property("Distance", boost::get(&EdgePayload::dist, graph));
 	try
    {
		boost::read_graphviz(fin, graph, dynamicProperties, "Index" );
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
    //writeGraphToStream(std::cout);

	data.resize(3,boost::num_vertices(graph));		//ONLY 3D POINTS SO FAR
// 	qDebug() << __FUNCTION__ << "Graph size" << boost::num_vertices(graph);
	int i=0;

	BGL_FORALL_VERTICES(v, graph, Graph)
    {
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		//qDebug() << graph[v].pose;
		vertexMap.insert(i,v);
		i++;
	}
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);

	ComponentMap cm;
	connectedComponents(cm);
}

void PlannerPRM::setInnerModel(InnerModel *innerModel_)
{
	innerModel = new InnerModel(*innerModel_);
}

////////////////////////////////////////////////////////////////////////
/// LEARNERS
////////////////////////////////////////////////////////////////////////

/**
 * @brief Called form above to do some learning while awaiting for new orders. Each learning primitive has to take short
 *
 * @return bool
 */
bool PlannerPRM::learnForAWhile()
{
	ComponentMap cm;
	static int iter = 0, verticesAnt=0, edgesAnt=0, compsAnt=0;
	int32_t connected, pointsAdded, deles;
	QList<QVec> points;
	int32_t removed=0;
	QVector<int> cola;

	if( verticesAnt > 100)
	{
// 		qDebug() << __FUNCTION__ << "Graph with 100 nodes. Only CONNECTING, REMOVE Y TOOCLOSE";
		cola << 1 << 1 << 1 << 2 << 1 << 1 << 2 << 1 << 2 << 3;   //relative ratios of primitive execution
	}
	else
	{
		cola << 0 << 0 << 1 << 0 << 0 << 1 << 0 << 0 << 2 << 0;   //relative ratios of primitive execution
	}
	int ind;
	ind  = cola[iter++%10];
	switch (ind) {
 		case 0:
 			points = sampler.sampleFreeSpaceR2( 5 );
			pointsAdded = constructGraph(points);
			break;
		case 1:
			connected = connectIsolatedComponents(connected);
			break;
		case 2:
			removed = removeSmallComponents();
			break;
		case 3:
 			deles = removeTooCloseElements();
 		default:
			break;
	}

// 	qDebug() << __FUNCTION__ << "--------------------------BEGIN--------------------------------------";
// 	qDebug() << "Action taken:";
	switch (iter) {
		case 0:
// 			qDebug() << "	RemoveSmallComponents:" << removed << "components removed";
			break;
		case 1:
// 			qDebug() << "	ConnectIsolatedComponents:" << connected << "components connected";
			break;
		case 2:
// 			qDebug() << "	ConstructGraph:" <<  pointsAdded << "new points inserted";
			break;
 		case 3:
// 			qDebug() << "	RemoveTooCloseElements:" <<  deles << "points deleted";
		default:
			break;
	}
	int comps = connectedComponents(cm).size();
// 	qDebug() << "The graph now has: ";
// 	qDebug() << "	Vertices:" << boost::num_vertices(graph);
// 	qDebug() << "	Vertices variation:" << boost::num_vertices(graph) - verticesAnt;
// 	qDebug() <<	"	Edges:" << boost::num_edges(graph);
// 	qDebug() << "	Edges variation:" << boost::num_edges(graph) - edgesAnt;
// 	qDebug() << "	Components:" << comps;
// 	qDebug() << "	Components variation:" << comps - compsAnt;
// 	qDebug() << __FUNCTION__ << "-------------------------END-----------------------------------------";

	compsAnt = comps;
	verticesAnt = boost::num_vertices(graph);
	edgesAnt = boost::num_edges(graph);
	//iter = (iter+1) % 3;

	//We need a measure of progress. Some comb of conn comps, removed, connected and graph size
	//With this measure we can control de learning time and the amount dedicated to each primitive.

	//remember
	std::ofstream fout("grafo.dot");
	writeGraphToStream(fout);

	return true;
}


/**
 * @brief Gets a fresh new path driven by the robot and adds a subsample of the points to the graph
 *
 * @param path ...
 * @return void
 */
bool PlannerPRM::learnPath(const QList< QVec >& path)
{
 	qDebug() << __FUNCTION__ << "Learning the path with" << path.size() << " points";

//	const int nWays=10;   //Max number of point to be inserted
	const int MIN_STEP = 350;

	if( path.size() < 2)
		return false;

	QList<QVec> sList;
	float dist =0;

	//Trim the path to a fixed number of waypoints
	for(int i=0; i<path.size()-1; i++)
	{
		dist += (path[i]-path[i+1]).norm2();
		if( dist > MIN_STEP)
		{
			sList << path[i];
			dist = 0;
		}
	}

 	qDebug() << __FUNCTION__ << "Learning with shortened path of" << sList.size() << " points";
	constructGraph( sList, 10, 2000, 400);
	learnForAWhile();
	return true;
}

////////////////////////////////////////////////////////////////////////
/// SMOOTHERS
////////////////////////////////////////////////////////////////////////

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 *
 * @param list List of poses comprising the path
 * @return void
 */
void PlannerPRM::smoothPath( const QList<QVec> & list)
{
	qDebug() << __LINE__;
	if ( sampler.checkRobotValidDirectionToTargetOneShot( list.first(), list.last()) )
	{
		if(currentSmoothedPath.contains(list.first()) == false)
		  currentSmoothedPath.append(list.first());
		if(currentSmoothedPath.contains(list.last()) == false)
		  currentSmoothedPath.append(list.last());
		return;
	}
	else		//call again with the first half first and the second half later
	{
		if(list.size()>2)
		{
			smoothPath( list.mid(0,list.size()/2 +1));
		    smoothPath( list.mid( list.size()/2 , -1 ));
		}
	}
}

void PlannerPRM::smoothPathIter(QList<QVec> & list)
{

	//pick two ordered points and check free path between them
	int tam = list.size()*3;
	int i=0;
	while( i< tam and list.size() > 4)
	{
		int first = (int)QVec::uniformVector(1,0, list.size()-2)[0];
		int second = (int)QVec::uniformVector(1,first+2,list.size()-1)[0];

		QList<QVec>::iterator it = list.begin()+first;
		QList<QVec>::iterator itt = list.begin()+second;

		qDebug() << __LINE__;
		if ( sampler.checkRobotValidDirectionToTargetOneShot( *it, *itt ))
		{
			for(QList<QVec>::iterator ir = it + 1; ir != itt-1; ++ir)
			{
				list.erase(ir);
// 				qDebug() << __FUNCTION__ << "list size" << list.size();
			}
		}
		i++;
	}
}

////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
/// DRAW
////////////////////////////////////////////////////////////////////////

bool PlannerPRM::drawGraph(InnerModelViewer *innerViewer)
{
	cleanGraph(innerViewer);
	InnerModelDraw::addTransform(innerViewer, "graph", "floor");

	QString item;

	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
    {
		item = "g_" + QString::number(i);
		QString  parentT = QString("g_") + QString::number(i);
		InnerModelDraw::addTransform(innerViewer, parentT, "graph");
		innerViewer->innerModel->updateTransformValues(parentT, graph[v].pose.x(), 10, graph[v].pose.z(), 0,0,0);
		InnerModelDraw::addPlane_ignoreExisting(innerViewer, item+"_plane", parentT, QVec::vec3(0,0,0), QVec::vec3(0,1,0), "#00A0A0", QVec::vec3(60,60,10));
		//qDebug() << "Vertices inserted " << item << item + "_plane";
		i++;
	}

	i=0;
	BGL_FORALL_EDGES(e, graph, Graph)
    {
		item = "ge_" + QString::number(i);
		QVec p1 = graph[boost::source(e,graph)].pose;
		QVec p2 = graph[boost::target(e,graph)].pose;
		QVec center = (p2-p1)/(T)2.f;

		QString  parentTE = QString("ge_") + QString::number(i);
		InnerModelDraw::addTransform(innerViewer, parentTE, "graph");
		innerViewer->innerModel->updateTransformValues(parentTE, p1.x()+center.x(), p1.y()+center.y(), p1.z()+center.z(),  0, QLine2D(p1,p2).getAngleWithZAxis()+M_PI/2, 0 );
		InnerModelDraw::addPlane_ignoreExisting(innerViewer, QString("ge_")+QString::number(i)+"_plane", parentTE, QVec::vec3(0,0,0), QVec::vec3(0,1,0), "#00A0A0", QVec::vec3((p1-p2).norm2(), 15 , 15));
		i++;
	}
	return true;
}

void PlannerPRM::cleanGraph(InnerModelViewer *innerViewer)
{
	if (innerViewer->innerModel->getNode("graph"))
		InnerModelDraw::removeNode(innerViewer, "graph");
}

// std::tuple<std::vector<Vertex>, QMap<u_int32_t, VertexIndex> > PlannerPRM::connectedComponents()
// {
// 	qDebug() << __FUNCTION__;
//
// 	//create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
// 	typedef std::map<Vertex, size_t>IndexMap;
// 	IndexMap indexMap;
// 	boost::associative_property_map<IndexMap> propmapIndex(indexMap);
//     //indexing the vertices
// 	int i=0;
// 	BGL_FORALL_VERTICES(v, graph, Graph)
// 		boost::put(propmapIndex, v, i++);
//
// 	std::vector<VertexIndex> rank(boost::num_vertices(graph));
// 	std::vector<Vertex> parent(boost::num_vertices(graph));
// 	boost::disjoint_sets<VertexIndex*, Vertex*> ds(&rank[0], &parent[0]);
//
// 	boost::initialize_incremental_components(graph, ds);
// 	boost::incremental_components(graph, ds);
//
// 	QMap<u_int32_t, VertexIndex> sizeComps;
//  	Components components(parent.begin(),parent.end(), propmapIndex);
// //	Components components(propmapIndex);
//
//
// 	// get the property map for vertex indices
// //     typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
// //     IndexMap index = boost::get(boost::vertex_index, graph);
//
// 	std::cout << "Number of connected components " << components.size() << std::endl;
// 	for(auto p : components)
// 	{
// 		std::cout << "Component " << p << " contains: " << std::endl;
// 		auto pair = components[p];
// 		uint32_t sizeC = 0;
// 		for( auto it = pair.first; it != pair.second; ++it)
// 		{	sizeC++;
// 		//	std::cout << graph[*it].index << " ";
// 		}
// 		//sizeComps.push_back(sizeC);
// 		sizeComps.insert(sizeC, p);
// 		std::cout << std::endl << "Total: " << sizeC << " nodes" << std::endl;
// 	}
// 	std::cout  << "--------------------------------" << std::endl;
//
// 	return std::make_tuple(parent, sizeComps);
//}
