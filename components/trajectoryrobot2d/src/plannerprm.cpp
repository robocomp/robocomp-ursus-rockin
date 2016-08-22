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
PlannerPRM::PlannerPRM()
{	
	//Number of points in graph
	//graphNumPoints = nPoints;
	
	//Number of neighbours in the graph
	//graphNeighPoints = neigh;
	
	//Default file name
	
}

void PlannerPRM::initialize(Sampler* sampler_, int nPointsInGraph, int nNeighboursInGraph)
{
	sampler = sampler_;
	
	////////////////////////
	/// Initialize RRTplaner
	////////////////////////
	plannerRRT.initialize(sampler); 
	 
	
	////////////////////////
	/// Check if graph already exists
	////////////////////////
	if( QFile(fileName).exists())
	{
		qDebug() << __FUNCTION__ << "Graph file exits. Loading";
		readGraphFromFile("grafo.dot");
	}
	else
	{
		qDebug() << __FUNCTION__ << "Graph file DOES NOT exit. Creating with " << nPointsInGraph << "nodes and " 
						 << nNeighboursInGraph << "neighboors";
		QList<QVec> pointList = sampler->sampleFreeSpaceR2(nPointsInGraph);
  	constructGraph(pointList, nNeighboursInGraph, 3500.f, 500);  ///GET From IM ----------------------------------
		std::ofstream fout(fileName.toStdString());
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
bool PlannerPRM::computePath(QVec& target, InnerModel* innerModel)
{
	QVec robot = innerModel->transform("world","robot");
	QVec robotRotation = innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min();
	
 	qDebug() << __FUNCTION__ << "Starting planning with robot at:" << robot <<  "and target at:" << target;

	/////////////////////////////////////////////
	//If robot on obstacle we can¡t proceed
	/////////////////////////////////////////////
	if( std::get<bool>(sampler->checkRobotValidStateAtTarget(robot, robotRotation)) == false)
	{
		qDebug() << __FUNCTION__ << "Robot collides in origin. Aborting planner";  
		return false;
	}
	
	/////////////////////////////////////////////
	//If target on obstacle find a point close to it on the robot-target line. For now returns
	/////////////////////////////////////////////
	if( sampler->searchRobotValidStateCloseToTarget(target) == false )
	{
		qDebug() << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
		return false;
	}
	
	////////////////////////////////////////////
	// PLanner uses another instance of InnerModel to plan so we resynchronize both before initiate planning
	////////////////////////////////////////////
	//innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());

	//Clean former path
	currentPath.clear();
	
	/////////////////////////////////////////////
	//Check if the target is in "plain sight"
	/////////////////////////////////////////////
	if ( sampler->checkRobotValidDirectionToTargetOneShot( robot, target) )
	{
 		qDebug() << __FUNCTION__ << "Target on sight. Proceeding";
		currentPath << robot << target;
		return true;
	}
	qDebug() << __FUNCTION__ << "Target not on sight. Proceeding";
	
	/////////////////////////////////////////////
	//If not, search in KD-tree for closest points to origin and target
	/////////////////////////////////////////////
	Vertex robotVertex, targetVertex;
	searchClosestPoints(robot, target, robotVertex, targetVertex);
	qDebug() << __FUNCTION__ << "Computing closest point to robot in graph";
	qDebug() << __FUNCTION__ << "Closest point to robot:" << graph[robotVertex].pose;
	qDebug() << __FUNCTION__ << "Closest point to targett:" << graph[targetVertex].pose;
	
	/////////////////////////////////////////////
	// Check if the closest point in the graph is in "plain sight"
	// Otherwise call the planner
	/////////////////////////////////////////////
	if ( sampler->checkRobotValidDirectionToTargetOneShot( robot, graph[robotVertex].pose) )
	{
 		qDebug() << __FUNCTION__ << "Closest point to robot in graph is ON sight";
		currentPath << robot << graph[robotVertex].pose;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Closest point to robot in graph is NOT on sight. Proceeding to search";
		/////////////////////////////////////////////
		//Obtain a free path from [robot] to [robotVertex] using RRTConnect. Return if fail.
		/////////////////////////////////////////////
		qDebug() << __FUNCTION__ << "Searching with RRT from ROBOT to closest point in graph (CPG)";
		QList<QVec> path;
		if (planWithRRT(robot, graph[robotVertex].pose, path) )
		{
			path.removeLast();
			currentPath << path;
			qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from ROBOT to CPG with a length of " << currentPath.size() << "steps.";
		}
		else
		{
			qDebug() << __FUNCTION__ << "ERROR: Path from ROBOT to CPG NOT FOUND:";
			return false;
		}
	}

	/////////////////////////////////////////////
	//Now we are in the graph
	//Search in graph a minimun path using Dijkstra algorithm
	/////////////////////////////////////////////
	if( robotVertex != targetVertex )  //Same node for both. We should skip searchGraph
	{
		std::vector<Vertex> vertexPath;
		if ( searchGraph(robotVertex, targetVertex, vertexPath) == true)
			for( Vertex v : vertexPath )
				currentPath.append(graph[v].pose);
		else //No path found. The path does not connect to target's closest point in graph
		{
			qDebug() << __FUNCTION__ << "No path through graph. Starting RRTConnect from " << graph[robotVertex].pose << " to" <<  target;
		  QList<QVec> path;	
		  if ( planWithRRT(graph[robotVertex].pose, target, path) )
			{
				path.removeFirst();
				currentPath += path;
	  		qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from CPG to CPT with a length of " << currentPath.size() << "steps.";
				qDebug() << __FUNCTION__ << "RRTConnect path form CPG to CPT:" <<  path;
			}
			else
				return false;
		}
	}
	else	//add the only node
		currentPath += graph[robotVertex].pose;

	
	///////////////////////////////////////////////////////////
	// Check if the closest point in the graph to the TARGET is in "plain sight".
	// Otherwise call the planner
	////////////////////////////////////////////////////////////7
	if ( sampler->checkRobotValidDirectionToTargetOneShot( graph[targetVertex].pose, target ))
	{
 		qDebug() << __FUNCTION__ << "Closest point to target in graph ON sight";
		currentPath << target << graph[targetVertex].pose;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Closest point to target in graph NOT on sight. Proceeding to search";
		
		/////////////////////////////////////////////
		//Obtain a free path from target to targetVertex using RRTConnect.
		/////////////////////////////////////////////
		qDebug() << __FUNCTION__ << "Searching with RRT from closest point in graph (CPG) to TARGET";
		QList<QVec> path;
		if (planWithRRT(graph[robotVertex].pose, target, path) == true)
		{
				path.removeLast();
				currentPath << path;
		}
		else
		{
				qDebug() << __FUNCTION__ << "ERROR: Path from TARGET to CPG NOT FOUND using RRT planner. Can't do better";
				return false;
		}
	}

	/////////////////////////////////////
	// Smoothing
	/////////////////////////////////////
	pathSmoother(currentPath);
	return true;
}

////////////////////////////////////////////////////////////////////////
/// One query planner to be used when no path found un PRM
////////////////////////////////////////////////////////////////////////


bool PlannerPRM::planWithRRT(const QVec &origin, const QVec &target, QList<QVec> &path)
{
 	qDebug() << __FUNCTION__ << "RRTConnect start...";
	const float diffV = (origin-target).norm2();
	
	if (diffV < 200) // HALF ROBOT RADIOUS:::::::::::::::::::::::::::::::::::::
	{
		qDebug() << __FUNCTION__ << "Origin and target too close. Returning trivial path";
		path << origin << target;
		return true;
	}

	QVec point;
	if (sampler->checkRobotValidDirectionToTargetOneShot( origin, target ))
	{
		qDebug() << __FUNCTION__ << "Found target directly in line of sight. Returning trivial path";
		path << origin << target;
		return true;
	}

	qDebug() << __FUNCTION__ << "Calling Full Power of RRTConnect OMPL planner. This may take a while";
	
	if (plannerRRT.computePath(origin, target, 600) == true)  //max time but it would be better an asynchronous versin
	{
		path += plannerRRT.getPath();
		return true;
	}
	else
		return false;
}

/**
 * @brief Searches in the graph closest points to origin and target
 * 
 * @param origin ...
 * @param target ...
 * @param originVertex to return selected vertex
 * @param targetVertex ...
 * @return void
 */
void PlannerPRM::searchClosestPoints(const QVec& origin, const QVec& target, Vertex& originVertex, Vertex& targetVertex)
{
	qDebug() << __FUNCTION__ << "Searching from " << origin << "and " << target;

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

 	qDebug() << __FUNCTION__ << "Closest point to origin is at" << data(0,indices(0,0)) << data(1,indices(0,0)) << data(2,indices(0,0)) << " and corresponds to " << graph[originVertex].pose;
 	qDebug() << __FUNCTION__ << "Closest point to target is at" << data(0,indices(0,1)) << data(1,indices(0,1)) << data(2,indices(0,1)) << " and corresponds to " << graph[targetVertex].pose;

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
	
	qDebug() << __FUNCTION__ << "Searching the graph between " << graph[originVertex].pose << "and " << graph[targetVertex].pose;
	
	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<float> distances(boost::num_vertices(graph));     // To store distances

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

	//////////////////////////
	// Extract a shortest path
	//////////////////////////
	PathType path;
	Vertex v = targetVertex;

	//////////////////////////
	// Start by setting 'u' to the destintaion node's predecessor   |||// Keep tracking the path until we get to the source
	/////////////////////////
	for( Vertex u = predecessorMap[v]; u != v; v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
	{
		std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph);
		Graph::edge_descriptor edge = edgePair.first;
		path.push_back( edge );
	}

 	qDebug() << __FUNCTION__ << " Path found with length: " << path.size() << "steps and length " << 	distanceMap[targetVertex];
;
	Vertex lastVertex;
	if(path.size() > 0)
	{
		vertexPath.clear();
		for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
		{
			vertexPath.push_back(boost::source(*pathIterator, graph));
			lastVertex = boost::target(*pathIterator, graph);
		}
		vertexPath.push_back(lastVertex);
		return true;
	}
	else
	{
		qDebug() << "Path no found between nodes";
		return false;
	}
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
/// Graph operations
/////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief 
 * 
 * @param pointList list of smapled points
 * @param NEIGHBOORS max number of neighbours
 * @param MAX_DISTANTE_TO_CHECK max distance to check free space
 * @param robotSize ...
 * @return int32_t
 */
int32_t PlannerPRM::constructGraph(const QList<QVec> &pointList, uint NEIGHBOORS, float MAX_DISTANTE_TO_CHECK, uint robotSize)
{

	qDebug() << __FUNCTION__ << "Constructing graph with " << pointList.size() << " new points and " <<  NEIGHBOORS << "neighboors";

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
				reachEnd = sampler->checkRobotValidDirectionToTargetOneShot(iiv, ijdie);
				//qDebug() << __FUNCTION__ << "i" << i << "to k" << k << indKI << "at dist " << distsTo(k,j) << "reaches the end:" << reachEnd;

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
		if( sampler->checkRobotValidDirectionToTargetOneShot( graph[v].pose , graph[vv].pose) )
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
// 	//QList<QVec> pList = sampler->sampleFreeSpaceR2Uniform( QRectF(10, 20, 50, 70), MAX_POINTS);
// 	QList<QVec> pList = sampler->sampleFreeSpaceR2Gaussian(closestPose.x(), closestPose.z(), sqrt(minDist),sqrt(minDist), MAX_POINTS);

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

/**
 * @brief Reads and existing graph from file
 * 
 * @param name ...
 * @return void
 */
void PlannerPRM::readGraphFromFile(QString name)
{
 	std::ifstream fin(name.toStdString().c_str());

	boost::dynamic_properties dynamicProperties;
	dynamicProperties.property("Index", boost::get(&VertexPayload::vertex_id, graph));
  dynamicProperties.property("Pose", boost::get(&VertexPayload::pose, graph));
  dynamicProperties.property("Distance", boost::get(&EdgePayload::dist, graph));
	
 	try{	boost::read_graphviz(fin, graph, dynamicProperties, "Index" );   }
  catch (std::exception& e){ std::cout << e.what() << std::endl; }
  
	data.resize(3,boost::num_vertices(graph));		//ONLY 3D POINTS SO FAR
 	qDebug() << __FUNCTION__ << "Graph size" << boost::num_vertices(graph);
	int i=0;

	BGL_FORALL_VERTICES(v, graph, Graph)
    {
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		vertexMap.insert(i,v);
		i++;
	}
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);

	ComponentMap cm;
	connectedComponents(cm);
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
 			points = sampler->sampleFreeSpaceR2( 5 );
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
	constructGraph( sList, 10, 100, 10);
	constructGraph(currentPath);
	learnForAWhile();
	return true;
}

////////////////////////////////////////////////////////////////////////
/// SMOOTHERS
////////////////////////////////////////////////////////////////////////
bool PlannerPRM::pathSmoother(QList<QVec> & pointList)
{
	currentSmoothedPath.clear();
	qDebug() << __FUNCTION__ << "number of points " << pointList.size();
	smoothPath(pointList);
	qDebug() << __FUNCTION__ << "number of points " << currentSmoothedPath.size();
	
	pointList = currentSmoothedPath;
	return true;
}

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 *
 * @param list List of poses comprising the path
 * @return void
 */
void PlannerPRM::smoothPath( const QList<QVec> & list)
{
	qDebug() << __LINE__;
	
	if ( sampler->checkRobotValidDirectionToTargetOneShot( list.first(), list.last()) )
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
		if ( sampler->checkRobotValidDirectionToTargetOneShot( *it, *itt ))
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


///////////////////////////////////////////////////////////////////////
/// DRAW
////////////////////////////////////////////////////////////////////////

/**
 * @brief Draws the graph on a InnerModelViewer instance
 * 
 * @param innerViewer ...
 * @return bool
 */
bool PlannerPRM::drawGraph(InnerModelViewer *innerViewer)
{
	cleanGraph(innerViewer);
	InnerModelDraw::addTransform(innerViewer, "graph", "world");

	QString item;
	int i=0;
	
	//compute connected components
	ComponentMap cm;
	ConnectedComponents components = connectedComponents(cm);
	QString color;
	
	for(auto comp : components)
	{
		color = "#A0A0A0";
		for(auto elem : comp.second)
		{			
			item = "g_" + QString::number(i);
			QString  parentT = QString("g_") + QString::number(i);
			InnerModelDraw::addTransform(innerViewer, parentT, "graph");
			innerViewer->innerModel->updateTransformValues(parentT, graph[elem].pose.x(), 10, graph[elem].pose.z(), 0,0,0);
			InnerModelDraw::addPlane_ignoreExisting(innerViewer, item + "_plane", parentT, QVec::vec3(0,0,0), QVec::vec3(0,1,0), 
																					color, QVec::vec3(60,60,10));
			i++;
		}
	}
	
// 	BGL_FORALL_VERTICES(v, graph, Graph)
//   {
// 		item = "g_" + QString::number(i);
// 		QString  parentT = QString("g_") + QString::number(i);
// 		InnerModelDraw::addTransform(innerViewer, parentT, "graph");
// 		innerViewer->innerModel->updateTransformValues(parentT, graph[v].pose.x(), 10, graph[v].pose.z(), 0,0,0);
// 		InnerModelDraw::addPlane_ignoreExisting(innerViewer, item + "_plane", parentT, QVec::vec3(0,0,0), QVec::vec3(0,1,0), 
// 																						"#00A0A0", QVec::vec3(60,60,10));
// 		i++;
// 	}

	i=0;
	BGL_FORALL_EDGES(e, graph, Graph)
    {
		item = "ge_" + QString::number(i);
		QVec p1 = graph[boost::source(e,graph)].pose;
		QVec p2 = graph[boost::target(e,graph)].pose;
		QVec center = (p2-p1)/(T)2.f;

		QString  parentTE = QString("ge_") + QString::number(i);
		InnerModelDraw::addTransform(innerViewer, parentTE, "graph");
		innerViewer->innerModel->updateTransformValues(parentTE, p1.x()+center.x(), p1.y()+center.y(), p1.z()+center.z(), 
																									 0, QLine2D(p1,p2).getAngleWithZAxis()+M_PI/2, 0 );
		InnerModelDraw::addPlane_ignoreExisting(innerViewer, QString("ge_")+QString::number(i)+"_plane", parentTE, QVec::vec3(0,0,0), 
																						QVec::vec3(0,1,0), "#00A0A0", QVec::vec3((p1-p2).norm2(), 15 , 15));
		i++;
	}
	return true;
}

void PlannerPRM::cleanGraph(InnerModelViewer *innerViewer)
{
  if (innerViewer->innerModel->getNode("graph"))
	 InnerModelDraw::removeNode(innerViewer, "graph");
}

void PlannerPRM::removeGraph(InnerModelViewer* innerViewer)
{
  cleanGraph(innerViewer);
  graph.clear();
  data.resize(0,0);
  qDebug() << __FUNCTION__	<< "graph size" << boost::num_vertices(graph);

  
}

// /**
//  * @brief Creates a grpah if it does not exist
//  * 
//  * @return void
//  */
// void PlannerPRM::createGraph()
// {
// 	//remove this, sampler must be up  and running
// 	//sampler->initialize(innerModel, outerRegion, innerRegions);
// 	
// 	QList<QVec> pointList = sampler->sampleFreeSpaceR2(graphNumPoints);
// 	qDebug() << __FUNCTION__ << "constructing new graph";
// 	constructGraph(pointList,graphNeighPoints, 2500.f, 400);
// 	std::ofstream fout(fileName);
// 	writeGraphToStream(fout);
// }
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
