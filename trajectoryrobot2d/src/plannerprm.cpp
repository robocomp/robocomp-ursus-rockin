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

PlannerPRM::PlannerPRM(const InnerModel& innerModel_, uint nPoints, uint neigh,  QObject *parent)
{
	xMin = 0.;
	xMax = 10000.;
	zMin = -10000.;
	zMax = 0.;
			
	innerModel = new InnerModel(innerModel_);
	
	//Init random sequence generator
	qsrand ( QTime::currentTime().msec() );
	
	//Eigen matrix for libanabo
	NUM_POINTS = nPoints;
	NEIGHBOORS = neigh;
	data.resize(3,NUM_POINTS);		//ONLY 3D POINTS SO FAR

	if( QFile("grafo.dot").exists())
	{
		readGraphFromFile("grafo.dot");
		searchGraph(QVec(),QVec());
		
		qFatal("fary");
	}
	else
		createGraph(innerModel, NUM_POINTS, NEIGHBOORS, 2500.f);
		
}

bool PlannerPRM::computePath(const QVec& target, InnerModel* inner)
{	
	qDebug() << __FILE__ << __FUNCTION__ << "Starting planning with target at:" << target << "and robot at:" << inner->transform("world","robot");
	
	QVec currentTarget = target;  //local variable to get samples from free space
   
	//We need to resynchronize here because in subsequent call the robot will be "Dios sabe dónde"
	QVec robot = inner->transform("world", "robot");	
	QVec robotRotation = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min();
	innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());

	QList<QVec> currentPath;   			//Results will be saved here
	
	//Build list of colision meshes
	robotNodes.clear(); restNodes.clear();
	recursiveIncludeMeshes(innerModel->getRoot(), "robot", false, robotNodes, restNodes);
		
	//If target on obstacle, abort.  OJO targetRotation is not specified, Using robot initial orientation
	if (collisionDetector(target,robotRotation,innerModel) == true)
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
		return false;
	}
		
	
	//search in graph closest point to origin
	searchGraph(robot,target);
	
	//connect to graph
	
	//search in graph closest point to target
	
	//connect to graph 
	
	return true;
	
}

bool PlannerPRM::searchGraph(const QVec& origin, const QVec& target)
{
	//prepare the query
	Eigen::MatrixXi indices;
	Eigen::MatrixXf distsTo;
	Eigen::MatrixXf query(3,2);
	indices.resize(2, query.cols());
	distsTo.resize(2, query.cols());
	query(0,0) = origin.x();query(1,0) = origin.y();query(2,0) = origin.z();
	query(0,1) = target.x();query(1,1) = target.y();query(2,1) = target.z();
	
	nabo->knn(query, indices, distsTo, 2, 0, 0);
	
	Vertex vertexOrigin = vertexMap.value(indices(0));
	Vertex vertexTarget = vertexMap.value(indices(1));
	
	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<float> distances(boost::num_vertices(graph));    // To store distances
	std::vector<int> vertex_index_map(boost::num_vertices(graph));
 
	IndexMap indexMap = boost::get(boost::vertex_index, graph);
	auto predecessorMap = boost::make_iterator_property_map(&predecessors[0], indexMap);
	auto distanceMap = boost::make_iterator_property_map(&distances[0], indexMap);

	boost::dijkstra_shortest_paths(graph, vertexOrigin, boost::weight_map(boost::get(&EdgePayload::dist, graph))
												.predecessor_map(predecessorMap)
												.distance_map(distanceMap));

	// Output results
	std::cout << "distances and parents:" << std::endl;
	auto nameMap( boost::get(&VertexPayload::index, graph) );
	auto poseMap( boost::get(&VertexPayload::pose, graph) );
 
	BGL_FORALL_VERTICES(v, graph, Graph)
	{
		std::cout << "distance(" << nameMap[vertexOrigin] << ", " << nameMap[v] << ") = " << distanceMap[v] << ", ";
		std::cout << "predecessor(" << nameMap[v] << ") = " << nameMap[predecessorMap[v]] << std::endl;
	}

	// Extract a shortest path
 
	PathType path;
	Vertex v = vertexTarget;
	
	// Start by setting 'u' to the destintaion node's predecessor   |||// Keep tracking the path until we get to the source
	for( Vertex u = predecessorMap[v]; u != v; v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
	{
		std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph);
		Graph::edge_descriptor edge = edgePair.first;
		path.push_back( edge );
	}
 
	if(path.size() > 0)
	{
		// Write shortest path
		currentPath.clear();
		std::cout << "Shortest path from origin to target:" << std::endl;
		float totalDistance = 0;
		for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
		{
			std::cout << nameMap[boost::source(*pathIterator, graph)] << " -> " << nameMap[boost::target(*pathIterator, graph)]
					<< " = " << boost::get( &EdgePayload::dist, graph, *pathIterator ) << std::endl;
			currentPath.append(poseMap[boost::source(*pathIterator, graph)]);
		}
		std::cout << std::endl;
		std::cout << "Distance: " << distanceMap[vertexTarget] << std::endl;
		return true;
	}
	else
		return false;
}

void PlannerPRM::createGraph(InnerModel *inner, uint NUM_POINTS, uint NEIGHBOORS, float MAX_DISTANTE_TO_CHECK)
{
	
	qDebug() << __FUNCTION__ << "Creating grpah";
	
	float MAX_DISTANTE_TO_CHECK_SQR = MAX_DISTANTE_TO_CHECK * MAX_DISTANTE_TO_CHECK;
	bool reachEnd;
	
	for(uint i=0; i<NUM_POINTS; i++)
	{
		QVec point = sampleFreeSpaceR2(inner, QPointF(xMax, zMin));
		data(0,i) = point.x();
		data(1,i) = point.y();
		data(2,i) = point.z();
	}
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);
	
	Eigen::MatrixXi indices;
	Eigen::MatrixXf distsTo;
	indices.resize(NEIGHBOORS, data.cols());
	distsTo.resize(NEIGHBOORS, data.cols());

	//Query for sorted distances to NEIGHBOORS neighboors  
	nabo->knn(data, indices, distsTo, NEIGHBOORS, 0, Nabo::NNSearchF::SORT_RESULTS);
	
	Vertex vertex, vertexN;
	
	for(uint i=0; i<NUM_POINTS; i++)
	{
		//Add new vertex if not present
		if( vertexMap.contains(i))
		{
			vertex = vertexMap.value(i);
			qDebug() << "Recovered vertex " << i;
		}
		else
		{
			vertex = boost::add_vertex(graph);
			vertexMap.insert(i,vertex);
			graph[vertex].pose = QVec::vec3(data(0,i), data(1,i), data(2,i));
			graph[vertex].index = i;
			qDebug() << "Insert vertex " << i;
		}
		
		for(uint k=0; k<NEIGHBOORS; k++)
		{	
			if(distsTo(k,i) < MAX_DISTANTE_TO_CHECK_SQR )  //check if distance is lower that threshold
			{
				trySegmentToTarget(QVec::vec3(data(0,i), 0, data(1,i)), QVec::vec3(data(0,indices(k,i)), 0, data(1,indices(k,i))), reachEnd);
				qDebug() << "i" << i << "k" << k << indices(k,i) << distsTo(k,i);
				
				//if free path to neighboor, insert it in the graph if does not exist
				if( reachEnd == true)
				{
					if( vertexMap.contains(indices(k,i)))
					{
						vertexN = vertexMap.value(indices(k,i));
						qDebug() << "Recovered vertex " << indices(k,i);
					}
					else
					{
						vertexN = boost::add_vertex(graph);
						vertexMap.insert(indices(k,i),vertexN);
						graph[vertexN].pose = QVec::vec3(data(0,indices(k,i)), 0, data(1,indices(k,i)));
						graph[vertexN].index = indices(k,i);	
						qDebug() << "Insert vertex " << indices(k,i);
					}			

					//compute connected components to reject same isle elements
					std::vector<VertexIndex> rank(boost::num_vertices(graph));
					std::vector<Vertex> parent(boost::num_vertices(graph));
					boost::disjoint_sets<VertexIndex*, Vertex*> ds(&rank[0], &parent[0]);
					boost::initialize_incremental_components(graph, ds);
					boost::incremental_components(graph, ds);
 					Components components(parent.begin(),parent.end());
			
					bool sameComp = boost::same_component(vertex,vertexN,ds);

					qDebug() << "Try to create and edge from" << graph[vertex].index << "a" << graph[vertexN].index;
					qDebug() << "true if already exists" << (boost::edge(vertex,vertexN,graph).second == true) << "same" << sameComp;
					
					if( (boost::edge(vertex,vertexN,graph).second == false ) and (sameComp == false) )
					{
						EdgePayload edge;
						edge.dist = (QVec(graph[vertex].pose) - QVec(graph[vertexN].pose)).norm2();
						boost::add_edge(vertex, vertexN, edge, graph);	
						qDebug() << "insertamos de" << graph[vertex].index << "a" << graph[vertexN].index;
					}
				}
			}
			else
			{
				break;
			}
		}
	}
	
	//print and save the graph with payloads
	writeGraphToStream(std::cout);
	std::ofstream fout("grafo.dot");
	writeGraphToStream(fout);
		
}

void PlannerPRM::writeGraphToStream(std::ostream &stream)
{
	boost::write_graphviz(stream, graph, make_vertex_writer(boost::get(&VertexPayload::index, graph),boost::get(&VertexPayload::pose, graph)),
										 make_edge_writer(boost::get(&EdgePayload::dist, graph)));
}

void PlannerPRM::readGraphFromFile(QString name)
{
 	std::ifstream fin(name.toStdString().c_str());
	boost::dynamic_properties dynamicProperties; 
	dynamicProperties.property("Index", boost::get(&VertexPayload::index, graph));
    dynamicProperties.property("Pose", boost::get(&VertexPayload::pose, graph));
    dynamicProperties.property("Distance", boost::get(&EdgePayload::dist, graph)); 
 	try
    {
		bool status = boost::read_graphviz(fin, graph, dynamicProperties, "Index" );
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    } 
    writeGraphToStream(std::cout);
	
	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
    {
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		vertexMap.insert(i,v);
	}
	
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);
	
}

void PlannerPRM::setInnerModel(const InnerModel& innerModel_)
{
	innerModel = new InnerModel(innerModel_);
}

////////////////////////////////////////////////////////////////////////
/// COLLISION DETECTOR - FCL -
////////////////////////////////////////////////////////////////////////


bool PlannerPRM::collisionDetector(const QVec &position, const QVec &rotation, InnerModel *inner)
{
	inner->updateTransformValues("robot", position.x(), position.y(), position.z(), rotation.x(), rotation.y(), rotation.z());
	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			if (inner->collide(robotNodes[in], restNodes[out]))
			{
				return true;
			}
		}
	}

	return false;
}

void PlannerPRM::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
{
	if (node->id == robotId)
	{
		inside = true;
	}
	
	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;

	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		for (int i=0; i<node->children.size(); i++)
		{
			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out);
		}
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if (inside)
		{
			in.push_back(node->id);
		}
		else
		{
			out.push_back(node->id);
		}
	}
}

/////////////////////////////////////////////////////
/// Samplers. This baby here admits a lot of improvements!
////////////////////////////////////////////////////

/**
* @brief Picks a random point of the list created by the sampler and checks that it is out of obstacles (Free Space)
* 
* @param currentTarget Current target provided to bias the sample. 20% of the times the target is returned.
* @return RMat::QVec
*/

QVec PlannerPRM::sampleFreeSpaceR2(InnerModel *inner, const QPointF &XZLimits)  //ÑAPA!!!! meter los cuatro valores 
{
 	const QVec zeros(3,0.f);
 	bool collision = true;
	QVec p;
	while( collision == true )
	{
		p =	QVec::vec3( qrand()*XZLimits.x()/RAND_MAX, 0, qrand()*XZLimits.y()/RAND_MAX );
		collision = collisionDetector(p, zeros, inner);	
	}
	return p;
}

////////////////////////////////////////////////////////////////////////
/// LOCAL CONTROLLER. Also a lot of improvements fit here    												USE BINARY SEARCH!!!!!!!!!!!!!!!!!!!
////////////////////////////////////////////////////////////////////////


/**
 * @brief Local controller. Goes along a straight line connecting the current robot pose and target pose in world coordinates
 * checking collisions with the environment
 * @param origin Current pose
 * @param target Target pose 
 * @param reachEnd True is successful
 * @param arbol Current search tree
 * @param nodeCurrentPos ...
 * @return RMat::QVec final pose reached
 */
QVec PlannerPRM::trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd)
{
	//return trySegmentToTargetBinarySearch(origin, target, reachEnd, arbol, nodeCurrentPos);
	
	float stepSize = 100.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
	float step;
	
	//if too close return target
	if (nSteps == 0) 
	{
		reachEnd = true;
		return target;
	}
	step = 1./nSteps;
	
	//go along visual ray connecting robot pose and target pos in world coordinates
	// l*robot + (1-r)*roiPos = 0
	
	QVec point(3), pointAnt(3);
	float landa = step;
	QVec pos(3), front(3);
	
	pointAnt=origin;
	for(uint i=1 ; i<=nSteps; i++)
	{
		// center of robot position
		point = (origin * (1-landa)) + (target * landa);
		
		//Collision detector
		//qDebug() << point << origin << target << innerModel->transform("world","robot");
		if (collisionDetector(point, 0, innerModel) == true)
		{
		  reachEnd = false;
		  return pointAnt;
		}
		 
		landa = landa + step;
		pointAnt = point;
	}
	reachEnd= true;
	return target;
}


// 					BOOST_FOREACH(VertexIndex current_index, components) 
// 					{
// 						std::cout << "component " << current_index << " contains: ";
// 						// Iterate through the child vertex indices for [current_index]
// 						BOOST_FOREACH(VertexIndex child_index,components[current_index]) 
// 						{
// 							std::cout << child_index << " {" << graph[child_index].index<< "} ";
// 						}
// 					std::cout << std::endl;
// 					}
	