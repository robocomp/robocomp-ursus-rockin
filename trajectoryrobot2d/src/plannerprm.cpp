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

PlannerPRM::PlannerPRM(const InnerModel& innerModel_, QObject* parent)
{
	xMin = 0.;
	xMax = 10000.;
	zMin = -10000.;
	zMax = 0.;
	
	innerModel = new InnerModel(innerModel_);
	
	//Init random sequence generator
	qsrand ( QTime::currentTime().msec() );

	
}

bool PlannerPRM::computePath(const QVec& target, InnerModel* inner)
{
	static bool firstTime = true;
	
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
		
	
	if(firstTime == true)
	{
		createGraph(inner);
		firstTime = false;
	}
	
	//search in graph closest point to origin
	
	//connect to graph
	
	//search in graph closest point to target
	
	//connect to graph 
	
	return true;
	
}

void PlannerPRM::createGraph(InnerModel *inner)
{
	qDebug() << __FUNCTION__ << "Creating grpah";
	
	const int NUM_POINTS = 500;
	const int NEIGHBOORS = 10;
	const int MAX_DISTANTE_TO_CHECK_SQR = 9000*9000;
	bool reachEnd;
	QHash<int,Vertex> vertexMap;
	
	Eigen::MatrixXf data(2,NUM_POINTS);
	
	for(int i=0; i<NUM_POINTS; i++)
	{
		QVec point = sampleFreeSpaceR2(inner, QPointF(xMax, zMin));
		data(0,i) = point.x();
		data(1,i) = point.z();
	}
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);
	
	Eigen::MatrixXi indices;
	Eigen::MatrixXf distsTo;
	indices.resize(NEIGHBOORS, data.cols());
	distsTo.resize(NEIGHBOORS, data.cols());

	nabo->knn(data, indices, distsTo, NEIGHBOORS, 0, Nabo::NNSearchF::SORT_RESULTS);
	
	Vertex vertex, vertexN;
	
	QVec point;
	Graph::vertex_iterator vertexIterator;
	for(uint i=0; i<NUM_POINTS; i++)
	{
		if( vertexMap.contains(i))
		{
			vertex = vertexMap.value(i);
			qDebug() << "Recovered vertex " << i;
		}
		else
		{
			vertex = boost::add_vertex(graph);
			vertexMap.insert(i,vertex);
			graph[vertex].pose = QVec::vec3(data(0,i), 0, data(1,i));
			graph[vertex].index = i;
			qDebug() << "Insert vertex " << i;
		}
		
		for(uint k=0; k<NEIGHBOORS; k++)
		{	
			if(distsTo(k,i) < MAX_DISTANTE_TO_CHECK_SQR )  //check if distance is lower that threshold
			{
				trySegmentToTarget(point, QVec::vec3(data(0,indices(k,i)), 0, data(1,indices(k,i))), reachEnd);
				qDebug() << "i" << i << "k" << k << indices(k,i) << distsTo(k,i);
				
				//if free path to neighboor, insert in graph an the nodes and edges
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

					//compute connected components
					std::vector<VertexIndex> rank(boost::num_vertices(graph));
					std::vector<Vertex> parent(boost::num_vertices(graph));
					typedef VertexIndex* Rank;
					typedef Vertex* Parent;
					boost::disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);
					boost::initialize_incremental_components(graph, ds);
					boost::incremental_components(graph, ds);
 					Components components(parent.begin(),parent.end());
			
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
					bool sameComp;
					sameComp = boost::same_component(vertex,vertexN,ds);

					qDebug() << "probamos de" << graph[vertex].index << "a" << graph[vertexN].index;
					qDebug() << "already exists" << (boost::edge(vertex,vertexN,graph).second == true) << "same" << sameComp;
					
					if( (boost::edge(vertex,vertexN,graph).second == false ) and (sameComp == false) )
					{
						boost::add_edge(vertex,vertexN,graph);
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
	//boost::dynamic_properties dp;
	//dp.property("node_id",get(boost::vertex_index, graph));
	//boost::write_graphviz_dp(std::cout, graph, dp);
	std::ofstream fout("grafo.dot");
	boost::write_graphviz(fout, graph);
	boost::print_graph(graph);
	
}

void PlannerPRM::readGraphFromFile(QString name)
{
// 	 std::ifstream fin(name.toStdString().c_str());
// 	 bool status = boost::read_graphviz(fin,graph);
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
