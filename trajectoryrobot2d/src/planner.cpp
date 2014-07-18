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

#include "planner.h"
#include <boost/concept_check.hpp>

/**
 * @brief Constructor of a RRT Planner class. Currently works for (x,0,z) poses (2D)
 * 
 * @param innerModel_ Pointer to InnerModel
 * @param parent ...
 */
Planner::Planner(const InnerModel &innerModel_, QObject *parent)
{
	
	// 	innerModel = innerModel_;
	//Clone innermodel
	innerModel = new InnerModel(innerModel_);
	//Add a fake robot (transform and mesh) to Innermodel to emulate paths with it. OJOOOO Se necesita un clon completo con brazos, etc para las colisiones
	
	//InnerModelTransform *t = innerModel->newTransform("baseT", "static", innerModel->getNode("floor"), 0, 0, 0, 0, 0, 0);
	//innerModel->getNode("floor")->addChild(t);
	//InnerModelMesh *m = innerModel->newMesh ("baseFake", t, "/home/robocomp/robocomp/files/osgModels/robex/robex.ive", 1000, 0, 0, 0, -181, 0, 0, 0);
	//t->addChild(m);
  

	//Trees (forward and backward) creation
	arbol = new tree<QVec>;
	arbolGoal = new tree<QVec>;
	PATH_FOUND = false;
	MAX_ITER = 5000;
}

/**
* \brief computes a path to a target from current position of robot. Is called from outside whenever a new path to target is required
* @param target point in 3D space
*/
bool Planner::computePath(const QVec &target, InnerModel *inner)
{	
	qDebug() << __FILE__ << __FUNCTION__ << "Starting planning with target" << target;
	
	QVec currentTarget = target;  //local variable to get samples from free space
   
	//We need to resynchronize here because in subsequent call the robot will be "Dios sabe dónde"
	QVec robot = inner->transform("world", "robot");	
	float rYaw = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), 0, rYaw, 0);
	
	//If target on obstacle, abort
	if (collisionDetector(target,0,innerModel) == true)
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Robot collides in target. Aborting planner";
		PATH_FOUND = false;
		return false;
	}

	//Init random sequence generator
	qsrand ( QTime::currentTime().msec() );
	
	//Clean trees
 	arbol->clear();
	arbolGoal->clear();

	//Handy iterators
	tree<QVec>::iterator top, nodeCurrentPos, auxNode, auxNodeGoal;
	tree<QVec>::iterator topGoal, nodeCurrentPosGoal;

	//Initialize trees
	top = arbol->begin();
	topGoal =arbolGoal->begin();
	nodeCurrentPos = arbol->insert(top, robot);
	nodeCurrentPosGoal = arbolGoal->insert(topGoal, target);
 	CURRENT_LEAF_GOAL = nodeCurrentPosGoal;
 	CURRENT_LEAF = nodeCurrentPos;

	QVec res(3), resGoal(3);
	int i;
	bool reachEnd, success=false;
	int swapCounter = 0;

	for(i=0; i< MAX_ITER; i++)
	{
		//get new point from random sequence. First point is always the target
		if( i>0)
		{
			currentTarget = sampleFreeSpaceR2(target, innerModel);
		}

		// find closest point from random point to tree
		nodeCurrentPos = findClosestPointInTree( arbol, currentTarget);	
		
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNode = nodeCurrentPos;
		res = trySegmentToTarget( *nodeCurrentPos, currentTarget, reachEnd, arbol, auxNode);		
		// Can  only be equal if random point accidentally falls on tree
		if (equal(res, *nodeCurrentPos) == false) //Not achieved goal
			//appends res to tree at auxNode position
			CURRENT_LEAF = arbol->append_child( auxNode, res);			
		else
			CURRENT_LEAF=nodeCurrentPos;
		
		//search for closest point to res un goal tree
		nodeCurrentPosGoal = findClosestPointInTree( arbolGoal, res);		
		//Local navigation from nodeCurrentPosGoal to res using Bezier trajectories. Returns closest point in trajectory
		//resGoal = tryBezierToTarget( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, nodeCurrentPosGoal);
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNodeGoal = nodeCurrentPosGoal;
		resGoal = trySegmentToTarget( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, auxNodeGoal);		

		// if origin and goal are not the same point, add resGoal to tree
 		if (equal(resGoal, *nodeCurrentPosGoal) == false)
			CURRENT_LEAF_GOAL = arbolGoal->append_child( auxNodeGoal, resGoal);		
		else
			CURRENT_LEAF_GOAL=nodeCurrentPosGoal;

			
		// if the result on localnavigation in both trees meet then we have found a way from robot to target
		if (equal(res, resGoal))
		{
			success = true;
			break;	
		}
		
		//Balance trees by swaping pointers when sizes differ.
		if ( (not success) and (arbol->size() > arbolGoal->size()) )
		{
			aux = arbolGoal;
			arbolGoal = arbol;
			arbol = aux;
			tree<QVec>::pre_order_iterator auxLeaf;
  			auxLeaf = CURRENT_LEAF_GOAL;
  			CURRENT_LEAF_GOAL = CURRENT_LEAF;
  			CURRENT_LEAF = auxLeaf;			//if even (par) then trees are in original position
			swapCounter++;
		}
		
		if (i%50 == 0)
			qDebug()  << __FILE__ << __FUNCTION__ << "Planning iteration" << i;
	}
	
	if ( i < MAX_ITER)
	{
		PATH_FOUND = true;
		//If not even we do one more swap to restore trees to original position
		if( swapCounter%2 > 0)
		{
			aux = arbolGoal;
			arbolGoal = arbol;
			arbol = aux;
			tree<QVec>::pre_order_iterator auxLeaf;
			auxLeaf = CURRENT_LEAF_GOAL;
			CURRENT_LEAF_GOAL = CURRENT_LEAF;
			CURRENT_LEAF = auxLeaf;
		}
 		currentPath = recoverPath(arbol,CURRENT_LEAF, arbolGoal,CURRENT_LEAF_GOAL);
		if (currentPath.size()>0)
		{
		    smoothPath(currentPath);
			//smoothPathStochastic(currentPath);
		    //currentSmoothedPath = currentPath;
			return true;
		}
		else
			return false;
	}
	else
	{
		PATH_FOUND = false;
		return false;
	}
}

/**
 * @brief Method to compare poses for equality
 * 
 * @param p1 ...
 * @param p2 ...
 * @return bool
 */
bool Planner::equal(const QVec & p1, const QVec & p2)
{
	if( (p1-p2).norm2() < 50)
		return true;
	else
		return false;
}

	
/**
* @brief Searches the closest point in the current tree to the parameter currentTarget
* 
* @param arb Current tree
* @param currentTarget pose to be searched wrt to the tree
* @return tree< RMat::QVec, std::allocator< tree_node_< RMat::QVec > > >::iterator Iterator to the closest node in the tree
*/
tree<QVec>::iterator Planner::findClosestPointInTree( tree<QVec> * arb, const QVec &currentTarget)
{
	tree<QVec>::pre_order_iterator ini = arb->begin();
	tree<QVec>::pre_order_iterator end = arb->end();
	tree<QVec>::iterator nodeCurrentPos;
	float min = std::numeric_limits<float>::max();
	float dist;
	while(ini!=end)
	{
		dist = (currentTarget-(*ini)).norm2();
		if(  dist < min )
		{
			min = dist;
			nodeCurrentPos = ini;
		}
		++ini;
	}
   return nodeCurrentPos;
}
	
/**
 * @brief Recovers the path between current and currentGoal
 * 
 * @param arbol ...
 * @param _current ...
 * @param arbolGoal ...
 * @param _currentGoal ...
 * @return QList< QVec >
 */
QList<QVec> Planner::recoverPath(tree<QVec> *arbol , const tree<QVec>::pre_order_iterator & _current, tree<QVec> *arbolGoal, const tree<QVec>::pre_order_iterator & _currentGoal)
{
	QList<QVec> path;
	tree<QVec>::pre_order_iterator begin, current;
	
	if ( arbol == NULL or arbol->is_valid(_current) == false )
		  return path; 
	
	begin = arbol->begin();
	current = _current;
	
	path.append(*current);
	while(current!=begin) 
	{
	  current = arbol->parent(current);
	  path.prepend(*current);

	}
	//Second tree
	if ( arbolGoal == NULL or arbolGoal->is_valid(_currentGoal) == false)
	  return path;
	begin = arbolGoal->begin();
	tree<QVec>::pre_order_iterator currentGoal = _currentGoal;

	path.append(*currentGoal);

	while(currentGoal!=begin) 
	{
		currentGoal = arbolGoal->parent(currentGoal);
		path.append(*currentGoal);

	}
	return path;
}

WayPoints Planner::smoothRoad( WayPoints road)
{
	for(int i=0; i< road.size()-1; i++) 
	{
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		float dist = (w.pos-wNext.pos).norm2();		
		if( dist > ROBOT_RADIUS)
		{
			float l = ROBOT_RADIUS/dist;
			WayPoint wNew( (w.pos * (1-l)) + (wNext.pos * l));
			road.insert(i+1,wNew);
		}
	}
	currentSmoothedPath.clear();
	QList<QVec> l;
	foreach(WayPoint w, road)
		l.append(w.pos);

	smoothPath(l);
	
	WayPoints w;
	w.readRoadFromList(currentSmoothedPath);
 	return w;
}

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 * 
 * @param list List of poses comprising the path
 * @return void
 */
void Planner::smoothPath( const QList<QVec> & list)
{
	bool reachEnd;
	tree<QVec>::iterator it;

	trySegmentToTarget( list.first(), list.last(), reachEnd, NULL, it);

	if (reachEnd == true) 
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

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 * 
 * @param list List of poses comprising the path
 * @return void
 */
void Planner::smoothPathStochastic(QList< QVec >& list)
{
	bool reachEnd;
	tree<QVec>::iterator it;
	const int M_ITER = 50;
	
	for(int i=0; i<M_ITER; i++)
	{
		//pick two points in path
		int first = (int)(QVec::uniformVector(1, 0, list.size()-3)[0]);
		int second = (int)(QVec::uniformVector(1,first+1, list.size()-2)[0]);
		//Check if there is a shortcut
		qDebug() << "size" << list.size() << "first" << first << "second" << second;
		trySegmentToTarget( list[first], list[second], reachEnd, NULL, it);	
		if( reachEnd == true)  //remove detour
		{
			qDebug() << "cortando desde " << first+1 << "hasta" << second;
			for(int j=first+1; j<second; j++)
				list.removeAt(first+1);
		}
		if(list.size() < 3)
			break;
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

QVec Planner::sampleFreeSpaceR2(const QVec &currentTarget,  InnerModel *inner)
{
	bool collision = true;
	float range = 8000.f / RAND_MAX;
	QVec p(3,0.f);
	
	//Inject goal position to bias the distribution
	if ( qrand()*100.f/RAND_MAX > 80)
		return(currentTarget);
	
	while( collision == true )
	{
		p[0] =  range * rand() -3500;
		p[2] =  range * rand() -3500;
		collision = collisionDetector(p, 0, inner);	
	}
	return p;
}

////////////////////////////////////////////////////////////////////////
/// COLLISION DETECTOR - FCL -
////////////////////////////////////////////////////////////////////////


bool Planner::collisionDetector(const QVec position, const double alpha, InnerModel *im)
{
	std::vector<QString> robotNodes;
	std::vector<QString> restNodes;
	im->updateTransformValues("robot", position.x(), position.y(), position.z(), 0, alpha, 0);
	
// 	printf("RECURSIVE MESHES\n");
	recursiveIncludeMeshes(im->getRoot(), "robot", false, robotNodes, restNodes);
/*	
	printf("robot: ");
	for (uint i=0; i<robotNodes.size(); i++)
		printf("%s ", robotNodes[i].toStdString().c_str());
	printf("\n");
	
	printf("rest: ");
	for (uint i=0; i<restNodes.size(); i++)
		printf("%s ", restNodes[i].toStdString().c_str());
	printf("\n");
*/

	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		printf("%s:", robotNodes[in].toStdString().c_str());
		im->getNode(robotNodes[in])->collisionObject->computeAABB();
		fcl::AABB aabb = im->getNode(robotNodes[in])->collisionObject->getAABB();
		fcl::Vec3f v1 = aabb.center();
// 		printf("--  (%f,  %f,  %f) [%f , %f , %f]", v1[0], v1[1], v1[2], aabb.width(), aabb.height(), aabb.depth());
// 		printf("\n");
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			printf("%s ", restNodes[out].toStdString().c_str());
			if (im->collide(robotNodes[in], restNodes[out]))
			{
				printf("\ncolision:   %s <--> %s\n", robotNodes[in].toStdString().c_str(), restNodes[out].toStdString().c_str());
				return true;
			}
		}
	}

	return false;
}

void Planner::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
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

////////////////////////////////////////////////////////////////////////
/// LOCAL CONTROLLER. Also a lot of improvements fit here
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
QVec Planner::trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec> * arbol , tree<QVec>::iterator & nodeCurrentPos)
{
	float stepSize = 100.f; //100 mms chunks
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
		if (collisionDetector(point, 0, innerModel) == true)
		{
		  reachEnd = false;
		  return pointAnt;
		}
		else
		  if( arbol != NULL )
			nodeCurrentPos = arbol->append_child( nodeCurrentPos, point);			
		 
		landa = landa + step;
		pointAnt = point;
	}
	reachEnd= true;
	return target;
}

//////////////////////////////////////////////
////  Tree Drawing
///////////////////////////////////////////////

void Planner::drawTree(InnerModelManagerPrx innermodelmanager_proxy)
{
	tree<QVec>::pre_order_iterator begin = arbol->begin();
	tree<QVec>::pre_order_iterator end = arbol->end();
	tree<QVec>::pre_order_iterator padre;
		
	/*if (finalTarget.isEmpty() == false)
		world->drawEllipse(QPointF(this->finalTarget.x(), this->finalTarget.z()), 40, 40 , Qt::green, true);
	*/	
	QString name;
	int i=0;
	RoboCompInnerModelManager::Plane3D p;
	while(begin!=end) 
	{
		padre = arbol->parent(begin);
		if(arbol->is_valid(padre))
		{
			//world->drawLine(QLine((*padre).x(), (*padre).z(),(*begin).x(),(*begin).z()), Qt::blue, 20);
			name = "t_" + QString::number(i++);
			float len = (QVec::vec3((*padre).x(), 600, (*padre).z()) -  QVec::vec3((*begin).x(), 600, (*begin).z())).norm2() ;
			QVec midPoint = (QVec::vec3((*padre).x(), 600, (*padre).z()) + QVec::vec3((*begin).x(), 600, (*begin).z()) )*(T)0.5; ;
			p.px = midPoint.x(); p.py = midPoint.y();	p.pz = midPoint.z();
			QLine2D line(QVec::vec3((*padre).x(), 600, (*padre).z()),QVec::vec3((*begin).x(), 600, (*begin).z()));
			QVec normal = line.getNormalForOSGLineDraw();
			p.nx = normal.x();	p.ny = normal.y();	p.nz = normal.z();
			p.height = 20;	p.width = len;	p.thickness = 20;
			p.texture = "#0000FF";
			RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, name, "floor", p);
		}
		++begin;
	}
}
