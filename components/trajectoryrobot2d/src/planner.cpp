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
	//Clone innermodel
	innerModel = new InnerModel(innerModel_);
	MAX_ITER = MAX_ITER_INIT;
	
	//Init random sequence generator
	qsrand ( QTime::currentTime().msec() );

	//Init initializeCache
	initializeWayPointsCache();
	initializeRandomR2Cache();

	//Trees (forwardand backward) creation
	arbol = new tree<QVec>;
	arbolGoal = new tree<QVec>;
	PATH_FOUND = false;
}

/**
* \brief computes a path to a target from current position of robot. Is called from outside whenever a new path to target is required
* @param target point in 3D space
*/
bool Planner::computePath(const QVec &target, InnerModel *inner)
{	
	//static QTime reloj = QTime::currentTime();
	
	qDebug() << __FILE__ << __FUNCTION__ << "Starting planning with target at:" << target << "and robot at:" << inner->transform("world","robot");
	
	QVec currentTarget = target;  //local variable to get samples from free space
   
	//We need to resynchronize here because in subsequent call the robot will be "Dios sabe dónde"
	QVec robot = inner->transform("world", "robot");	
	QVec robotRotation = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min();
	innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());

	QList<QVec> currentPath;   			//Results will be saved here
	currentSmoothedPath.clear();
	
	//Build list of colision meshes
	robotNodes.clear(); restNodes.clear();
	recursiveIncludeMeshes(innerModel->getRoot(), "robot", false, robotNodes, restNodes);
		
	//If target on obstacle, abort.  OJO targetRotation is not specified, Using robot initial orientation
	if (collisionDetector(target,robotRotation,innerModel) == true)
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
		PATH_FOUND = false;
		return false;
	}
		
	
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
		currentTarget = sampleFreeSpaceR2(target, innerModel);
		
		//qDebug() << "in sampleFreeSpaceR2: " << reloj.restart();
		// find closest point from random point to tree
		nodeCurrentPos = findClosestPointInTree( arbol, currentTarget);	
		
		//qDebug() << "findClosestPointInTree: " << reloj.restart();
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNode = nodeCurrentPos;
		res = trySegmentToTarget( *nodeCurrentPos, currentTarget, reachEnd, arbol, auxNode);
		//qDebug() << "trySegmentToTarget First Tree: " << reloj.restart();
		//res = trySegmentToTargetBinarySearch( *nodeCurrentPos, currentTarget, reachEnd, arbol, auxNode);		
		// Can  only be equal if random point accidentally falls on tree
		if (equal(res, *nodeCurrentPos) == false) //Not achieved goal
			//appends res to tree at auxNode position
			CURRENT_LEAF = arbol->append_child( auxNode, res);			
		else
			CURRENT_LEAF=nodeCurrentPos;
		
		//search for closest point to res un goal tree
		nodeCurrentPosGoal = findClosestPointInTree( arbolGoal, res);		
		//qDebug() << "findClosestPointInTree Second Tree: " << reloj.restart();
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNodeGoal = nodeCurrentPosGoal;
		resGoal = trySegmentToTarget( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, auxNodeGoal);		
		//qDebug() << "trySegmentToTarget Second Tree: " << reloj.restart();
		//resGoal = trySegmentToTargetBinarySearch( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, auxNodeGoal);		

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
			qDebug() << __FILE__ << __FUNCTION__ << "Starting smother with a plan of " << currentPath.size() << "elements";
		    smoothPath(currentPath);
			storePlanInCache(currentPath);
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
 * @brief Stores good plans as in Bruce & Velose paper
 * 
 * @param currentPath ...
 * @return void
 */
void Planner::storePlanInCache(QList< QVec > currentPath)
{
	//insert
	for(int i=0; i<currentPath.size(); i++)
	{
		int a = qrand() * MAX_WAYPOINT_CACHE / RAND_MAX; 
		wayPointCache[a] = currentPath[i];
	}
}

void Planner::initializeWayPointsCache()
{
	for(int i=0; i<MAX_WAYPOINT_CACHE; i++)
	{
		wayPointCache.append(QVec::vec3( qrand()*10000.f/RAND_MAX, 0, -qrand()*10000.f/RAND_MAX));
	}
}

void Planner::initializeRandomR2Cache()
{
	randomR2Cache.resize(MAX_RANDOM_R2_CACHE);
	for(int i=0; i<MAX_RANDOM_R2_CACHE; i++)
	{
		randomR2Cache[i] = QVec::vec3( qrand()*10000.f/RAND_MAX, 0, -qrand()*10000.f/RAND_MAX );
	}
}

QVec Planner::getPointFromRandomR2Cache()
{
	static int i=0;
	
	if(i == MAX_RANDOM_R2_CACHE )
	{
		initializeRandomR2Cache();
		i=0;
	}
	return randomR2Cache[i++];
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
	static bool firstTime = true;
	float probTarget = 10;
	float probCache = 40;
	
// 	const float widthX = 10000;
//  	const float widthZ = -10000;
 	const QVec zeros(3,0.f);
 	bool collision = true;
// 	float rangeX = widthX / RAND_MAX;
// 	float rangeZ = widthZ / RAND_MAX;
// 	
	QVec p(3,0.f);

	float prob = qrand()*100.f/RAND_MAX;
	
	if( firstTime == true)
	{
		return currentTarget;
		firstTime = false;
	}
	
	if( prob < probTarget) 
		return currentTarget;
	
	if (prob >= probTarget and prob < probTarget+probCache) 
	{	
		int a = floor(qrand() * (wayPointCache.size()-1) / RAND_MAX);
		return wayPointCache[a];
	}
	
	while( collision == true )
	{
		p = getPointFromRandomR2Cache();	
		collision = collisionDetector(p, zeros, inner);	
	}
	return p;
}

////////////////////////////////////////////////////////////////////////
/// COLLISION DETECTOR - FCL -
////////////////////////////////////////////////////////////////////////


bool Planner::collisionDetector(const QVec &position, const QVec &rotation, InnerModel *im)
{
	im->updateTransformValues("robot", position.x(), position.y(), position.z(), rotation.x(), rotation.y(), rotation.z());
	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			if (im->collide(robotNodes[in], restNodes[out]))
			{
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
QVec Planner::trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec> * arbol , tree<QVec>::iterator & nodeCurrentPos)
{
	//return trySegmentToTargetBinarySearch(origin, target, reachEnd, arbol, nodeCurrentPos);
	
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
		//qDebug() << point << origin << target << innerModel->transform("world","robot");
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

/**
 * @brief Version using binary search
 * 
 * @param origin ...
 * @param target ...
 * @param reachEnd ...
 * @param arbol ...
 * @param nodeCurrentPos ...
 * @return RMat::QVec
 */
QVec Planner::trySegmentToTargetBinarySearch(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec> * arbol , tree<QVec>::iterator & nodeCurrentPos)
{
	const float MAX_LENGTH_ALONG_RAY = (target-origin).norm2();
	bool hit = false;
	QVec finalPoint;
	float wRob=600, hRob=1600;

	if( MAX_LENGTH_ALONG_RAY < 50) 
	{
		qDebug() << __FUNCTION__ << "target y origin too close";
		return target;
	}
		
	//Compute angle between origin-target line and world Zaxis
	float alfa1 = QLine2D(target,origin).getAngleWithZAxis();
	
	// Update robot's position and align it with alfa1 so it looks at the TARGET point 	
	innerModel->updateTransformValues("robot", origin.x(), origin.y(), origin.z(), 0., alfa1, 0.);
	
	// Compute rotation matrix between robot and world. Should be the same as alfa
	QMat r1q = innerModel->getRotationMatrixTo("world", "robot");	
	
	// Create a tall box for robot body with center at zero and sides:
	boost::shared_ptr<fcl::Box> robotBox(new fcl::Box(wRob, hRob, wRob));
	
	// Create a collision object
	fcl::CollisionObject robotBoxCol(robotBox);
	
	//Create and fcl rotation matrix to orient the box with the robot
	const fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
		
	//Check collision at maximum distance
	float hitDistance = MAX_LENGTH_ALONG_RAY;
	
	//Resize big box to enlarge it along the ray direction
	robotBox->side = fcl::Vec3f(wRob, hRob, hitDistance);
		
	//Compute the coord of the tip of a "nose" going away from the robot (Z dir) up to hitDistance/2
	const QVec boxBack = innerModel->transform("world", QVec::vec3(0, hRob/2, hitDistance/2.), "robot");
	
	//move the big box so it is aligned with the robot and placed along the nose
	robotBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
		
	//qDebug() << "checking ang" << r1q.extractAnglesR_min().y() << "and size " << boxBack;
	
	//Check collision of the box with the world
	for (uint out=0; out<restNodes.size(); out++)
	{
		hit = innerModel->collide(restNodes[out], &robotBoxCol);
		if (hit) break;
	}	
	
	//Binary search. If not free way do a binary search
	if (hit)
	{	
		hit = false;
		float min=0;
		float max=MAX_LENGTH_ALONG_RAY;
		
		while (max-min>10)
		{
			//set hitDistance half way
			hitDistance = (max+min)/2.;
			// Stretch and create the stick
			robotBox->side = fcl::Vec3f(wRob,hRob,hitDistance);
			const QVec boxBack = innerModel->transform("world", QVec::vec3(0, hRob/2, hitDistance/2.), "robot");
			robotBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
			
			//qDebug() << "checking ang" << r1q.extractAnglesR_min().y() << "and size " << boxBack << hitDistance;

			// Check collision using current ray length
			for (uint out=0; out<restNodes.size(); out++)
			{
				hit = innerModel->collide(restNodes[out], &robotBoxCol);
				if (hit)
					break;
			}
			
			// Manage next min-max range
			if (hit)
				max = hitDistance;
			else
				min = hitDistance;
		}
		// Set final hit distance
		hitDistance = (max+min)/2.;
		
		reachEnd = false;
		if( hitDistance < 50) 
			return origin;
		
		finalPoint = innerModel->transform("world", QVec::vec3(0, 0, hitDistance-10), "robot");
		//nodeCurrentPos = arbol->append_child( nodeCurrentPos, innerModel->transform("world", QVec::vec3(0, 0, hitDistance/2), "robot")); 
		//nodeCurrentPos = arbol->append_child( nodeCurrentPos, finalPoint);			
		//qDebug() << "No.." << finalPoint;
		return finalPoint;
	}
	else  //we made it up to the Target!
	{
		reachEnd = true;
		//qDebug() << "Yes.." << target;
		return target;
	}
	// also we might need to insert a few nodes in the tree covering from origin to hitDistance
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
