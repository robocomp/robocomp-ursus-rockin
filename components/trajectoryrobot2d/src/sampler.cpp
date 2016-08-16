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

#include "sampler.h"

Sampler::Sampler()
{
}

void Sampler::initialize(InnerModel *inner, const QRectF& outerRegion_, const QList< QRectF> &innerRegions_)
{
	innerModel = inner;
	innerRegions = innerRegions_;
	outerRegion = outerRegion_;

	robotNodes.clear(); restNodes.clear(); 
	excludedNodes.insert("floor_plane");
	recursiveIncludeMeshes(inner->getRoot(), "robot", false, robotNodes, restNodes, excludedNodes);
	
	//Init random sequence generator
	qsrand( QTime::currentTime().msec() );
	
}
/**
* @brief Picks a random point within the limits of outerRegion and innerRegions and checks that it is out of obstacles (Free Space)
* 
* @return RMat::QVec, a 3D vector with 0 in the Y coordinate
*/
QList<QVec> Sampler::sampleFreeSpaceR2(uint nPoints)  
{
 	bool validState = false;
	QVec p,q,res(3,0.f);
	QList<QVec> list;
	
	
	printf(" %f %f\n", outerRegion.left(), outerRegion.right());
	printf(" %f %f\n", outerRegion.top(), outerRegion.bottom());
	
	for(uint32_t i=0; i<nPoints;i++)
	{
		while( validState == false )   ///CHECK ALSO FOR TIMEOUT
		{
			p =	QVec::uniformVector(1, outerRegion.left(), outerRegion.right());
			q =	QVec::uniformVector(1, outerRegion.top(), outerRegion.bottom());
			QPointF s(p.x(),q.x());
			bool in = false;
			foreach(QRectF rect, innerRegions)
			{
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			}
			if (in == false) 
			{
				res[0] = p.x();
				res[2] = q.x();
				validState = checkRobotValidStateAtTarget(res);
			}
		}
		res = innerModel->transform("floor_plane", res, "root");
		list.append(res);
		validState = false;
	}
	return list;
}

/**
* @brief Picks a (list of) random point within the limits of a given box and checks that it is out of obstacles (Free Space)
* 
* @return RMat::QVec, a 3D vector with 0 in the Y coordinate
*/
QList<QVec> Sampler::sampleFreeSpaceR2Uniform( const QRectF &box, uint32_t nPoints)  
{
 	bool validState = false;
	QVec p,q, res(3,0.f);
	QList<QVec> list;

	for(uint32_t i=0; i<nPoints;i++)
	{
		while( validState == false )
		{
			p =	QVec::uniformVector(1,box.left(), box.right());
			q =	QVec::uniformVector(1,box.top(), box.bottom());
			QPointF s(p.x(),q.x());
			bool in = false;
			foreach(QRectF rect, innerRegions)
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			if( in == false) 
			{
				res[0] = p.x(); res[2] = q.x();
				validState = checkRobotValidStateAtTarget(res);
			}
		}
		list.append(res);
		validState = false;
	}
	return list;
}


/**
* @brief Picks a (list of) random point within the limits of a given box and checks that it is out of obstacles (Free Space)
* 
* @return RMat::QVec, a 3D vector with 0 in the Y coordinate
*/
QList<QVec> Sampler::sampleFreeSpaceR2Gaussian(float meanX, float meanY, float sigma1, float sigma2, uint32_t nPoints)  
{
 	bool validState = false;
	QVec p,q;
	QList<QVec> list;

	for(uint32_t i=0; i<nPoints;i++)
	{
		while( validState == false )
		{
			p =	QVec::gaussianSamples(1, meanX, sigma1);
			q =	QVec::gaussianSamples(1, meanY, sigma2);
			QPointF s(p.x(),q.x());
			qDebug() << s;
			bool in = false;
			foreach(QRectF rect, innerRegions)
				if( rect.contains(s))
				{
					in = true;
					break;
				}
			if( in == false) 
				validState = checkRobotValidStateAtTarget(p);
		}
		list.append(QVec::vec3(p.x(),0.f,q.x()));
	}
	return list;
}

/**
 * @brief  CHECK ROBOT VALID STATE AT TARGET
 * Comprueba el estado del robot (si ha chocado con algo o no). 
 * 
 * @param targetPos posicion del robot en el MUNDO (componentes de traslacion).
 * @param targetRot posicion del robot en el MUNDO (componentes de rotacion).
 * 
 * @return bool: - TRUE: cuando las mallas del robot no chocan con las mallas de
 * 				         cualquier objeto del mundo.
 * 				 - FALSE: cuando alguna malla del robot choca con alguna malla
 * 						  de cualquier objeto del mundo.
 */ 
//Does not return IM to its original state
bool Sampler::checkRobotValidStateAtTarget(const QVec &targetPos, const QVec &targetRot) const 
{
	innerModel->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());
	
	for ( auto in : robotNodes )
		for ( auto out : restNodes )
			if ( innerModel->collide( in, out))
			{
				//qDebug() << __FUNCTION__ << "collision de " << in << " con " << out;
				return false;
			}

	return true;
}

bool Sampler::isStateValid(const ompl::base::State *state) 
{
	const float x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
	const float z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
	
	innerModel->updateTransformValues("robot", x, 0, z, 0, 0, 0);
	
	for (uint32_t in=0; in<robotNodes.size(); in++)
		for (uint32_t out=0; out<restNodes.size(); out++)
			if (innerModel->collide(robotNodes[in], restNodes[out]))
			{
				return false;
			}
	return true;
}

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
bool Sampler::checkRobotValidDirectionToTarget(const QVec & origin , const QVec & target, QVec &lastPoint)
{
	//return trySegmentToTargetBinarySearch(origin, target, reachEnd, arbol, nodeCurrentPos);
	
	float stepSize = 100.f; //100 mms chunks  SHOULD BE RELATED TO THE ACTUAL SIZE OF THE ROBOT!!!!!
	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
	float step;
	
	//if too close return target
	if (nSteps == 0) 
	{
		lastPoint = target;
		return false;
	}
	step = 1./nSteps;

	//go along visual ray connecting robot pose and target pos in world coordinates. l*robot + (1-r)*roiPos = 0
	QVec point(3);
	float landa = step;
	
	lastPoint = origin;
	for(uint i=1 ; i<=nSteps; i++)
	{
		point = (origin * (1-landa)) + (target * landa);
		if (checkRobotValidStateAtTarget(point) ) 
		{
			lastPoint  = point;
			landa = landa + step;
		}
		else
			return false;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// PRIVATE
///////////////////////////////////////////////////////////////////////////////////////////

void Sampler::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded)
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
			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);
		}
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )			
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
}


bool Sampler::checkRobotValidDirectionToTargetBinarySearch(const QVec & origin , const QVec & target, QVec &lastPoint) const
{
	const float MAX_LENGTH_ALONG_RAY = (target-origin).norm2();
	bool hit = false;
	QVec finalPoint;
	float wRob=600, hRob=1600;  //GET FROM INNERMODEL!!!

	
	if( MAX_LENGTH_ALONG_RAY < 50)   //FRACTION OF ROBOT SIZE
	{
		qDebug() << __FUNCTION__ << "target y origin too close";
		lastPoint = target;
		return false;
	}
		
	//Compute angle between origin-target line and world Zaxis
	float alfa1 = QLine2D(target,origin).getAngleWithZAxis();
	//qDebug() << "Angle with Z axis" << origin << target << alfa1;
	
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
		
		if( hitDistance < 50) 
			lastPoint = origin;
		else
			lastPoint = innerModel->transform("world", QVec::vec3(0, 0, hitDistance-10), "robot");
		return false;;
	}
	else  //we made it up to the Target!
	{
		lastPoint = target;
		return true;
	}
}

/**
 * @brief Checks is there is a valid straight tunnel from origin to target the size of the robot
 * 
 * @param origin ...
 * @param target ...
 * @return bool
 */

bool Sampler::checkRobotValidDirectionToTargetOneShot(const QVec & origin , const QVec & target) const
{
	//qDebug() << __FUNCTION__ << "Checking between: " << origin << "and " << target;
	
	const float MAX_LENGTH_ALONG_RAY = (target-origin).norm2();
	QVec finalPoint;
	float wRob=420, hRob=1600;  //GET FROM INNERMODEL!!! 
	
// 	if( MAX_LENGTH_ALONG_RAY < 50)   //COMMENT THIS FOR NOW ::::::::::::::::::::...
// 	{
// 		qDebug() << __FUNCTION__ << "target y origin too close";
// 		return false;
// 	}
		
	//Compute angle between origin-target line and world Zaxis
	float alfa1 = QLine2D(target,origin).getAngleWithZAxis();
	//qDebug() << "Angle with Z axis" << origin << target << alfa1;
	
	// Update robot's position and align it with alfa1 so it looks at the TARGET point 	
	innerModel->updateTransformValues("robot", origin.x(), origin.y(), origin.z(), 0., alfa1, 0.);
	
	// Compute rotation matrix between robot and world. Should be the same as alfa
	QMat r1q = innerModel->getRotationMatrixTo("world", "robot");
	
	//qDebug()<< "alfa1" << alfa1 << r1q.extractAnglesR_min().y() << "robot" << innerModel->transform("world","robot"); 
	
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
	const QVec boxBack = innerModel->transform("world", QVec::vec3(0, hRob/2, hitDistance/2), "robot");
	
	//move the big box so it is aligned with the robot and placed along the nose
	robotBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
		
	//Check collision of the box with the world
	for (auto it : restNodes)
	{
		if ( innerModel->collide(it, &robotBoxCol))
		{
			//qDebug() << __FUNCTION__ << ": Robot collides with " << it;
			return false;
		}
	}
	return true;
}

bool Sampler::searchRobotValidStateCloseToTarget(QVec& target)
{
	//If current is good, return
	if( checkRobotValidStateAtTarget(target) == true)
		return true;
	
	target.print("target");
	//Start searching radially from target to origin and adding the vertices of a n regular polygon of radius 1000 and center "target"
	const int nVertices = 12;
	const float radius = 1000.f;
	QVec lastPoint, minVertex, vertex;
	float fi,vert;
	float minDist = radius;
	
	for(int i=0; i< nVertices; i++)
	{
		fi = (2.f*M_PI/nVertices) * i;
		int k;
		bool free;
		for(k=100; k<radius; k=k+100)
		{
			vertex = QVec::vec3(target.x() + k*sin(fi), target.y(), target.z() + k*cos(fi));
			free = checkRobotValidStateAtTarget(vertex);
			if (free == true) 
				break;
		}
		if( free and k < minDist )
		{
			minVertex = vertex;
			minDist = k;	
			vert = fi;
		}
	}
	if( minDist < radius)
	{
		target = minVertex;
		target.print("new target");
		qDebug() << minDist << vert;
		return true;
	}
	else
		return false;
}
