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

void Sampler::initialize(InnerModel &inner, const QRectF& outerRegion_, const QList< QRectF> &innerRegions_)
{
	innerModel = inner;
	innerRegions = innerRegions_;
	outerRegion = outerRegion_;
	robotNodes.clear(); restNodes.clear();
	recursiveIncludeMeshes(inner.getRoot(), "robot", false, robotNodes, restNodes);
	
	//Init random sequence generator
	qsrand( QTime::currentTime().msec() );
	
}
/**
* @brief Picks a random point of the list created by the sampler and checks that it is out of obstacles (Free Space)
* 
* @param currentTarget Current target provided to bias the sample. 20% of the times the target is returned.
* @return RMat::QVec
*/
QVec Sampler::sampleFreeSpaceR2()  //ÑAPA!!!! meter los cuatro valores 
{
 	bool validState = false;
	QVec p;

	while( validState == false )
	{
		p =	QVec::vec3( qrand() * outerRegion.right() / RAND_MAX, 0, qrand() * outerRegion.bottom() / RAND_MAX );  //ÑAPA BRUTAL
		QPointF s(p.x(),p.z());
		bool in = false;
		foreach(QRectF q, innerRegions)
			if( q.contains(s))
			{
				in = true;
				break;
			}
		if( in == false) 
			validState = checkRobotValidStateAtTarget(p);
	}
	return p;
}

//Does not return IM to its original state
bool Sampler::checkRobotValidStateAtTarget(const QVec &targetPos, const QVec &targetRot) 
{
	innerModel.updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());
	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			//if (inner->collide(robotNodes[in], restNodes[out]))
			if (innerModel.collide(robotNodes[in], restNodes[out]))
			{
				return false;
			}
		}
	}
	return true;
}

bool Sampler::isStateValid(const ompl::base::State *state) 
{
	const float x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
	const float z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
	
	innerModel.updateTransformValues("robot", x, 0, z, 0, 0, 0);
	
	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			if (innerModel.collide(robotNodes[in], restNodes[out]))
			{
				return false;
			}
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
// PRIVATE
///////////////////////////////////////////////////////////////////////////////////////////

void Sampler::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
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
			break;
	}
	return true;
}



// bool Sampler::checkRobotValidStateReturnIMToOriginal(InnerModel* innerModel, QVec target)  //Does not return IM to its original state
// {
// 	QVec tr = innerModel->transform(robotName,worldName);
// 	innerModel->updateTranslationValues("robot", target.x(), target.y(), target.z());
// 	for (uint32_t in=0; in<robotNodes.size(); in++)
// 	{
// 		for (uint32_t out=0; out<restNodes.size(); out++)
// 		{
// 			if (innerModel->collide(robotNodes[in], restNodes[out]))
// 			{	
// 				innerModel->updateTranslationValues(robotName, tr.x(), tr.y(), tr.z());
// 				return true;
// 			}
// 		}
// 	}
// 	innerModel->updateTranslationValues(robotName, tr.x(), tr.y(), tr.z());
// 	return false;
// }
