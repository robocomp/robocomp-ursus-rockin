/*
 * Copyright 2014 pbustos <email>
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

#include "localizer.h"

Localizer::Localizer(InnerModel *inner)
{
	
	clonModel = new InnerModel( *inner );
}

void Localizer::localize(const RoboCompLaser::TLaserData &laser, InnerModel *inner)
{
	
	QVec point = inner->transform("world","robot");
	float alfa = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	QVec angleList(10);
	for( int i=0; i<10; i++)
	{
		angleList[i] = laser[i].angle;
	}
	renderLaser( point , alfa, angleList);
	
}

void Localizer::renderLaser(const QVec &point, float alfa, const QVec &angleList)
{
	const float MAX_LENGTH_ALONG_RAY = 4000;    //GET FROM VISTUAL LASER ESPECIFICATION
	
	std::vector<QString> restNodes;
	std::vector<QString> robotNodes;
	RoboCompLaser::TLaserData virtualLaser(angleList.size());
	
	//move robot to position
	clonModel->updateTransformValues("robot", point.x(), point.y(), point.z(), 0, alfa, 0);
	
	recursiveIncludeMeshes( clonModel->getRoot(), "robot", false, robotNodes, restNodes);
	
	for(uint32_t i=0; i<angleList.size(); i++)
	{
		//QLine2D dirVectorAlongLaserLine(clonModel->transform("world","laser"), QVec::vec3(sin(angleList[i]*1000, 0, cos(angleList[i]*1000));
		int hitDistance = 0;
		bool hit = false;
	
		while( (hit == false ) and (hitDistance < MAX_LENGTH_ALONG_RAY))
		{
			//stretch the stick
			hitDistance += 10;
			for (uint32_t out=0; out<restNodes.size(); out++)
			{
				hit = clonModel->collide("laserLine", restNodes[out] );
			}
		}
		//fill the new laser
		virtualLaser[i].dist = hitDistance;
		virtualLaser[i].angle = angleList[i];
	}
}

void Localizer::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
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
		//printf("collidable: %s\n", node->id.toStdString().c_str());
		if (inside)
		{
			//in.push_back(node->id);
		}
		else
		{
			out.push_back(node->id);
		}
	}
}


