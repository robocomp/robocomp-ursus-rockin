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
	recursiveIncludeMeshes( clonModel->getRoot(), "robot", false, robotNodes, restNodes);
}

void Localizer::localize(const RoboCompLaser::TLaserData &laser, InnerModel *inner)
{
	
	QVec point = inner->transform("world","robot");
	float alfa = inner->getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	QVec angleList(1,0.f);
// 	for( int i=0; i<10; i++)
// 	{
// 		angleList[i] = laser[i].angle;
// 	}
	
	renderLaser( point , alfa, angleList);
	
}

void Localizer::renderLaser(const QVec &point, float alfa, const QVec &angleList)
{
	const float MAX_LENGTH_ALONG_RAY = 4000;    //GET FROM VISTUAL LASER ESPECIFICATION	
	RoboCompLaser::TLaserData virtualLaser(angleList.size());
	//move robot to position
	clonModel->updateTransformValues("robot", point.x(), point.y(), point.z(), 0, alfa, 0);

	QMat r1q = clonModel->getRotationMatrixTo("world", "laser") ;	
	QVec laser = clonModel->getTranslationVectorTo("world","laser");
	fcl::Vec3f T1( laser(0), laser(1), laser(2) );
		
	for(uint32_t i=0; i<angleList.size(); i++)
	{
		//Create laserLine as an FCL CollisionObject made of a long thin Triangle
		fcl::Vec3f a(laser.x(), laser.y(), laser.z());		// peak of the triangle
		int hitDistance = 0;
		bool hit = false;
		
		while( (hit == false ) and (hitDistance < MAX_LENGTH_ALONG_RAY))
		{
			//stretch the stick
			hitDistance += 10;
			boost::shared_ptr<fcl::Box> laserBox( new fcl::Box( 5, 5, hitDistance));	
			r1q = r1q * RMat::Rot3DOY(angleList[i]);
			fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
			fcl::CollisionObject laserBoxCol(laserBox, R1, T1);
			
			for (int32_t out=0; out<restNodes.size(); out++)
			{
// 				hit = clonModel->collide(restNodes[out], &laserBoxCol );
			}
		}
		//fill the new laser
		virtualLaser[i].dist = hitDistance;
		virtualLaser[i].angle = angleList[i];
		qDebug() << "VLaser:" << virtualLaser[0].dist << virtualLaser[0].angle;
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

/*		float angToGo = 10./hitDistance;
			B = clonModel->transform("world",QVec::vec3(-hitDistance*sin(angToGo+angToGo*0.5), 0, hitDistance*cos(angToGo+angToGo*0.5)) ,"laser");
			C = clonModel->transform("world",QVec::vec3(-hitDistance*sin(angToGo-angToGo*0.5), 0, hitDistance*cos(angToGo-angToGo*0.5)) ,"laser");
			fcl::Vec3f b(B.x(),B.y(),B.z());
			fcl::Vec3f c(C.x(),C.y(),C.z());
	*/		//boost::shared_ptr<fcl::TriangleP> laserTrg( new fcl::TriangleP( a, b, c));	
	
