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

	icp.setDefault();
	
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
	
	
	laserRender( point , alfa, angleList);
	
	//estimatePoseWithICP(laser, point, alfa);
	
}

void Localizer::laserRender(const QVec &point, float alfa, const QVec &angleList)
{
	// TODO: GET FROM VISTUAL LASER SPECIFICATION	
	const float MAX_LENGTH_ALONG_RAY = 50000;
	RoboCompLaser::TLaserData virtualLaser(angleList.size());

	// Update robot's position
	clonModel->updateTransformValues("robot", point.x(), point.y(), point.z(), 0., alfa, 0.);

	QMat r1q1 = clonModel->getRotationMatrixTo("world", "laser");

	for (int i=0; i<angleList.size(); i++)
	{
		// Create laserLine as an FCL CollisionObject made of a fcl::Box
		int hitDistance = 0;
		bool hit = false;
		
		const QMat r1q = r1q1 * RMat::Rot3DOY(angleList[i]);
		const fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );

		while (hit==false and hitDistance<MAX_LENGTH_ALONG_RAY)
		{
			// Stretch and create the stick
			hitDistance += 10;
			boost::shared_ptr<fcl::Box> laserBox(new fcl::Box(10, 10, hitDistance));
			fcl::CollisionObject laserBoxCol(laserBox);

			const QVec boxBack = clonModel->transform("world", QVec::vec3(0, 0, hitDistance/2.), "laser");
			laserBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
			
			for (uint out=0; out<restNodes.size(); out++)
			{
 				hit = clonModel->collide(restNodes[out], &laserBoxCol);
				if (hit)
				{
					break;
				}
			}
		}
		// Fill the laser's structure
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
	

void Localizer::estimatePoseWithICP(const RoboCompLaser::TLaserData &laserData, const QVec &robotT, float ang)
{
	static bool firstTime = true;
	float x,y;
	
	//CHECK IF SOME DISPLACEMENT HAS OCCURED
	
	refM.resize(3,laserData.size());
	dataM.resize(3,laserData.size());
	
	for(uint i=0; i< laserData.size(); i++)
	{
		x = laserData[i].dist * sin( laserData[i].angle );
		y = laserData[i].dist * cos( laserData[i].angle );
		if (firstTime == true)
		{
			refM(0,i) = x;
			refM(1,i) = y;
			refM(2,1) = 1.f;
			firstTime = false;
		}
		else
		{
			refM(0,i) = dataM(0,i);
			refM(1,i) = dataM(1,i);
		}
		dataM(0,i) = x;
		dataM(1,i) = y;
		dataM(2,i) = 1.f;
	}
	const DP::Labels l;
	const DP ref(refM, l);
	const DP data(dataM, l);
	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);
	std::cout << "Final transformation:" << std::endl << T << std::endl;
	
	//Now we compute the new pose to obtain an estimated bState.
	
	
}
