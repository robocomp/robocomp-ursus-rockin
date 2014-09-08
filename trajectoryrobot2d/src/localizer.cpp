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

#include <fstream>

#include "localizer.h"

Localizer::Localizer(InnerModel &innerModel)
{
	//clonModel = new InnerModel( *inner );
	//recursiveIncludeMeshes( clonModel->getRoot(), "robot", false, robotNodes, restNodes);
	recursiveIncludeMeshes( innerModel.getRoot(), "robot", false, robotNodes, restNodes);

	icp.setDefault();	
}

void Localizer::localize(const RoboCompLaser::TLaserData &laser, InnerModel &innerModel, int nLaserRays)
{
	QVec point = innerModel.transform("world", "robot");
	float alfa = innerModel.getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	float step = float(laser.size()) / float(nLaserRays);

	virtualLaser.resize(nLaserRays);
	subsampledLaser.resize(nLaserRays);
	
	int k = 0;
 	for (float i=0; k<nLaserRays; i+=step)
 	{
 		virtualLaser[k].angle    = laser[int(floor(i))].angle;
		subsampledLaser[k].dist  = laser[int(floor(i))].dist;
		subsampledLaser[k].angle = laser[int(floor(i))].angle;
		k++;
 	}
	
	laserRender(innerModel, point, alfa);

// 	std::ofstream outputFileVS;
// 	outputFileVS.open("laserVirtualS.csv");
// 	std::ofstream outputFileNS;
// 	outputFileNS.open("laserNormalS.csv");
// 	for (size_t i=0; i<virtualLaser.size(); i++)
// 	{
// 		outputFileVS <<    virtualLaser[i].angle << "," <<    virtualLaser[i].dist << "\n";
// 		outputFileNS << subsampledLaser[i].angle << "," << subsampledLaser[i].dist << "\n";
// 	}
// 	outputFileVS.close();
// 	outputFileNS.close();
// 
// 	sleep(3);
// 	

	qDebug() << "VLaser:" ;
	for (uint i=0; i<virtualLaser.size(); i++)
	{
		qDebug() << "	Dist:" << virtualLaser[i].dist << ". DistSub:" << subsampledLaser[i].dist << ". Angle" << virtualLaser[i].angle
							<< ". Diff: " << subsampledLaser[i].dist - virtualLaser[i].dist;
	}
// 	estimatePoseWithICP(laser, point, alfa);
	
	localizeInMapWithICP();
	
}

void Localizer::laserRender(InnerModel &innerModel, const QVec& point, float alfa)
{
	// TODO: GET FROM VISTUAL LASER SPECIFICATION	
	const float MAX_LENGTH_ALONG_RAY = 4000;
	
	// Update robot's position
	QVec currentPos = innerModel.transform("world","robot");
	float currentAlfa = innerModel.getRotationMatrixTo("world", "robot").extractAnglesR_min().y();
	innerModel.updateTransformValues("robot", point.x(), 0, point.z(), 0., alfa, 0.);
	
	// Compute rotation matrix between laser and world
	QMat r1q1 = innerModel.getRotationMatrixTo("world", "laser");
	
	// Create hitting appex
	boost::shared_ptr<fcl::Box> laserBox(new fcl::Box(0.1, 0.1, 0.1));
	fcl::CollisionObject laserBoxCol(laserBox);

	for (uint32_t i=0; i<virtualLaser.size(); i++)
	{
		bool hit = false;
		// Rotation of appex
		const QMat r1q = r1q1 * RMat::Rot3DOY(virtualLaser[i].angle);
		const fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
		
		// Check collision at maximum distance
		float hitDistance = MAX_LENGTH_ALONG_RAY;
		laserBox->side = fcl::Vec3f(0.1, 0.1, hitDistance);
 		innerModel.updateRotationValues("laserPose", 0, virtualLaser[i].angle, 0);
 		const QVec boxBack = innerModel.transform("world", QVec::vec3(0, 0, hitDistance/2.), "laser");
 		innerModel.updateRotationValues("laserPose", 0, 0, 0);
 		laserBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
		
 		for (uint out=0; out<restNodes.size(); out++)
 		{
 			hit = innerModel.collide(restNodes[out], &laserBoxCol);
 			if (hit) 
				break;
 		}
 		
 		// Binary search
 		if (hit)
		{
			hit = false;
			float min=0;
			float max=MAX_LENGTH_ALONG_RAY;

			while (max-min>10)
			{
				// Stretch and create the stick
				hitDistance = (max+min)/2.;
				laserBox->side = fcl::Vec3f(0.1, 0.1, hitDistance);
				innerModel.updateRotationValues("laserPose", 0, virtualLaser[i].angle, 0);
				const QVec boxBack = innerModel.transform("world", QVec::vec3(0, 0, hitDistance/2.), "laser");
				innerModel.updateRotationValues("laserPose", 0, 0, 0);
				laserBoxCol.setTransform(R1, fcl::Vec3f(boxBack(0), boxBack(1), boxBack(2)));
				
				// Check collision using current ray length
				for (uint32_t out=0; out<restNodes.size(); out++)
				{
					hit = innerModel.collide(restNodes[out], &laserBoxCol);
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
		}
		// Fill the laser's structure
		virtualLaser[i].dist = hitDistance;
	}
	
	//Restore innermodel
	innerModel.updateTransformValues("robot", currentPos.x(), 0, currentPos.z(), 0., currentAlfa, 0.);
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

void Localizer::localizeInMapWithICP()
{
	float xV,yV,xR,yR;
	
	//CHECK IF SOME DISPLACEMENT HAS OCCURED
	
	refM.resize(3,virtualLaser.size());
	dataM.resize(3,subsampledLaser.size());
	
	for(uint i=0; i<virtualLaser.size(); i++)
	{
		xV = virtualLaser[i].dist * sin( virtualLaser[i].angle );
		yV = virtualLaser[i].dist * cos( virtualLaser[i].angle );
		xR = subsampledLaser[i].dist * sin( subsampledLaser[i].angle);
		yR = subsampledLaser[i].dist * cos( subsampledLaser[i].angle);
		
		refM(0,i) = xV;
		refM(1,i) = yV;
		refM(2,i) = 1.f;
		dataM(0,i) = xR;
		dataM(1,i) = yR;
		dataM(2,i) = 1.f;
	}
	
	const DP::Labels l;
	const DP ref(refM, l);
	const DP data(dataM, l);
	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);
	std::cout << "Final transformation:" << std::endl << T << std::endl;
	
	//Now we compute the new pose to obtain an estimated bState.
	
// 	DP data_out(data);
// 	icp.transformations.apply(data_out, T);
// 	ref.save("test_ref.vtk");
// 	data.save("test_data_in.vtk");
// 	data_out.save("test_data_out.vtk");
// 	
// 	qFatal("fary");
}

