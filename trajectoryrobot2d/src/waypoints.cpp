/*
 * Copyright 2013 <copyright holder> <email>
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

#include "waypoints.h"

WayPoints::WayPoints()
{
	currentPointIndex = 0;
	nextPointIndex = 1;
	
	distanceToRoad = 0 ;
	angleWithTangent = 0;
	distanceToTarget = 0;
	roadCurvature = 0;
	finish = false;
	isBlocked = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	
}

WayPoints::~WayPoints()
{
}

void WayPoints::readRoadFromFile(InnerModel *innerModel, std::string name)
{
	clear();
	std::ifstream file(name.c_str(), std::ios_base::in);
	if( file.is_open() )
	{
		QVec rPos = innerModel->getBaseCoordinates();
		rPos[2] += 100;  ///CHECK
		append(WayPoint(rPos));
		
		while(file.eof() == false)
		{
			WayPoint w(QVec::zeros(3));
			file >> w.pos[0] >> w.pos[2];
			append(w);
		}
		qDebug() << "SpecificWorker::ReadFromFile:: " << QString::fromStdString(name) << "read with" << size() << "points";
	}
	else
		qDebug() << "Could not open File " << QString::fromStdString( name );
}

void WayPoints::readRoadFromList( QList< QVec > list)
{
	Q_ASSERT_X(list.size()>0, "readRoadFromList", "Empty list");
	clear();
	foreach(QVec point, list)
	{
		append(WayPoint(point));
	}
}

void WayPoints::computeDistancesToNext()
{
	for(int i=0; i< this->size()-1; i++) // exlude 0 because it is underneath the robot
	{
		this->operator[](i).initialDistanceToNext = (this->operator[](i).pos - this->operator[](i+1).pos).norm2();
	}
}

void WayPoints::removeFirst(InnerModelManagerPrx innermodelmanager_proxy)
{
	RcisDraw::removeObject(innermodelmanager_proxy, first().centerTransformName);
	RcisDraw::removeObject(innermodelmanager_proxy, first().centerMeshName);
	RcisDraw::removeObject(innermodelmanager_proxy, first().ballTransformName);
	RcisDraw::removeObject(innermodelmanager_proxy, first().ballMeshName);
	
	QList< WayPoint >::removeFirst();
}

/**
 * @brief Compute QLine2D corresponding to the robot Z axis in World reference frame
 * 
 * @return QLine2D
 */
QLine2D WayPoints::getRobotZAxis(InnerModel* innerModel)
{
	Q_ASSERT(currentPoint<road.size()-1 and road.size()>0);
	
	QVec robot2DPos = QVec::vec2( innerModel->getBaseX(), innerModel->getBaseZ());
	QVec nose = innerModel->transform("world", QVec::vec3(0,0,1000), "base");
	QVec noseR = QVec::vec2(nose.x(),nose.z());
	return QLine2D(robot2DPos, noseR);
}


float WayPoints::robotDistanceToCurrentPoint(InnerModel* innerModel)
{
	return (innerModel->getBaseCoordinates() - (*this)[currentPointIndex].pos).norm2();
}

float WayPoints::robotDistanceToNextPoint(InnerModel* innerModel)
{
	return (innerModel->getBaseCoordinates() - (*this)[nextPointIndex].pos).norm2();
}

QLine2D WayPoints::getTangentToCurrentPoint()
{
	Q_ASSERT (currentPoint < size()-1 and size()>0);

	QVec p1 = QVec::vec2( (*this)[currentPointIndex].pos.x(), (*this)[currentPointIndex].pos.z());
	QVec p2 = QVec::vec2( (*this)[currentPointIndex+1].pos.x(), (*this)[currentPointIndex+1].pos.z());	
	QLine2D line( p1, p2);
	return line;
}

void WayPoints::printRobotState(InnerModel* innerModel)
{
		qDebug() << "WayPoints ---------------------";
		qDebug() << "	num points " << this->size();
		qDebug() << "	current point index " << this->currentPointIndex;
		qDebug() << "	next point index" << this->nextPointIndex;
		qDebug() << "	dist to road " << distanceToRoad;
		qDebug() << "	angle with road " << angleWithTangent;
		qDebug() << "	dist to target " << distanceToTarget;
		qDebug() << "	dist to current " << robotDistanceToCurrentPoint(innerModel);
		qDebug() << "	dist to next " << robotDistanceToNextPoint(innerModel);
		qDebug() << "	distance to frontier " << currentDistanceToFrontier;
		qDebug() << "	road curvature " << roadCurvature;
		qDebug() << "	Current point" << getCurrentPoint().pos;
		qDebug() << "	Next Point" << getNextPoint().pos;
		qDebug();		
}

void WayPoints::print()
{
	qDebug() << "Printing Road";
	for(int i=0; i<this->size(); i++)
	{
		WayPoint &w = (*this)[i];
		qDebug() << "		" << w.pos << i << "Visible" << w.isVisible << "MinDist" << w.minDist << "MinDistVector" << w.minDistPoint 
							<< "visibilityAngle" << w.visibleLaserAngle << "visibilityDistance" << w.visibleLaserDist << "in robot frame" << w.posInRobotFrame;
	}
}

bool WayPoints::draw(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo)
{
	static int nPoints=0;
	
	RoboCompInnerModelManager::Pose3D pose;
	pose.y = 0;
	pose.x = 0;
	pose.z = 0;
	pose.rx = pose.ry = pose.z = 0.;
	RoboCompInnerModelManager::meshType mesh;
	
	//Number or points in RCIS
//  	RoboCompInnerModelManager::NodeInformationSequence nodesInfo;
//  	try
//  	{ 
// 		innermodelmanager_proxy->getAllNodeInformation(nodesInfo);
// 		qDebug() << " WayPoints::draw nodes in RCIS" << nodesInfo.size() << "points in road" <<this->size();;
// 	}
//  	catch(const Ice::Exception &e){ std::cout << "Removing points " << e << std::endl;};
	
	if( this->isEmpty() )
		return false;
	
	QString item;
	
	if( upTo == -1) 
		upTo = this->size();  
	if( upTo < 0 ) 
		upTo = 0;
	if( upTo > this->size() ) 
		upTo = this->size();
	
	WayPoint &w = (*this)[0];
	item = "p_" + QString::number(0);		
	pose.x = w.pos.x();
	pose.y = 1900;
	pose.z = w.pos.z();
	RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "floor", pose);
	RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, QVec::vec3(0,0,1), 50, 50, "#335577" );
	
	for(int i=1; i<upTo; i++)
	{
		WayPoint &w = (*this)[i];
		WayPoint &wAnt = (*this)[i-1];
		
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint( QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		item = "p_" + QString::number(i);		
		pose.x = w.pos.x();
		pose.y = 1900;
		pose.z = w.pos.z();
		RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "floor", pose);
		RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, normal, 150, 50, "#005500" );
		if ( i-1 == currentPointIndex )
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 1000, 30, "#000055" );	
		else if (i == nextPointIndex )
		{
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30, "#999900" );
			QVec normalR = (getRobotZAxis(innerModel).getPerpendicularLineThroughPoint( QVec::vec2(getNextPoint().pos.x(), getNextPoint().pos.z()))).getNormalForOSGLineDraw();
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_lineA", item, normalR, 1000, 20, "#009999" );  //ligh blue, frontier
		}
		else	if(w.isVisible)
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30, "#550099" );  //Morado
		else	
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30 );
		
		
		w.centerTransformName = item;
		w.centerLineName = item + "_line";
		w.centerPointName = item + "_point";
	}
	
	//Cleanup 
	for(int i=upTo; i<nPoints; i++)
	{
		try
		{
			QString obj = QString("p_" + QString::number(i));
			//qDebug() << "RCDRaw::removing " << obj;
			innermodelmanager_proxy->removeNode(obj.toStdString());
		}
		catch(const Ice::Exception &ex){ std::cout << "Shit removinh" << ex << std::endl;};	
	}
	nPoints = upTo;
	
	return true;
}

// void WayPoints::draw2(InnerModelManagerPrx innermodelmanager_proxy, int upTo)
// {
// 	RoboCompInnerModelManager::Pose3D pose;
// 	RoboCompInnerModelManager::meshType mesh;
// 	QString item;
// 	
// 	if( upTo == -1) 
// 		upTo = this->size();  
// 	if( upTo < 0 ) 
// 		upTo = 0;
// 	if( upTo > this->size() ) 
// 		upTo = this->size();
// 	
// 	for(int i=0; i<upTo; i++)
// 	{
// 		WayPoint &w = (*this)[i];
// 		posef = 100;
// 		pose.x = w.pos.x(); 
// 		pose.z = w.pos.z();
// 		mesh.scaleX = w.minDist;	
// 		mesh.scaleZ = w.minDist;
// 		mesh.scaleY = 5;
// 		mesh.meshPath = "/home/robocomp/robocomp/Files/osgModels/basics/sphere.ive";
// 		mesh.pose.x = mesh.pose.y = mesh.pose.z = 0.;
// 		item = "b_" + QString::number(i);
// 		RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "floor", pose);
// 		RcisDraw::addMesh_ignoreExisting(innermodelmanager_proxy, item + "_mesh", item, mesh );
// 		w.ballTransformName = item;
// 		w.ballMeshName = item+"_mesh";
// 		//the blue centers
// 		item = "p_" + QString::number(i);		
// 		pose.y = 100;		pose.x = w.pos.x(); 		pose.z = w.pos.z();
// 		mesh.meshPath = "/home/robocomp/robocomp/Files/osgModels/humanColor/jointbox_azul.osg";	 
// 		mesh.scaleX = mesh.scaleY = mesh.scaleZ=50;
// 		mesh.pose.x = mesh.pose.y = mesh.pose.z = 0;
// 		RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "floor", pose);
// 		RcisDraw::addMesh_ignoreExisting(innermodelmanager_proxy, item + "_mesh", item, mesh );
// 		w.centerTransformName = item;
// 		w.centerMeshName = item+"_mesh";
// 	}
// }

