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
	indexOfClosestPointToRobot = 0;
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	indexOfClosestPointToRobot = 0;
	currentPointIndex = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	currentPointIndex = 0;
	nextPointIndex = 1;
	finish = false;
	isBlocked = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;

	meanSpeed = 300.f;  //Initial speed. Should be read from disk
}

WayPoints::~WayPoints()
{
}

void WayPoints::reset()
{
	clear();
	indexOfClosestPointToRobot = 0;
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	indexOfClosestPointToRobot = 0;
	currentPointIndex = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	currentPointIndex = 0;
	nextPointIndex = 1;
	finish = false;
	isBlocked = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	backList.clear();
}

void WayPoints::startRoad()
{
	reloj.restart();
	setETA();
	initialDurationEstimation = getETA();
}

void WayPoints::endRoad()
{
	elapsedTime = reloj.restart();
	meanSpeed = 0.5 * meanSpeed + 0.5 * (getRobotDistanceToTarget() / elapsedTime);
	//We should save it now to disk
}

void WayPoints::readRoadFromFile(InnerModel *innerModel, std::string name)
{
	clear();
	std::ifstream file(name.c_str(), std::ios_base::in);
	if( file.is_open() )
	{
		QVec rPos = innerModel->transform("world","robot");
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
	for(int i=0; i+1<this->size(); i++) // exlude 0 because it is underneath the robot
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
	Q_ASSERT(currentPoint+1<road.size() and road.size()>0);
	
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
	Q_ASSERT (currentPoint+1 < size() and size()>0);

	QVec p1 = QVec::vec2( (*this)[currentPointIndex].pos.x(), (*this)[currentPointIndex].pos.z());
	QVec p2 = QVec::vec2( (*this)[currentPointIndex+1].pos.x(), (*this)[currentPointIndex+1].pos.z());	
	QLine2D line( p1, p2);
	return line;
}

void WayPoints::printRobotState(InnerModel* innerModel, const CurrentTarget &currentTarget)
{
		QVec robot3DPos = innerModel->transform("world", "robot");
		qDebug() << "-------Road status report  ---------------------";
		qDebug() << "	Robot position:" << robot3DPos;
		qDebug() << "	Target:" << currentTarget.getTranslation();
		qDebug() << "	Num points:" << this->size();
		qDebug() << "	Robot dist to closest point in road:" << getRobotDistanceToClosestPoint();
		qDebug() << "	Robot perp. dist to road tangent at closest point:" << getRobotPerpendicularDistanceToRoad();
		qDebug() << "	Angle with road:" << getAngleWithTangentAtClosestPoint();
		qDebug() << "	Dist to target:" << getRobotDistanceToTarget();
		qDebug() << "	Distance variation to target:" << getRobotDistanceVariationToTarget();
		qDebug() << "	Road curvature:" << getRoadCurvatureAtClosestPoint();
		//qDebug() << "	Index of closest point:" << getIndexOfClosestPointToRobot();
		qDebug() << "	Closest point:" << getIndexOfClosestPointToRobot()->pos;
		qDebug() << "	Tangent at closest point:" << getTangentAtClosestPoint();
		qDebug() << "	Current point index: " << currentPointIndex;
		qDebug() << "	Is Blocked:" << isBlocked;
		qDebug() << "	Is Lost:" << isLost;
		qDebug() << "	Is Finished:" << isFinished();
		qDebug() << "	Requires replanning:" << requiresReplanning;
		qDebug() << "	ETA:" << estimatedTimeOfArrival << " sg";
		qDebug() << "	ElapsedTime:" << elapsedTime/1000 << " sg";
		qDebug() << "	Duration estimation:" << initialDurationEstimation << " sg";
		qDebug() << "	Estimation error:" << initialDurationEstimation - elapsedTime/1000 << " sg";
		
		qDebug() << "----------------------------------------------------";
}

void WayPoints::print() const
{
	qDebug() << "Printing Road";
	for(int i=0; i<this->size(); i++)
	{
		const WayPoint &w = (*this)[i];
		qDebug() << "		" << w.pos << i << "Visible" << w.isVisible << "MinDist" << w.minDist << "MinDistVector" << w.minDistPoint 
							<< "visibilityAngle" << w.visibleLaserAngle << "visibilityDistance" << w.visibleLaserDist << "in robot frame" << w.posInRobotFrame;
	}
}

bool WayPoints::draw2(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo)
{
	RoboCompInnerModelManager::Pose3D pose;
	pose.y = 0;	pose.x = 0;	pose.z = 0;
	pose.rx = pose.ry = pose.z = 0.;
	
	try
	{	std::string  parentAll = "road";
		innermodelmanager_proxy->addTransform(parentAll,"static","floor", pose);		
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
	
	for(int i=1; i<size(); i++)
	{
		WayPoint &w = (*this)[i];
		WayPoint &wAnt = (*this)[i-1];
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint( QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		QVec tangent = roadTangentAtClosestPoint.getNormalForOSGLineDraw();		//OJO, PETA SI NO ESTA LA TG CALCULADA ANTES
		QString item = "p_" + QString::number(i);		
		pose.x = w.pos.x();	pose.y = 10; pose.z = w.pos.z();
		
		RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "road", pose);	
		if ( (int)i == (int)currentPointIndex+1 )	//CHANGE TO getIndexOfClosestPointToRobot()
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, tangent, 1000, 30, "#000055" );	
		if (w.isVisible)
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, normal, 250, 50, "#005500" );
		else
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, normal, 250, 50, "#550099" );  //Morado
	}
}

bool WayPoints::draw(InnerModelManagerPrx innermodelmanager_proxy, InnerModel *innerModel, int upTo)
{
	return draw2(innermodelmanager_proxy, innerModel, upTo);
	
	RoboCompInnerModelManager::Pose3D pose;
	pose.y = 0;
	pose.x = 0;
	pose.z = 0;
	pose.rx = pose.ry = pose.z = 0.;
	RoboCompInnerModelManager::meshType mesh;
		
	if( this->isEmpty() )
		return false;
	
	//clearDraw(innermodelmanager_proxy);
	
	QString item;
	if( upTo == -1) 
		upTo = this->size();  
	if( upTo < 0 ) 
		upTo = 0;
	if( upTo > this->size() ) 
		upTo = this->size();
	
	
	try
	{	std::string  parentAll = "road";
		innermodelmanager_proxy->addTransform(parentAll,"static","floor", pose);		
	}
	catch(const RoboCompInnerModelManager::InnerModelManagerError &ex)
	{ std::cout << ex << std::endl;}
	
	WayPoint &w = (*this)[0];
	item = "p_" + QString::number(0);		
	pose.x = w.pos.x();
	pose.y = 10;
	pose.z = w.pos.z();
	RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "road", pose);
	RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, QVec::vec3(0,0,1), 50, 50, "#335577" );
	
	for(int i=1; i<upTo; i++)
	{
		WayPoint &w = (*this)[i];
		WayPoint &wAnt = (*this)[i-1];
		
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint( QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		//QVec tangent = l.getNormalForOSGLineDraw();
		QVec tangent = roadTangentAtClosestPoint.getNormalForOSGLineDraw();		//OJO, PETA SI NO ESTA LA TG CALCULADA ANTES
		item = "p_" + QString::number(i);		
		pose.x = w.pos.x();
		pose.y = 10;
		pose.z = w.pos.z();
		RcisDraw::addTransform_ignoreExisting(innermodelmanager_proxy, item, "road", pose);
		RcisDraw::drawLine(innermodelmanager_proxy, item + "_point", item, normal, 150, 50, "#005500" );
		if ( (int)i == (int)currentPointIndex+1 )	//CHANGE TO getIndexOfClosestPointToRobot()
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, tangent, 1000, 30, "#000055" );	
	/*	else if (i == nextPointIndex )
		{
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30, "#999900" );
			QVec normalR = (getRobotZAxis(innerModel).getPerpendicularLineThroughPoint( QVec::vec2(getNextPoint().pos.x(), getNextPoint().pos.z()))).getNormalForOSGLineDraw();
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_lineA", item, normalR, 1000, 20, "#009999" );  //ligh blue, frontier
		}
*/
		else if(w.isVisible)
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30, "#550099" );  //Morado
		else	
			RcisDraw::drawLine(innermodelmanager_proxy, item + "_line", item, normal, 400, 30 );
		
		w.centerTransformName = item;
		w.centerLineName = item + "_line";
		w.centerPointName = item + "_point";
	}
	
	return true;
}


void WayPoints::clearDraw(InnerModelManagerPrx innermodelmanager_proxy)
{
	try
	{
 		//qDebug() << __FUNCTION__ << "deleting ROAD";
 		innermodelmanager_proxy->removeNode("road");
 	} 
	catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	{	
		//std::cout << e.text << std::endl;		
	}
}


//////////////////////////////////////////////////////////////////////////////////
///////COMPUTATION OF SCALAR MAGNITUDES OF FORCEFIELD
//////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Computes closest point in trajectory to robot. Updates robotDistanceToClosestPoint and currentPointIndex
 * 
 * @param robot 3d vector coding robot's position (x,y,z)
 * @return WayPoint closest to Robot
 */
WayPoints::iterator WayPoints::computeClosestPointToRobot(const QVec& robot)
{
	float min = FLT_MAX;
	QList<WayPoint>::iterator it = this->begin();
	QList<WayPoint>::iterator res = this->end();
	uint count=0, index = 0;
	
	for(it = this->begin(); it != this->end(); ++it, ++count)
	{
		float d = (it->pos - robot).norm2();
		//qDebug () << "d" << d;
		if( d < min ) 
		{
			min = d;
			res = it;
			index = count;
		}
	}
	
	robotDistanceToClosestPoint = min;
	currentPointIndex = index;   //DEPRECATED
	orderOfClosestPointToRobot = index;
	indexOfClosestPointToRobot = res;  // INDICES SHOULD NOT BE USED WITH LISTS
	return res;
}



/**
 * @brief Computes the road tangent at point pointed by iterator w. 
 * 
 * @param w WayPoints::iterator pointing to the point of interest
 * @return QLine2D
 */
QLine2D WayPoints::computeTangentAt(WayPoints::iterator w) const
{
	static QLine2D antLine = QLine2D(QVec::zeros(3), QVec::vec3(0,0,1));  //Initial well formed tangent
	
	if( size() <2) 
	{
		qDebug() << __FUNCTION__ << "Warning. A size 1 road got into here!";
		return antLine;
	}

	WayPoints::iterator ant,post;
	const float MIN_DISTANCE_ALLOWED = 10.f;
	
	if( w == this->begin())
		ant = w;
	else
		ant = w-1;
	
	if( w+1 == this->end())
		post = w;
	else
		post = w+1;
		
	//Now check if the points are too close
	if((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED )
		return antLine;
	
		/*and ((post+1) != this->end()))  //Search by the end side
		while ( post != this->end() and ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED))
		{ ++post; };
	
	if (post == this->end()) --post;
	if( (post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED and (post != this->begin())) //Search by the begin side
		while ( post != this->begin() and ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED))
		{ --post; };
	
	if( (post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED )
	{
		qDebug() << __FUNCTION__ << "Warning. This should not happen. Looks like the all remaining points are the same";*/
	//	return antLine;
	//}
		
	QLine2D l( ant->pos , post->pos );
	if( isnan(l[0]) or isnan(l[1]) or isnan(l[2]))
	{
		ant->pos.print("ant");
 		post->pos.print("post");
		l.print("line");
		print();
		qFatal("Fary in tg");
	}
	antLine = l;
	return  l;
}

/**
 * @brief Computes the distance from the robot to the last visible point in the road
 * 
 * @param road ...
 * @param closestPoint ...
 * @param robotPos ...
 * @return float
 */
float WayPoints::computeDistanceToLastVisible(WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for(it = closestPoint; it != end()-1; ++it)
	{
		if(it->isVisible == true )
			dist += (it->pos - (it+1)->pos).norm2();
		else 
			break;
	} 
	indexOfLastVisiblePoint = it;
	return dist;
}

/**
 * @brief Computes the distance to the Target along the road
 * 
 * @param road ...
 * @param robotPos ...
 * @return float Distance to target in the units of InnerModel
 */
float WayPoints::computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos)
{
	static  float antDist = (robotPos - closestPoint->pos).norm2();
	
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for(it = closestPoint; it != end()-1; ++it)
	{
		dist += (it->pos - (it+1)->pos).norm2();
	}
	float distE = (robotPos - it->pos).norm2();
	robotDistanceVariationToTarget = distE - antDist;
	antDist = distE;
	return dist;
}

/**
 * @brief Compute angle between current and next segment using the cross product
 * @param robot2DPos (X,Y) positionof the robot
 * @return float value between 0 curvature for straight lines and PI for U turns.
 */

float WayPoints::computeRoadCurvature(WayPoints::iterator closestPoint, uint pointsAhead)
{
	WayPoints::iterator it, final;
	uint count = 0;
	int achieved = 0;
	float sumAng = 0.f;
	for(it = closestPoint; it != end()-1 and count < pointsAhead; ++it, ++count, ++achieved)
	{
		float ang = computeTangentAt( it ).signedAngleWithLine2D( computeTangentAt( it + 1));
		//qDebug() << achieved << road.size() << ang;
		sumAng += ang;
	}
	if( achieved > 0 and isnan(sumAng)==false) 
		return sumAng/achieved;
	else 
		return 0;
}

void WayPoints::setETA()
{
	estimatedTimeOfArrival = getRobotDistanceToTarget() / meanSpeed;
	elapsedTime = reloj.elapsed();
}

/**
 * @brief Computes all scalar values used by the Controller to obtain the force field that acts on the robot
 * 
 * @return void
 */
bool WayPoints::computeForces()
{
	
	//Get robot's position in world and create robot's nose
	QVec robot3DPos = innerModel->transform("world", "robot");
	QVec noseInRobot = innerModel->transform("world", QVec::vec3(0,0,1000), "robot");
	QLine2D nose =  QLine2D(  QVec::vec2(robot3DPos.x(),robot3DPos.z()), QVec::vec2(noseInRobot.x(), noseInRobot.z()));
	
	//Compute closest existing trajectory point to robot
	WayPoints::iterator closestPoint = computeClosestPointToRobot(robot3DPos);
	
	//Compute roadTangent at closestPoint;
	//qDebug() << __FILE__  << __FUNCTION__ << "just here"  << getRobotDistanceToClosestPoint();
// 	if(closestPoint == this->end())
// 	{
// 		qDebug("fary en Compute Forces");
// 		return false;
// 	}
	
	QLine2D tangent = computeTangentAt( closestPoint );
	setTangentAtClosestPoint(tangent);
	
	//Compute signed perpenduicular distance from robot to tangent at closest point
	setRobotPerpendicularDistanceToRoad( tangent.perpendicularDistanceToPoint(robot3DPos) );
	float ang = nose.signedAngleWithLine2D( tangent );
	if (isnan(ang))
		ang = 0;
 	setAngleWithTangentAtClosestPoint( ang );
	
	//Compute distanceToTarget along trajectory
  	setRobotDistanceToTarget( computeDistanceToTarget(closestPoint, robot3DPos) );  //computes robotDistanceVariationToTarget
	setRobotDistanceVariationToTarget( robotDistanceVariationToTarget);
	
	//Update estimated time of arrival
	setETA();
	
	//Check for arrival to target  TOO SIMPLE 
	if(	( ((int)getCurrentPointIndex()+1 == (int)size())  and  ( getRobotDistanceToTarget() < 100) )
		or ( (getRobotDistanceToTarget() < 1000) and ( getRobotDistanceVariationToTarget() > 0) ) )
		setFinished(true);
	
	//compute curvature of trajectory at closest point to robot
  	setRoadCurvatureAtClosestPoint( computeRoadCurvature(closestPoint, 3) );
	setRobotDistanceToLastVisible( computeDistanceToLastVisible(closestPoint, robot3DPos ) );
	return true;
}

