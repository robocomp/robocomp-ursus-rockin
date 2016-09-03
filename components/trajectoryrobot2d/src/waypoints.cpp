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
	iterToClosestPointToRobot = 0;
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	iterToClosestPointToRobot = 0;
	indexOfCurrentPoint = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	indexOfCurrentPoint = 0;
	indexOfNextPoint = 1;
	finish = false;
	blockedRoad = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	meanSpeed = 300.f;  //Initial speed. Should be read from disk
	antDist = std::numeric_limits<float>::max();
}

WayPoints::~WayPoints()
{
}

void WayPoints::reset()
{
	clear();
	iterToClosestPointToRobot = this->end();
	robotDistanceToClosestPoint = 0;
	robotPerpendicularDistanceToRoad = 0;
	indexOfCurrentPoint = 0;
	angleWithTangentAtClosestPoint = 0;
	roadCurvatureAtClosestPoint = 0;
	robotDistanceToTarget = 0;
	robotDistanceToLastVisible = 0;
	indexOfCurrentPoint = 0;
	indexOfNextPoint = 1;
	finish = false;
	blockedRoad = false;
	isLost = false;
	currentCollisionIndex = 0;
	currentDistanceToFrontier = 0;
	requiresReplanning = false;
	backList.clear();
	antDist = std::numeric_limits<float>::max();

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
	if (file.is_open())
	{
		QVec rPos = innerModel->transform("world", "robot");
		append(WayPoint(rPos));

		while (file.eof() == false)
		{
			WayPoint w(QVec::zeros(3));
			file >> w.pos[0] >> w.pos[2];
			append(w);
		}
// 		qDebug() << "SpecificWorker::ReadFromFile:: " << QString::fromStdString(name) << "read with" << size() << "points";
	}
	else
		qDebug() << "Could not open File " << QString::fromStdString(name);
}

void WayPoints::readRoadFromList(QList<QVec> list)
{
	Q_ASSERT_X(list.size() > 0, "readRoadFromList", "Empty list");
	clear();
			foreach(QVec point, list)
		{
			append(WayPoint(point));
		}
	//print();
}

void WayPoints::computeDistancesToNext()
{
	for (int i = 0; i + 1 < this->size(); i++) // exlude 0 because it is underneath the robot
	{
		this->operator[](i).initialDistanceToNext = (this->operator[](i).pos - this->operator[](i + 1).pos).norm2();
	}
}

void WayPoints::removeFirst(InnerModelViewer *innerViewer)
{
	InnerModelDraw::removeObject(innerViewer, first().centerTransformName);
	InnerModelDraw::removeObject(innerViewer, first().centerMeshName);
	InnerModelDraw::removeObject(innerViewer, first().ballTransformName);
	InnerModelDraw::removeObject(innerViewer, first().ballMeshName);

	QList<WayPoint>::removeFirst();
}

/**
 * @brief Compute QLine2D corresponding to the robot Z axis in World reference frame
 *
 * @return QLine2D
 */
QLine2D WayPoints::getRobotZAxis(InnerModel *innerModel)
{
	Q_ASSERT(currentPoint + 1 < road.size() and road.size() > 0);

	QVec robotPos = innerModel->transform("world", QVec::zeros(3), "base");
	//QVec robot2DPos = QVec::vec2( innerModel->getBaseX(), innerModel->getBaseZ());
	QVec nose = innerModel->transform("world", QVec::vec3(0, 0, 1000), "base");
	//QVec noseR = QVec::vec2(nose.x(),nose.z());
	return QLine2D(robotPos, nose);

}


float WayPoints::robotDistanceToCurrentPoint(InnerModel *innerModel)
{
	return (innerModel->transform("world", QVec::zeros(3), "base") - (*this)[indexOfCurrentPoint].pos).norm2();
}

float WayPoints::robotDistanceToNextPoint(InnerModel *innerModel)
{
	return (innerModel->transform("world", QVec::zeros(3), "base") - (*this)[indexOfNextPoint].pos).norm2();
}

QLine2D WayPoints::getTangentToCurrentPoint()
{
	Q_ASSERT (currentPoint + 1 < size() and size() > 0);

	QVec p1 = QVec::vec2((*this)[indexOfCurrentPoint].pos.x(), (*this)[indexOfCurrentPoint].pos.z());
	QVec p2 = QVec::vec2((*this)[indexOfCurrentPoint + 1].pos.x(), (*this)[indexOfCurrentPoint + 1].pos.z());
	QLine2D line(p1, p2);
	return line;
}

void WayPoints::printRobotState(InnerModel *innerModel, const CurrentTarget &currentTarget)
{
	QVec robot3DPos = innerModel->transform("world", "robot");
	qDebug() << "-------Road status report  ---------------------";
	qDebug() << "	Robot position:" << robot3DPos;
	qDebug() << "	Target:" << currentTarget.getTranslation();
	qDebug() << "	Target2:" << last().pos;
	qDebug() << "	Num points:" << this->size();
	qDebug() << "	Robot dist to closest point in road:" << getRobotDistanceToClosestPoint();
	qDebug() << "	Robot perp. dist to road tangent at closest point:" << getRobotPerpendicularDistanceToRoad();
	qDebug() << "	Angle with road:" << getAngleWithTangentAtClosestPoint();
	qDebug() << "	Dist to target:" << getRobotDistanceToTarget();
	qDebug() << "	Distance variation to target:" << getRobotDistanceVariationToTarget();
	qDebug() << "	Road curvature:" << getRoadCurvatureAtClosestPoint();
	//qDebug() << "	Index of closest point:" << getIndexOfClosestPointToRobot();
	qDebug() << "	Closest point:" << getIterToClosestPointToRobot()->pos;
	qDebug() << "	Tangent at closest point:" << getTangentAtClosestPoint();
	qDebug() << "	Current point index: " << indexOfCurrentPoint;
	qDebug() << "	Is Blocked:" << isBlocked();
	qDebug() << "	Is Lost:" << isLost;
	qDebug() << "	Is Finished:" << isFinished();
	qDebug() << "	Requires replanning:" << requiresReplanning;
	qDebug() << "	ETA:" << estimatedTimeOfArrival << " sg";
	qDebug() << "	ElapsedTime:" << elapsedTime / 1000 << " sg";
	qDebug() << "	Duration estimation:" << initialDurationEstimation << " sg";
	qDebug() << "	Estimation error:" << initialDurationEstimation - elapsedTime / 1000 << " sg";
	qDebug() << "	Road BackList size:" << backList.size();


	qDebug() << "----------------------------------------------------";
}

void WayPoints::print() const
{
	qDebug() << __FUNCTION__ << "Printing Road";
	for (int i = 0; i < this->size(); i++)
	{
		const WayPoint &w = (*this)[i];
		qDebug() << "		-" << w.pos << w.rot << i << "Visible" << w.isVisible << "MinDist" << w.minDist << "MinDistVector"
		         << w.minDistPoint;
		qDebug() << "		" << "laser ang visibility" << w.visibleLaserAngle << "laser dist visibility"
		         << w.visibleLaserDist << "in robot frame" << w.posInRobotFrame;
	}
}

bool WayPoints::draw(InnerModelViewer *innerViewer, InnerModel *innerModel, const CurrentTarget &currentTarget)
{
	clearDraw(innerViewer);
	if (size() == 0) return false;
	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "road", "world");

	//Draw all points now
	for (int i = 1; i < size(); i++)
	{
		WayPoint &w = (*this)[i];
		WayPoint &wAnt = (*this)[i - 1];
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		QVec tangent = roadTangentAtClosestPoint.getNormalForOSGLineDraw();        //OJO, PETA SI NO ESTA LA TG CALCULADA ANTES
		QString item = "p_" + QString::number(i);
		InnerModelDraw::addTransform_ignoreExisting(innerViewer, item, "road");
		
		innerViewer->innerModel->updateTransformValues(item, w.pos.x(), 10, w.pos.z(), 0, 0, 0);

		if ((int) i == (int) indexOfCurrentPoint + 1) //CHANGE TO getIndexOfClosestPointToRobot()
		{
			InnerModelDraw::drawLine(innerViewer, item + "_line", item, tangent, 600, 30, "#000055");
		}
		if (w.isVisible)
			InnerModelDraw::drawLine(innerViewer, item + "_point", item, normal, 250, 50, "#005500");
		else
			InnerModelDraw::drawLine(innerViewer, item + "_point", item, normal, 250, 50, "#550099");  //Morado
	}
	if (currentTarget.hasRotation() == true)    //Draw an arrow indicating final desired orientation
	{
		float rot = currentTarget.getRotation().y();
		WayPoint &w = this->last();
		QLine2D l(w.pos, w.pos + QVec::vec3((T) (500 * sin(rot)), 0, (T) (500 * cos(rot))));
		QVec ln = l.getNormalForOSGLineDraw();
		QString item = "p_" + QString::number(this->size() - 1);
		InnerModelDraw::drawLine(innerViewer, item + "_line", item, ln, 600, 30, "#400055");
	}

	return true;
}


void WayPoints::clearDraw(InnerModelViewer *innerViewer)
{
	if (innerViewer->innerModel->getNode("road"))
		InnerModelDraw::removeNode(innerViewer, "road");
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
WayPoints::iterator WayPoints::computeClosestPointToRobot(const QVec &robot)
{
	float min = FLT_MAX;
	QList<WayPoint>::iterator it = this->begin();
	QList<WayPoint>::iterator res = this->end();
	uint count = 0, index = 0;

	for (it = this->begin(); it != this->end(); ++it, ++count)
	{
		float d = (it->pos - robot).norm2();
		if (d < min)
		{
			min = d;
			res = it;
			index = count;
		}
	}

	robotDistanceToClosestPoint = min;
	indexOfCurrentPoint = index;   //DEPRECATED
	indexOfClosestPointToRobot = index;
	iterToClosestPointToRobot = res;  // INDICES SHOULD NOT BE USED WITH LISTS
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
	static QLine2D antLine = QLine2D(QVec::zeros(3), QVec::vec3(0, 0, 1));  //Initial well formed tangent

	if (size() < 2)
	{
		qDebug() << __FUNCTION__ << "Warning. A size 1 road got into here!";
		return antLine;
	}

	WayPoints::iterator ant, post;
	const float MIN_DISTANCE_ALLOWED = 10.f;

	if (w == this->begin())
		ant = w;
	else
		ant = w - 1;

	if (w + 1 == this->end())
		post = w;
	else
		post = w + 1;

	//Now check if the points are too close
	if ((post->pos - ant->pos).norm2() < MIN_DISTANCE_ALLOWED)
		return antLine;

	// The tangent
	QLine2D l(ant->pos, post->pos);

	if (std::isnan(l[0]) or std::isnan(l[1]) or std::isnan(l[2]))
	{
		ant->pos.print("ant");
		post->pos.print("post");
		l.print("line");
		print();
		qFatal("Fary in tg");
	}
	antLine = l;
	return l;
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
	for (it = closestPoint; it != end() - 1; ++it)
	{
		if (it->isVisible == true)
			dist += (it->pos - (it + 1)->pos).norm2();
		else
			break;
	}
	iterToLastVisiblePoint = it;
	return dist;
}

/**
 * @brief Computes the distance to the Target along the road starting at closestPoint
 *
 * @param road ...
 * @param robotPos ...
 * @return float Distance to target in the units of InnerModel
 */
float WayPoints::computeDistanceToTarget(WayPoints::iterator closestPoint, const QVec &robotPos)
{
	float dist = (robotPos - closestPoint->pos).norm2();
	WayPoints::iterator it;
	for (it = closestPoint; it != end() - 1; ++it)
	{
		dist += (it->pos - (it + 1)->pos).norm2();
	}
	float distE = (robotPos -
	               it->pos).norm2();  //Euclidean distance to last point to detect the robot going away from the target
	robotDistanceVariationToTarget = distE - antDist;
	antDist = distE;
	return dist;
}


/**
 * @brief Computes distance between two points along the road 
 * 
 * @param firstPoint ...
 * @param secondPoint ...
 * @return float distance (-1 if it could not be computed)
 */
float WayPoints::computeDistanceBetweenPointsAlongRoad(WayPoints::iterator firstPoint, WayPoints::iterator secondPoint)
{
	float dist = 0.f;
	for (WayPoints::iterator it = firstPoint; it != secondPoint - 1; ++it)
	{
		dist += (it->pos - (it + 1)->pos).norm2();
	}
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
	for (it = closestPoint; it != end() - 1 and count < pointsAhead; ++it, ++count, ++achieved)
	{
		float ang = computeTangentAt(it).signedAngleWithLine2D(computeTangentAt(it + 1));
		//qDebug() << achieved << road.size() << ang;
		sumAng += ang;
	}
	if (achieved > 0 and std::isnan(sumAng) == false)
		return sumAng / achieved;
	else
		return 0;
}

/**
 * @brief Estimated time of arrival en miliseconds
 * 
 * @return void
 */
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
void WayPoints::update()
{
	//////////////////////////////////////////////////////
	//Get robot's position in world and create robot's nose
	//////////////////////////////////////////////////////
	QVec robot3DPos = innerModel->transform("world", "robot");
	QVec noseInRobot = innerModel->transform("world", QVec::vec3(0, 0, 1000), "robot");
	QLine2D nose = QLine2D(QVec::vec2(robot3DPos.x(), robot3DPos.z()), QVec::vec2(noseInRobot.x(), noseInRobot.z()));

	////////////////////////////////////////////////////
	//Compute closest existing trajectory point to robot
	///////////////////////////////////////////////////
	if (getRobotDistanceToTarget() < 1000)
	{
		robot3DPos = innerModel->transform("world", "virtualRobot");
	}
	WayPoints::iterator closestPoint = computeClosestPointToRobot(robot3DPos);

	///////////////////////////////////////
	//Compute roadTangent at closestPoint
	///////////////////////////////////////
	QLine2D tangent = computeTangentAt(closestPoint);
	setTangentAtClosestPoint(tangent);

	//////////////////////////////////////////////////////////////////////////////
	//Compute signed perpenduicular distance from robot to tangent at closest point
	///////////////////////////////////////////////////////////////////////////////
	setRobotPerpendicularDistanceToRoad(tangent.perpendicularDistanceToPoint(robot3DPos));

	////////////////////////////////////////////////////
	//Compute signed angle between nose and tangent at closest point
	////////////////////////////////////////////////////
	float ang = nose.signedAngleWithLine2D(tangent);
	if (std::isnan(ang))
		ang = 0;
	setAngleWithTangentAtClosestPoint(ang);

	/////////////////////////////////////////////
	//Compute distance to target along trajectory
	/////////////////////////////////////////////
	setRobotDistanceToTarget(computeDistanceToTarget(closestPoint, robot3DPos));  //computes robotDistanceVariationToTarget
	setRobotDistanceVariationToTarget(robotDistanceVariationToTarget);

	//////////////////////////////////
	//Update estimated time of arrival
	//////////////////////////////////
	setETA();

	////////////////////////////////////////////////////////////
	//Compute curvature of trajectory at closest point to robot
	////////////////////////////////////////////////////////////
	setRoadCurvatureAtClosestPoint(computeRoadCurvature(closestPoint, 3));

	////////////////////////////////////////////////////////////
	//Compute distance to last road point visible with laser field
	////////////////////////////////////////////////////////////
	setRobotDistanceToLastVisible(computeDistanceToLastVisible(closestPoint, robot3DPos));

	////////////////////////////////////////////////////////////
	// Compute robot angle in each point
	// //////////////////////////////////////////////////////////
	for( WayPoints::iterator it = this->begin(); it != this->end(); ++it )
	{
		QLine2D l = computeTangentAt(it);
		QVec d = l.getDirectionVector();
		float ang =  atan2(d.x(), d.y());
		if(ang>0) ang -= M_PI;
		else ang += M_PI;
		it->rot = QVec::vec3(0, ang, 0);
	}

	///////////////////////////////////////////
	//Check for arrival to target (translation)  TOO SIMPLE
	///////////////////////////////////////////
	if (((((int) getIndexOfCurrentPoint() + 1 == (int) size()) or  (getRobotDistanceToTarget() < 50))) or
	    ((getRobotDistanceToTarget() < 100) and (getRobotDistanceVariationToTarget() > 0)))
	{
		setFinished(true);
		//		qDebug() << __FUNCTION__ << "Arrived:" << (int)getCurrentPointIndex()+1 << (int)getCurrentPointIndex()+1 << getRobotDistanceToTarget() << getRobotDistanceVariationToTarget();
		// 		getIndexOfClosestPointToRobot()->pos.print("closest point");
	}

	///////////////////////////////////////////
	//Check for blocked road
	///////////////////////////////////////////
//	if( isVisible(indexOfNextPoint) == false)
//		setBlocked(true);
}

