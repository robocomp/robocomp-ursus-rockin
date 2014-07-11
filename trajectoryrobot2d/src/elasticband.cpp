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

#include "elasticband.h"
#include <complex>
#include <boost/concept_check.hpp>

ElasticBand::ElasticBand(InnerModel *_innermodel):
	innermodel(_innermodel)
{
}

ElasticBand::~ElasticBand()
{
}

bool ElasticBand::update(WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	
	//qDebug() << __FILE__ << __FUNCTION__ << "ElasticBand::Update - "<< "num points " << road.size();
	
	if( road.finish == true )
			return false;

	//Tags all points in the road as visible or blocked, depending on laser visibility
	//Only visible points are processed in this iteration
	checkVisiblePoints(road, laserData);

	//road.print();
	
 	if( road[1].isVisible == false)
 	{
		qDebug() << __FILE__ << __FUNCTION__ << "ElasticBand::update NextPoint is NOT visible or road BLOCKED";
		road.requiresReplanning = true;
 		return false;
 	}
	
	///Instead of the Jacobian wrt the point distance field, a better approach would be the Jacobian wrt the real shape of the robot. 
	///Mindist would be between robot pose at point, including angle, and laser field
	
	//Computes repulsive and attractive forces and moves the road elements accordingly
	computeForces(road, laserData); 	 
	
 	addPoints(road);
	//adjustPoints(road);
 	cleanPoints(road);
	
	return true;
}

// void ElasticBand::adjustPoints(WayPoints &road)
// {
// 	Q_ASSERT( road.size()>1 );
// 		
// 	for(int i=0; i< road.size()-1; i++) 
// 	{
// 		if( i>0 and road[i].isVisible == false )
// 			break;
// 	
// 		WayPoint &w = road[i];
// 		WayPoint &wNext = road[i+1];
// 		float dist = (w.pos-wNext.pos).norm2();
// 		//qDebug() << __FUNCTION__ << "i" << i << "dist" << dist;
// 		if( dist > ROBOT_RADIUS/2 + 50)
// 		{
// 			float l = ROBOT_RADIUS/2/dist;
// 			WayPoint wNew( (w.pos * (1-l)) + (wNext.pos * l));
// 			road.insert(i+1,wNew);
// 			//qDebug() << __FILE__ << __FUNCTION__ << "addPoints:: inserted at" << i+1 << (wNew.pos-road[0].pos).norm2();
// 		}
// 		else if( i>0 and i < road.size()-2 and dist < ROBOT_RADIUS/5)
// 		{
// 			road.removeAt(i+1);
// 			//qDebug() << __FILE__<< __FUNCTION__ << "removed from" << i+1 << dist;
// 		}
// 	}
// 	//qDebug() << "ElasticBand addpoints road size" << road.size();
// }


/**
 * @brief Adds points to the band if two existing ones are too far apart
 * 
 * @param road ...
 * @return void
 */
void ElasticBand::addPoints(WayPoints &road)
{
	Q_ASSERT( road.size()>1 );
		
	for(int i=0; i< road.size()-1; i++) 
	{
		if( i>0 and road[i].isVisible == false )
			break;
	
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		//qDebug() << "i" << i << "dist" << (w.pos-wNext.pos).norm2();
		float dist = (w.pos-wNext.pos).norm2();
		
		if( dist > ROBOT_RADIUS)
		{
			//WayPoint wNew( (w.pos * (T)0.5) + (wNext.pos * (T)0.5));
			float l = ROBOT_RADIUS/dist;
			WayPoint wNew( (w.pos * (1-l)) + (wNext.pos * l));
			road.insert(i+1,wNew);
		//	qDebug() << __FILE__ << __FUNCTION__ << "addPoints:: inserted at" << i+1 << dist;
		}
	}
	//qDebug() << "ElasticBand addpoints road size" << road.size();
}

/**
 * @brief Removes points from the band if two of them are too close
 * 
 * @param road ...
 * @return void
 */
void ElasticBand::cleanPoints(WayPoints &road)
{
	Q_ASSERT( road.size()>1 );
	
	int i;
	for(i=1; i< road.size()-2; i++) // exlude 1 to avoid deleting the nextPoint and last to avoid deleting the target
	{
		//qDebug() << "i" << i << "visible in clean" << road[i].isVisible;
		if( i>0 and road[i].isVisible == false )
			break;
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		//qDebug() << "i" << i << "dist" << (w.pos-wNext.pos).norm2();
		float dist = (w.pos-wNext.pos).norm2();
		if( dist < ROBOT_RADIUS/3. )
		{
			road.removeAt(i+1);
		//	qDebug() << __FILE__<< __FUNCTION__ << "removed from" << i+1 << dist;
		}
	}
}

float ElasticBand::computeForces(WayPoints &road, const RoboCompLaser::TLaserData& laserData)
{
	Q_ASSERT( road.size()>3 );  //CHECK THIS TO ALLOW TWO AND ONE POINTS ROADS
	
	QVec atractionForce(3);
	QVec repulsionForce(3);
	
	//band[0].min = 250; //robot size
	//band.last().min = 250;
	
	QVec jacobian(3);
	float totalChange=0.f;
	
	for(int i=1; i< road.size()-1; i++)  //We need to exclude the current and next points to keep them within robot reach
	{		
		if( road[i].isVisible == false )
			break;
		
		WayPoint &w0 = road[i-1];
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i+1];
		
		//compute linear derivatives
// 		QVec izq = QVec::zeros(3);
// 		QVec der = QVec::zeros(3);
// 		if( i == 0 )
// 		{
// 			der = (w2.pos-w1.pos).normalize();
// 			atractionForce = der;	
// 		}
// 		else if( i == road.size()-1 )
// 		{
// 			izq = (w0.pos - w1.pos).normalize();
// 			atractionForce = izq;
// 		}		
// 		else
// 		{
// 			izq = (w0.pos - w1.pos).normalize();
// 			der = (w2.pos - w1.pos).normalize();
// 			atractionForce = izq + der ;	
// 			//atractionForce = atractionForce - ((w0.pos - w2.pos)) / (w0.pos - w2.pos).norm2(); 
// 		}
		
		//LINEAR FORCE II
		float n = (w0.pos-w1.pos).norm2() / ( (w0.pos-w1.pos).norm2() + w1.initialDistanceToNext );
		atractionForce = (w2.pos - w0.pos)*n - (w1.pos - w0.pos);	
			
		//REPULSION FORCE FROM OBSTACLEs Compute jacobian of free space wrt to x,y
		QVec repulsionForce = QVec::zeros(3);
		
		computeDistanceField(w1, laserData, FORCE_DISTANCE_LIMIT);
		//qDebug() << ball.minDistHasChanged << ball.minDist << ball.minDistAnt;

		float h = DELTA_H;  //CHECK THIS
		if ( ( w1.minDistHasChanged == true) /*and (w1.minDist < 250)*/ )
		{	
			jacobian = QVec::vec3( w1.bMinusX  - w1.bPlusX , 
														 0 ,
									w1.bMinusY  - w1.bPlusY ) * (T)(1.f/(2.f*h));
																
			repulsionForce = jacobian * ( FORCE_DISTANCE_LIMIT - w1.minDist );
		
		//qDebug() << "ElasticBand::computeForces" << i << w1.minDist << w1.pos << jacobian << repulsionForce << atractionForce;
		}
		
		//REPULSION FORCE II
// 		QVec repulsionForce = QVec::zeros(3);
// 		computeDistanceField(w1, laserData, FORCE_DISTANCE_LIMIT);
// 		repulsionForce = w1.minDistPoint * (FORCE_DISTANCE_LIMIT - w1.minDist);
		
		float alpha = -0.7;
		float beta = 0.03;
			
		QVec change = (atractionForce*alpha) + (repulsionForce*beta);		
		
		//Now we remove the tangencial component of the force to avoid recirculation of band points
		//QVec pp = road.getTangentToCurrentPoint().getPerpendicularVector();
		//QVec nChange = pp * (pp * change);
		
		w1.pos = w1.pos - change;
		//w1.pos = w1.pos - nChange;
		
		totalChange = totalChange + change.norm2();
	}
	return totalChange;
}


/**
 * @brief A point of the road is visible if it is between the robot and the laser beam running through it, and if the previous point was visible
 * We go through the laser array until the nearest beam is found
 * @param road ...
 * @param laserData ...
 * @return bool
 */
bool ElasticBand::checkVisiblePoints(WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{	
	Q_ASSERT(road.size()>1);
	
	float maxAngle, minAngle;
	if(laserData[0].angle > laserData.back().angle)
	{		maxAngle = laserData[0].angle;	minAngle = laserData.back().angle;	}
	else 
	{		minAngle = laserData[0].angle;	maxAngle = laserData.back().angle;	}
	
	for(int i=1; i< road.size(); i++)//We leave out the first point wich is usually under the robot
	{		
		//qDebug() << "i" << i << "checking visibility";
	
		WayPoint &w = road[i];	
		w.isVisible = true;
		QVec pr = innermodel->transform("laser", w.pos, "world");
		float angle = atan2(pr.x(),pr.z());
		
		if(((angle < minAngle) or (angle > maxAngle)) and (pr.z()>10)) 
		{
			qDebug() << __FILE__<< __FUNCTION__ << "ElasticBand::checkVisiblePoints - exiting due to angle" << angle << i << pr;
			w.isVisible = false;
			for(int k=i;k<road.size();k++)
				road[k].isVisible = false;
			return false;
		}
			
		uint j;
		float init = laserData[0].angle;
				
		//qDebug() << "angle" << angle << "length" << length << "init" << init << pr << w1.pos << w2.pos;
		for(j=1; j< laserData.size();j++)
		{
			if( laserData[j].angle > init ) //ascending order
			{	
				if ( laserData[j].angle >= angle ) //already there
					break;
			}
			else //descending
			{
				if( laserData[j].angle <= angle )
					break;
			}
		}
		//Check if the point is behind the laser beam end
		w.visibleLaserAngle = laserData[j].angle;
		pr[1]=innermodel->getNode("laser")->getTr()[1];
		w.posInRobotFrame = pr;
		w.visibleLaserDist = laserData[j].dist;
		
		if( pr.norm2() > laserData[j].dist and (pr.z()>10)) // laser beam is smaller than p point and p is beyond the laser. When p is being crossed visibility is compromised
		{
			w.isVisible = false;
			for(int k=i;k<road.size();k++)
				road[k].isVisible = false;

			return false;
		}
	}
	return true;
}

void ElasticBand::computeDistanceField(WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit)
{
	
	ball.minDist = ball.bMinusX = ball.bPlusX = ball.bMinusY = ball.bPlusY = std::numeric_limits<float>::max();
	ball.minDistHasChanged = false;
	
	//QVec c = innermodel->transform("base", ball.pos, "world");
	QVec c = ball.pos;
	c[1] =innermodel->getNode("laser")->getTr()[1]; //Put the y coordinate to laser height so norm() works allright
	int index = -1;
	
	for(uint i=0; i<laserData.size(); i++)
	{
		//QVec l = innermodel->laserToBase("laser",laserData[i].dist, laserData[i].angle);
		QVec l = innermodel->laserToWorld("laser",laserData[i].dist, laserData[i].angle);
	
		float dist = (l-c).norm2();
		
		if( dist < ball.minDist )
		{
			ball.minDist = dist;
			index = i;
		}
		
 		float h = DELTA_H; //Delta 
 		dist = (l-(c - QVec::vec3(h,0,0))).norm2();
 		if( dist < ball.bMinusX ) ball.bMinusX = dist;
		
		dist = (l-(c + QVec::vec3(h,0,0))).norm2();
		if( dist < ball.bPlusX ) ball.bPlusX = dist;
		
		dist = (l-(c - QVec::vec3(0,0,h))).norm2();
		if( dist < ball.bMinusY ) ball.bMinusY = dist;		
		
		dist = (l-(c + QVec::vec3(0,0,h))).norm2();
		if( dist < ball.bPlusY ) ball.bPlusY = dist;				
	}

	QVec lw = innermodel->laserToWorld("laser",laserData[index].dist, laserData[index].angle);
	ball.minDistPoint = (lw - c).normalize();
	
	//qDebug() << "before if" << ball.minDist;
	
		if ( ball.minDist < forceDistanceLimit )
	{
		if ( fabs(ball.minDist - ball.minDistAnt) > 2 ) //mm
		{
			ball.minDistAnt = ball.minDist;
			ball.minDistHasChanged = true;
		}
	}
	else
		ball.minDist = forceDistanceLimit;
}

void ElasticBand::checkBlocked(WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	//Compute max and min angles in laser beam
	float maxAngle, minAngle;
	if(laserData[0].angle > laserData.back().angle)
	{		maxAngle = laserData[0].angle;	minAngle = laserData.back().angle;	}
	else 
	{		minAngle = laserData[0].angle;	maxAngle = laserData.back().angle;	}
		
	road.isBlocked = false;
	for(int i=0; i< road.size()-1; i++)
	{
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i+1];
		float length = (w1.pos -w2.pos).norm2();
		int nSteps = round(length / ROBOT_STEP);
		float step = length / nSteps;
		float landa = step / length; 		
		QVec p(3), pr(3);
		
		//Now we go through the inner points of the line
		for (float k=0; k<=1; k+=landa)
		{
			p = (w1.pos * (T)(1. - k)) + (w2.pos * k);
			pr = innermodel->transform("base", p, "world");
			float angle = atan2(pr.x(),pr.z());
						
			//We go through the laser array until the nearest beam is found			
			if( angle > maxAngle or angle < minAngle) //if point is outside laser beam, ignore
			{
				//qDebug() << pr << angle << maxAngle << minAngle;
 				//qDebug() << "breaking";
				break;
			}
			
			if( pr.norm2() > (ROBOT_RADIUS * 2) ) 	//if point is more than 2 robots away, ignore so far. Myabe substitute for something like "the next three points from current position"
				break;
		
			uint j;
			float init = laserData[0].angle;
			
			//qDebug() << "inner index" << k << "angle" << angle << "length" << length << "init" << init << pr << w1.pos << w2.pos;
			for(j=1; j< laserData.size();j++)
			{
				if( laserData[j].angle > init ) //ascending order
				{	
					if ( laserData[j].angle >= angle ) //already there
						break;
				}
				else //descending
				{
					if( laserData[j].angle <= angle )
						break;
				}
			}

			//now we have the laser index we check if point is before or after the laser reading
			if( pr.norm2() > laserData[j].dist ) // laser beam is smaller than p distance to robot. The path is crossed by an obstacle
			{
				road.isBlocked = true;
				qDebug() << __FILE__ << "Blocked index" << i << "dist robot2point" << pr.norm2() << "laser" << laserData[j].dist << "k" << k << pr 
									<< "laserIndex" << j << "laserAngle" << laserData[j].angle << "angle" << angle;;
				break;
			}		
		}	
	}
}


/**
 * @brief The path between two adjacent nodes is free if the line joining their centers is not crossed by a laser measure * 
 * @param w1 ...
 * @param w2 ...
 * @param laserData ...
 * @return void
 */
bool ElasticBand::computeFreePath(const WayPoint &w1, const WayPoint &w2, const RoboCompLaser::TLaserData &laserData )
{
	QVec p(3);
	bool free = true;
	float length = (w1.pos -w2.pos).norm2();
	float landa = 150. / length; ///CHECK
	p = (w1.pos * (T)(1. - landa)) + (w2.pos * landa);
	QVec pr = innermodel->transform("base", p, "world");
	float angle = atan2(pr.x(),pr.z());
			
	//We go through the laser array until the nearest beam is found
	//Another option is to translate the whole laser array to the bubble reference system
	
	uint i;
	float init = laserData[0].angle;
			
	qDebug() << __FILE__<< __FUNCTION__ << "angle" << angle << "length" << length << "init" << init << pr << w1.pos << w2.pos;
	for(i=1; i< laserData.size();i++)
	{
		if( laserData[i].angle > init ) //ascending order
		{	
			if ( laserData[i].angle >= angle ) //already there
				break;
		}
		else //descending
		{
			if( laserData[i].angle <= angle )
				break;
		}
	}
	
	//Now is the path to the next bubble is blocked we find an alternative free direction or report complete failure.
	//additional criteria: free dir with minimun angle wrt to previous segment aka inertial term
	
	qDebug() << __FILE__ << __FUNCTION__ << "i" << i << pr.norm2()+250. << laserData[i].dist;
	//now we have the k index. we need a simple interpolation among neighboors and the final check
	if( pr.norm2()+250. > laserData[i].dist ) // laser beam is smaller than p point. The path is crossed by an obstacle
	{
		free = false;
	}
	return free;
}


// /**
//  * @brief The path between two adjacent nodes is free if the line joining their centers is not crossed by a laser measure * 
//  * @param w1 ...
//  * @param w2 ...
//  * @param laserData ...
//  * @return void
//  */
// bool ElasticBand::computeFreePath(const WayPoint &w1, const WayPoint &w2, const RoboCompLaser::TLaserData &laserData )
// {
// 	QVec p(3);
// 	float nSteps = 10;
// 	bool free = true;
// 	
// 	for(float landa=0.; landa<=1; landa += 1./nSteps)
// 	{
// 			p = (w1.pos * (T)(1. - landa)) + (w2.pos * landa);
// 			QVec pr = innermodel->transform("base", p, "world");
// 			float angle = atan2(pr.x(),pr.z());
// 			
// 			//We go through the laser array until the nearest beam is found
// 			uint i;
// 			float init = laserData[0].angle;
// 			
// 			//qDebug() << "angle" << angle << "init" << init;
// 			for(i=1; i< laserData.size();i++)
// 			{
// 				if( laserData[i].angle > init ) //ascending order
// 				{	
// 					if ( laserData[i].angle >= angle ) //already there
// 						break;
// 				}
// 				else //descending
// 				{
// 					if( laserData[i].angle <= angle )
// 						break;
// 				}
// 			}
// 			//qDebug() << "i" << i;
// 			//now we have the k index. we need a simple interpolation among neighboors and the final check
// 			if( p.norm2() > laserData[i].dist ) // laser beam is beyond the p point. The path is crossed by an obstacle
// 			{
// 				free = false;
// 				break;
// 			}
// 	}
// 	return free;
// }


/**
 * @brief Compute the length of the chord of two intersecting circles
 *
 * @param b1 ...
 * @param b2 ...
 * @return float
 **/
float ElasticBand::computeIntersectionChord( const WayPoint b1, const WayPoint b2)
{
	float d = (b1.pos-b2.pos).norm2();
	float r1 = b1.minDist;
	float r2 = b2.minDist;

	//qDebug() << b1.pos << b2.pos << d << r1 << r2;
	
	if (d > (r1 + r2)) //not intersecting
	{
		qDebug() << "ComputeIntersectionChord:: Not intersecting";
		return 0;
	}
	if ( d > 0.00001 ) 
	{
		qDebug() << "ComputeIntersectionChord:: chord";
		return (1.f/d) * sqrt((-d+r2-r1)*(-d-r2+r1)*(-d+r2+r1)*(d+r2+r1));
	}
	else //they are equal
	{
		qDebug() << "ComputeIntersectionChord:: Equal";
		return (r1>r2)?r2:r1;
	}
}

bool ElasticBand::checkCollision(WayPoints &road, const RoboCompLaser::TLaserData& laserData, float robotRadius)
{
	for(int i=0; i< road.size()-1; i++)
	{
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i+1];
		float length = (w1.pos -w2.pos).norm2();
		int nSteps = round(length / ROBOT_STEP);
		float step = length / nSteps;
		float landa = step / length; 
		
		QVec p(3), pr(3);
		
		//Now we go through the inner points of the line

		for (float k=0; k <=1; k+=landa)
		{
			//qDebug() << nSteps << landa << length<< k;
			p = (w1.pos * (T)(1. - k)) + (w2.pos * k);
			pr = innermodel->transform("base", p, "world");
			//float minDist = std::numeric_limits<float>::max();
			//qDebug() << "point" << i << w1.pos << w2.pos << p;
			for(uint j=0; j<laserData.size(); j++)
			{
				QVec l = innermodel->laserToBase("laser",laserData[j].dist, laserData[j].angle);
				float dist = (l-pr).norm2();
		
				if( dist < robotRadius ) 
				{	
				
					//Fix the collision and break
					//Estimate the Jacobian first by perturbating the point pr
					float h = 50; //Delta 
					QVec jacobian = QVec::vec3( (l-(pr - QVec::vec3(h,0,0))).norm2() - (l-(pr + QVec::vec3(h,0,0))).norm2(), 
																			 0 ,
																			(l-(pr - QVec::vec3(0,0,h))).norm2() - (l-(pr + QVec::vec3(0,0,h))).norm2()) * (T)(1.f/(2.f*h));
					// exact jump to go along gradient up to 270 away from center											
					// float jump = sqrt((270*270 - pow(l.x()-pr.x(),2) - pow(l.z()-pr.z(),2) ) / pow(jacobian.x() + jacobian.z(),2));
					// qDebug() << "jump" << jump << (l-(pr-(jacobian*jump))).norm2();
					// pr = pr - (jacobian * jump);
																			
					///Linear search along the jacobian direction performs better
					pr = pr - (jacobian * ( (robotRadius - dist)));
					int cont=0;
					while( (l-pr).norm2() < robotRadius or cont > 10)
					{
						pr = pr - jacobian;
						cont++;
					}
					//if (cont>10) the Jac is not working
					
					QVec q = innermodel->transform("world", pr, "base");
					float distQ = (l-pr).norm2();
					//If p==w2, move the point, if not, Insert a new point and move it
					int m;
				
					QVec w2b = w2.pos;
					
					if( (q-w2.pos).max(m) > 100)
					{
						WayPoint w(q);
						w.minDist = dist;
						road.insert(i+1, w);
						road.currentCollisionIndex = i+1;
						qDebug() << "INSERTING point at " << q;
					}
					else
					{
						w2.pos = q;
						w2.minDist = distQ;
						road.currentCollisionIndex = i;
						//We should check here is the new w2.pos collides with other laser beams, in case of, we are probably in a narrow passage and
						//we should report the situation as blocked
					}
					qDebug() << "point" << i << "dist" << dist << "p" << p << "w1" << w1.pos << "w2" << w2b << "q" << q << "road size" << road.size() 
										<< "jac" << jacobian* ( (robotRadius - dist) ) << "distQ" << distQ << "k" << k << "landa" << landa << landa + k << "error" << dist-robotRadius;
					return true;
				}
			}
		}
	}
	return false;
}


///////////////////
/// Continuous smoothing of the road to provide opportunistic skills
///////////////////

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 * 
 * @param list List of poses comprising the path
 * @return void
 */
// void ElasticBand::smoothPath( const WayPoints &road)
// {
// 	bool reachEnd;
// 	
// 	trySegmentToTarget( list.first(), list.last(), reachEnd, NULL, it);
// 
// 	if (reachEnd == true) 
// 	{
// 		if(currentSmoothedPath.contains(list.first()) == false)
// 		  currentSmoothedPath.append(list.first());
// 		if(currentSmoothedPath.contains(list.last()) == false)
// 		  currentSmoothedPath.append(list.last());
// 
// 		return;
// 	}
// 	else		//call again with the first half first and the second half later
// 	{
// 		if(list.size()>2)	   
// 		{
// 		      smoothPath( list.mid(0,list.size()/2 +1));
// 		      smoothPath( list.mid( list.size()/2 , -1 ));
// 		}
// 	}
// }


//bool ElasticBand::collisionDetector( const QVec &point,  InnerModel *innerModel)
//{
// 	//Check if the virtual robot collides with any obstacle
// 	innerModel->updateTransformValues("baseT", point.x(), point.y(), point.z(), 0, 0, 0);
// 	bool hit = false;
// 
// 	foreach( QString name, listCollisionObjects)
// 	{
// 		if (innerModel->collide("baseFake", name) /*or    //We cannot check the other meshes here because they are not on the clone robot. A complete clone is needed
// 			innerModel->collide("barracolumna", name) or 
// 			innerModel->collide("handleftMesh1", name) or 
// 			innerModel->collide("finger_right_2_mesh2", name)*/)
// 		{
// 			hit = true;
// 			break;
// 		}
// 	}
// 	return hit;
// 	
// }

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
// QVec ElasticBand::trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd)
// {
// 	float stepSize = 100.f; //100 mms chunks
// 	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
// 	float step;
// 	
// 	//if too close return target
// 	if (nSteps == 0) 
// 	{
// 		reachEnd = true;
// 		return target;
// 	}
// 	step = 1./nSteps;
// 	
// 	//go along visual ray connecting robot pose and target pos in world coordinates
// 	// l*robot + (1-r)*roiPos = 0
// 	
// 	QVec point(3), pointAnt(3);
// 	float landa = step;
// 	QVec pos(3), front(3);
// 	
// 	pointAnt=origin;
// 	for(uint i=1 ; i<=nSteps; i++)
// 	{
// 		// center of robot position
// 		point = (origin * (1-landa)) + (target * landa);
// 		
// 		//Collision detector
// 		if (collisionDetector( point, innerModel) == true)
// 		{
// 		  reachEnd = false;
// 		  return pointAnt;
// 		}		
// 		 
// 		landa = landa + step;
// 		pointAnt = point;
// 	}
// 	reachEnd= true;
// 	return target;
// }
// 




















