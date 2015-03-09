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

ElasticBand::ElasticBand()
{
}

ElasticBand::~ElasticBand()
{
}

bool ElasticBand::update(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData, const CurrentTarget &currentTarget, uint iter)
{
	//qDebug() << __FILE__ << __FUNCTION__ << "road size"<<  road.size();
	if( road.isFinished() == true )
			return false;

	//Tags all points in the road ar visible or blocked, depending on laser visibility. Only visible points are processed in this iteration
	checkVisiblePoints(innermodel, road, laserData);	
	 
	//shortCut(road);
	
	//Add points to achieve an homogenoeus chain
 	addPoints(road, currentTarget);

	//Remove point too close to each other
 	cleanPoints(road);
	
	//Compute the scalar magnitudes
	computeForces(innermodel, road, laserData); 	 
		
	//Delete half the tail behind, if greater than 6, to release resources
	if( road.getOrderOfClosestPointToRobot() > 6)
	{
		for(auto it = road.begin(); it != road.begin() + (road.getOrderOfClosestPointToRobot() / 2); ++it)
			road.backList.append(it->pos);
		road.erase(road.begin(), road.begin() + (road.getOrderOfClosestPointToRobot() / 2));
	}
	return true;
}

bool ElasticBand::checkIfNAN(const WayPoints &road)
{
	for(auto it = road.begin(); it != road.end(); ++it)
		if( isnan(it->pos.x()) or isnan(it->pos.y()) or isnan(it->pos.z()) )
		{
			road.print();
			return true;
		}
	return false;
}

bool ElasticBand::shortCut(WayPoints &road)  //NO FUNCIONA
{
	// Check if the target is visible and go straight to it
	static bool inhibit = false;
	static int veces = 0;
	int offset = road.size()/3;
	
	if( road.last().isVisible and (inhibit == false ))   //Probar a borrar fr 2/3 para alante
	{
		for( WayPoints::iterator it = road.begin()+offset; it != road.end()-1; it = road.erase(it));
		inhibit = true;  //periodo refractario
	}
	if( inhibit )
	{
		veces = veces + 1;
		veces = veces % 50;  //aprox 5 segs
	}
	if( veces == 0 )
		inhibit = false;
	return true;
}

/**
 * @brief Adds points to the band if two existing ones are too far apart (ROBOT_RADIUS)
 * 
 * @param road ...
 * @return void
 */
void ElasticBand::addPoints(WayPoints& road, const CurrentTarget& currentTarget)
{		
	//qDebug() << __FUNCTION__ ;
	int offset = 1;
	
	for(int i=0; i< road.size()-offset; i++)	
	{
		if( i>0 and road[i].isVisible == false )
			break;
	
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		float dist = (w.pos-wNext.pos).norm2();
		if( dist > ROAD_STEP_SEPARATION)  //SHOULD GET FROM IM
		{
			float l = 0.9*ROAD_STEP_SEPARATION/dist;   //Crucial que el punto se ponga mas cerca que la condición de entrada
			WayPoint wNew( (w.pos * (1-l)) + (wNext.pos * l));
			road.insert(i+1,wNew);
		}
	}
	
	//ELIMINATED AS REQUESTED BY MANSO
	//Move point before last to orient the robot. This works but only if the robots approaches from the lower quadrants
	//The angle formed by this point and the last one has to be the same es specified in the target
	//We solve this equations for (x,z)
	// (x' -x)/(z'-z) = tg(a) = t
	// sqr(x'-x) + sqr(z'-z) = sqr(r)
	// z = z' - (r/(sqrt(t*t -1)))
	// x = x' - r(sqrt(1-(1/t*t+1)))
	// 	if( (currentTarget.hasRotation() == true) and (road.last().hasRotation == false) )
	// 	{
	// 		qDebug() << __FUNCTION__ << "computing rotation" << road.last().pos;
	// 		float radius = 500;
	// 		float ta = tan(currentTarget.getRotation().y());
	// 		float xx = road.last().pos.x() - radius*sqrt(1.f - (1.f/(ta*ta+1)));
	// 		float zz = road.last().pos.z() - (radius/sqrt(ta*ta+1));
	// 		WayPoint wNew( QVec::vec3(xx,road.last().pos.y(),zz) );
	// 		road.insert(road.end()-1,wNew);
	// 		road.last().hasRotation = true;
	// 		qDebug() << __FUNCTION__ << "after rotation" << wNew.pos << currentTarget.getRotation().y() << ta;
	// 	
	// 	}
	//else
		//qDebug() << road.last().hasRotation << road.last().pos << (road.end()-2)->pos << currentTarget.getRotation().y();
	
}

/**
 * @brief Removes points from the band if two of them are too close, ROBOT_RADIUS/3.
 * 
 * @param road ...
 * @return void
 */
void ElasticBand::cleanPoints(WayPoints &road)
{
	int i;
	int offset=2;
	//if( road.last().hasRotation ) offset = 3; else offset = 2;
	
	for(i=1; i< road.size()-offset; i++) // exlude 1 to avoid deleting the nextPoint and the last two to avoid deleting the target rotation
	{
		if( road[i].isVisible == false )
			break;
		WayPoint &w = road[i];
		WayPoint &wNext = road[i+1];
		
		float dist = (w.pos-wNext.pos).norm2();
		if( dist < ROAD_STEP_SEPARATION/3. )
		{
			road.removeAt(i+1);
		}
	}
}

/**
 * @brief Computes the forces exerted on the elements of the road
 * 
 * @param road ...
 * @param laserData ...
 * @return float
 */
float ElasticBand::computeForces(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData& laserData)
{
	if(road.size() < 3 )
		return 0;
	
	QVec atractionForce(3);
	QVec repulsionForce(3);
	
	//band[0].min = 250; //robot size
	//band.last().min = 250;
	
	QVec jacobian(3);
	float totalChange=0.f;
	int lastP;					// To avoid moving the rotation element attached to the last
	if( road.last().hasRotation )
		lastP = road.size()-2;
	else
		lastP = road.size()-1;
	for(int i=1; i< lastP; i++) 
	{
		if( road[i].isVisible == false )
			break;
		
		WayPoint &w0 = road[i-1];
		WayPoint &w1 = road[i];
		WayPoint &w2 = road[i+1];
			
		//LINEAR FORCE II
		float n = (w0.pos-w1.pos).norm2() / ( (w0.pos-w1.pos).norm2() + w1.initialDistanceToNext );
		atractionForce = (w2.pos - w0.pos)*n - (w1.pos - w0.pos);	
			
		//REPULSION FORCE FROM OBSTACLEs Compute jacobian of free space wrt to x,y
		QVec repulsionForce = QVec::zeros(3);
		
		computeDistanceField(innermodel, w1, laserData, FORCE_DISTANCE_LIMIT);
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
		
		float alpha = -0.4;//0.6
		float beta = 0.80; //0.09
			
		QVec change = (atractionForce*alpha) + (repulsionForce*beta);		
		
		if(isnan(change.x()) or isnan(change.y()) or isnan(change.z()))
		{
			road.print();
			qDebug() << atractionForce << repulsionForce;
			qFatal("change");	
		}
		//Now we remove the tangencial component of the force to avoid recirculation of band points
		//QVec pp = road.getTangentToCurrentPoint().getPerpendicularVector();
		//QVec nChange = pp * (pp * change);
		
		w1.pos = w1.pos - change;		
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
bool ElasticBand::checkVisiblePoints(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{	
	assert(road.size()>1);
	assert(laserData.size() > 0);
	
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
		//Angle of point in laser plane
		float angle = atan2(pr.x(),pr.z());
		//Check for outofbounds
		if(((angle < minAngle) or (angle > maxAngle)) and (pr.z()>10)) 
		{
			//qDebug() << __FILE__<< __FUNCTION__ << "ElasticBand::checkVisiblePoints - exiting due to angle" << angle << i << pr;
			w.isVisible = false;
			for(int k=i;k<road.size();k++)
				road[k].isVisible = false;
			return false;
		}
			
		//Find laser index corresponding to "angle"
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

void ElasticBand::computeDistanceField(InnerModel *innermodel, WayPoint &ball, const RoboCompLaser::TLaserData &laserData, float forceDistanceLimit)
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

void ElasticBand::checkBlocked(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData &laserData)
{
	//Compute max and min angles in laser beam
	float maxAngle, minAngle;
	if(laserData[0].angle > laserData.back().angle)
	{	maxAngle = laserData[0].angle;	minAngle = laserData.back().angle;	}
	else 
	{	minAngle = laserData[0].angle;	maxAngle = laserData.back().angle;	}
		
	road.setBlocked( false );
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
				road.setBlocked(true);
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
bool ElasticBand::computeFreePath(InnerModel *innermodel, const WayPoint &w1, const WayPoint &w2, const RoboCompLaser::TLaserData &laserData )
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


bool ElasticBand::checkCollision(InnerModel *innermodel, WayPoints &road, const RoboCompLaser::TLaserData& laserData, float robotRadius)
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


//NECESITAMOS OTRA COMPROBACION DE BLOCKED
	
//  	if( road[1].isVisible == false)
//  	{
// 		qDebug() << __FILE__ << __FUNCTION__ << "ElasticBand::update NextPoint is NOT visible or road BLOCKED";
// 		road.requiresReplanning = true;
//  		return false;
//  	}
	
	///Instead of the Jacobian wrt the point distance field, a better approach would be the Jacobian wrt the real shape of the robot. 
	///Mindist would be between robot pose at point, including angle, and laser field
	
	//Computes repulsive and attractive forces and moves the road elements accordingly
// 	computeForces(road, laserData); 	 
// 	
//  	addPoints(road);
// 	//adjustPoints(road);
//  	cleanPoints(road);










