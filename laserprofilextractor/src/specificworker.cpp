/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
// 	static int minX = 0;   	//mm
// 	static int maxX = 8000;
// 	static int minZ = -8000;
// 	static int maxZ = 0;
// 	static int step = 100;  //mm
// 	
// 	for (int x=minX; x<maxX; x+=step) 
// 	{
// 		for(int z=minZ; z<maxZ; z+=step)
// 		{
// 			//move the robot
// 			InnerModelManager::Pose3D pose;
// 			pose.x = x; pose.y = 10; pose.z = z;
// 			pose.rx = 0; pose.ry = 0; pose.rz = 0;
// 			innermodelmanager_proxy->setPose("robot", pose);
// 			//wait for RCIS to update
// 			usleep(300000);
// 			//check if we have a reading from Cuba
// 			
// 			//if new reading process data and store
// 			
// 		}
// 	}
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//timer.start(Period);
	return true;
};

/**
 * @brief Merge and store features
 * 
 * @return void
 */
void SpecificWorker::processFeatures(RoboCompCuba2Dnaturallandmarks::Features features)
{
	try
	{
		//innermodelmanager_proxy->addCUBAFeature();
		qDebug() << __FUNCTION__ << "CUBA: " << "Segments:" << features.s.size() << "Circles:" << features.c.size() << "Points:" << features.p.size();
	}
	catch(Ice::Exception ex)
	{}
}



/////////////////////////////////////////////////77
//// SUBSCRIPTION
//////////////////////////////////////////////////

void SpecificWorker::newCubaFeatureList(const RoboCompCuba2Dnaturallandmarks::Features &features)
{
	processFeatures(features);
}
