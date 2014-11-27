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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
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
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel");
		if( QFile::exists(QString::fromStdString(par.value)) )
			innerModel = new InnerModel(par.value);
		else
		{
			std::cout << "Innermodel path " << par.value << " not found. "; qFatal("Abort");
		}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	
	try {  differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, can't communicate with robot proxy");}
	// DESCOMENTAR:!!!!!!!!!!!!!!
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; qFatal("Aborting, can't communicate with laser proxy");}

	innerModel->updateTranslationValues("robot", bState.x, 0, bState.z);   //"robot" should be an external parameter
	innerModel->updateRotationValues("robot", 0, bState.alpha, 0);

	timer.start(Period);
	return true;
};