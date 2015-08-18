/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
	QMutexLocker ml(mutex);
	sendPoseFlag = false;
	INITIALIZED  = false;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
		}
		else
			qFatal("Exiting now.");
	}
	catch(std::exception e) { qFatal("Error reading Innermodel param");}
	
	timer.start(Period);

	QMutexLocker ml(mutex);
	INITIALIZED = true;
	
	return true;
}

void SpecificWorker::compute()
{
	//TODO HACER MAQUINA DE ESTADO QUE BUSQUE TAZA Y LA COJA
	QMutexLocker ml(mutex);
	if(sendPoseFlag == true)
	{
		//PREPARAMOS EL TARGET Y LO ENVIAMOS AL VISUALIK
		//1) PASAMOS EL TAG AL SISTEMA DE REFERENCIA DEL MUNDO
		QVec tagPose = QVec::vec3(currentTag.tx, currentTag.ty, currentTag.tz);
		QVec tagInRoot = innerModel->transform("root", tagPose, "rgbd");
		//2) ENVIAMOS EL TARGET AL VIK
		Pose6D target;
		target.x = tagInRoot.x();
		target.y = tagInRoot.y()+100;
		target.z = tagInRoot.z();
		// fijamos las rotaciones:
		target.rx = 0;
		target.ry = -1.56;
		target.rz = -3.1416;
		
		WeightVector weights;
		weights.x = 1; weights.y = 1; weights.z = 1;
		weights.rx = 0.1; weights.ry = 0.1; weights.rz = 0.1;
		
		inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);
	}
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	int umbral = 10;
	
	for (auto tag : tags)
	{
		if (tag.id == 11)
		{
			QMutexLocker ml(mutex);
			QVec newTag = QVec::vec3(tag.tx, tag.ty, tag.tz);
			QVec oldTag = QVec::vec3(currentTag.tx, currentTag.ty, currentTag.tz);
			if((newTag-oldTag).norm2() > umbral)
				currentTag = tag;
		}	
	}
}






