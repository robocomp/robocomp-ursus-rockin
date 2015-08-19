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
	currentTag.tx = 0;
	currentTag.ty = 0;
	currentTag.tz = 0;
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
	updateInnerModel();
	//TODO HACER MAQUINA DE ESTADO QUE BUSQUE TAZA Y LA COJA
	QMutexLocker ml(mutex);
	if(sendPoseFlag == true)
	{
		//PREPARAMOS EL TARGET Y LO ENVIAMOS AL VISUALIK
		//1) PASAMOS EL TAG AL SISTEMA DE REFERENCIA DEL MUNDO
		QVec tagPose = QVec::vec3(currentTag.tx, currentTag.ty, currentTag.tz);
		tagPose.print("dddddddd");
		QVec tagInRoot = innerModel->transform("root", tagPose, "rgbd");
		//2) ENVIAMOS EL TARGET AL VIK
		Pose6D target;
		target.x = tagInRoot.x();
		target.y = tagInRoot.y();
		target.z = tagInRoot.z()-150;
		// fijamos las rotaciones:
		target.rx = 0;
		target.ry = -1.5707963267948966;
		target.rz = 0;
		
		WeightVector weights;
		weights.x = 1; weights.y = 1; weights.z = 1;
		weights.rx = 0.1; weights.ry = 0.1; weights.rz = 0.1;
		
		qDebug()<<"----->"<< target.x <<" "<< target.y <<" "<<target.z;
		//qFatal("FIN");
		inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);
		sendPoseFlag = false;
	}
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	static bool first = true;
	int umbral = 50;
	
	for (auto tag : tags)
	{
		if (tag.id == 31)
		{
			QMutexLocker ml(mutex);
			QVec newTag = QVec::vec3(tag.tx, tag.ty, tag.tz);
			QVec oldTag = QVec::vec3(currentTag.tx, currentTag.ty, currentTag.tz);
			newTag.print("nos llega esto");
			const float dist = (newTag-oldTag).norm2();
			if (dist > umbral or first)
			{
				if (first) first = false;
				currentTag = tag;
				sendPoseFlag = true;
				inversekinematics_proxy->stop("RIGHTARM");
			}
		}	
	}
}


void SpecificWorker::updateInnerModel()
{
	qDebug() << "-------- update";
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		//	qDebug() << QString::fromStdString(j.first) << j.second.pos;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
	}
}





