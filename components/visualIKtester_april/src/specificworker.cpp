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
	currentTag.tx = 0;
	currentTag.ty = 0;
	currentTag.tz = 0;
	sendPoseFlag  = false;
	INITIALIZED   = false;
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
		if (QFile(QString::fromStdString(par.value)).exists() == true)
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
	if(INITIALIZED) 
	{
		updateInnerModel();
		
		QMutexLocker ml(mutex);
		if(sendPoseFlag == true)
		{
			//PREPARAMOS EL TARGET Y LO ENVIAMOS AL VISUALIK
			//1) PASAMOS EL TAG AL SISTEMA DE REFERENCIA DEL MUNDO: ALINEAMOS EJES CON ROOT:
			InnerModelNode *nodeParent = innerModel->getNode("rgbd_transform");
			if(innerModel->getNode("tag_from_rgbd") == NULL)
			{
				nodeMarca = innerModel->newTransform("tag_from_rgbd", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
				nodeParent->addChild(nodeMarca);
				nodeMarca2 = innerModel->newTransform("tag_from_rgbd2", "static", nodeMarca, 0, 0, 0,       0., 0, 0,      0.);
				nodeMarca->addChild(nodeMarca2);
			}
			const QString SH1 = "tag_from_rgbd";
			const QString SH2 = "tag_from_rgbd2";
			innerModel->updateTransformValues(SH1, currentTag.tx, currentTag.ty, currentTag.tz,  currentTag.rx, currentTag.ry, currentTag.rz);
			innerModel->updateTransformValues(SH2,             0,             0,             0,        -M_PI_2,             0,             0);
	
			QVec tagPose_inRoot = innerModel->transform6D("root", SH2); //ya tenemos el target en el root
			
			//2) MODIFICAMOS TARGET PARA QUE SE QUEDE LA MANO UN POCO MAS A LA DERECHA
			Pose6D target;
			target.x = tagPose_inRoot.x();
			target.y = tagPose_inRoot.y();
			target.z = tagPose_inRoot.z()-150;
			// fijamos las rotaciones:
			target.rx = 0;
			target.ry = -1.5707963267948966;
			target.rz = 0;
			
			WeightVector weights;
			weights.x  = 1;   weights.y  = 1;   weights.z  = 1;
			weights.rx = 0.1; weights.ry = 0.1; weights.rz = 0.1;
			
			qDebug()<<"|| TARGET: "<< target.x <<" "<< target.y <<" "<<target.z<<",   "<<target.rx<<" "<<target.ry<<" "<<target.rz<<" ||";
			
			//2) ENVIAMOS EL TARGET AL VIK
			inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);
			sendPoseFlag = false;
		}
	}
}


void SpecificWorker::newAprilTag(const tagsList &tags)
{
	static bool firstTag = true;
	const int   treshold = 100;
	
	for (auto tag : tags)
	{
		if (tag.id == 25)
		{
			QMutexLocker ml(mutex);
			QVec newTag      = QVec::vec3(tag.tx, tag.ty, tag.tz);
			QVec oldTag      = QVec::vec3(currentTag.tx, currentTag.ty, currentTag.tz);
			const float dist = (newTag-oldTag).norm2();
			
			qDebug()<<"||  NEW TAG: "<<newTag<<" OLD TAG: "<<oldTag<<"    DISTANCE: "<<dist;
			
			if (dist > treshold or firstTag)
			{
				qDebug()<<"Envio";
				if (firstTag) 
					firstTag = false;
				
				currentTag   = tag;
				sendPoseFlag = true;
			}
		}	
	}
}

void SpecificWorker::updateInnerModel()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout<<"--> Excepción en actualizar InnerModel"<<std::endl;
	}
}






