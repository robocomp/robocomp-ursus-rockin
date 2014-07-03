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

SpecificWorker::SpecificWorker(MapPrx& mprx, QWidget *parent) : GenericWorker(mprx)
{
	this->params = params;
	
	//innerModel = new InnerModel("/home/robocomp/robocomp/Files/InnerModel/betaWorld.xml");  ///CHECK IT CORRESPONDS TO RCIS
	innerModel = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world/rockin.xml");  ///CHECK IT CORRESPONDS TO RCIS
	
	innerModel->setUpdateTranslationPointers("robot", &(bState.x), NULL, &(bState.z));
	innerModel->setUpdateRotationPointers("robot", NULL, &(bState.alpha), NULL);
	
	//moveBoxes();
 
	try { differentialrobot_proxy->getBaseState(bState); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	try { laserData = laser_proxy->getLaserData(); }
	catch(const Ice::Exception &ex) { cout << ex << endl; }
	
	innerModel->update();

	planner = new Planner(innerModel);

	//cleanWorld();
	
	//road.readRoadFromFile(innerModel, "puntos2.txt");
	
	//Set target
	target = QVec::vec3(5000,0,-6500);

	//Draw target as red box	
// 	RoboCompInnerModelManager::Plane3D plane;
// 	plane.px = target.x(); plane.py = 200; plane.pz = target.z(); plane.nx = 1; plane.texture = "#990000"; plane.thickness = 150; plane.height = plane.width = 100;
// 	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "floor", plane);
	
	//Plan path
	qDebug("Planning ...");
//	drawThinkingRobot("red");
	planner->computePath(target);
	
	if(planner->getPath().size() == 0)
		qFatal("Path NOT found");

	
//	drawThinkingRobot("green");
	//planner->drawTree(innermodelmanager_proxy);
	road.readRoadFromList( planner->getPath() );
	road.print();
	road.draw(innermodelmanager_proxy, innerModel);
	
	Q_ASSERT(road.size()>1);

	elasticband = new ElasticBand(innerModel, innermodelmanager_proxy);
	//elasticband->setRoad(road, laserData);
	elasticband->addPoints(road);
	
	pointstoroad = new PointsToRoad(innerModel, innermodelmanager_proxy);
	pointstoroad->setRoad(road);
	
	controller = new Controller(2);

	sleep(1);
	
//	road.draw(innermodelmanager_proxy, innerModel);
		
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::compute( )
{
	try{	differentialrobot_proxy->getBaseState(bState);  }  
	catch(const Ice::Exception &ex){ cout << ex << endl;};
	try{	laserData = laser_proxy->getLaserData();}  
	catch(const Ice::Exception &ex){ cout << ex << endl;};
	
	moveBoxes(); 
	
	innerModel->update();
		
	elasticband->update( road, laserData );
	
	elasticband->update( road, laserData );
	
	pointstoroad->update(road);
	
  controller->update(differentialrobot_proxy, road);
	
	road.draw(innermodelmanager_proxy, innerModel);	
	
	
	if(road.finish == true)
	{
		//Draw target as red box	
		RoboCompInnerModelManager::Plane3D plane;
		plane.px = target.x(); plane.py = 200; plane.pz = target.z(); plane.nx = 1; plane.texture = "#009900"; plane.thickness = 150; plane.height = plane.width = 100;
		RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "target", "floor", plane);
		qFatal("GOODBYE, FINISHED ROAD");
	}
	
	if(road.requiresReplanning == true)
	{
		qDebug("STUCK, PLANNING REQUIRED");
		qDebug("Planning ...");
		drawThinkingRobot("red");
		innerModel->update();
		bool havePlan = planner->computePath( target );
	
		if(havePlan == false or planner->getPath().size() == 0)
			qFatal("NO PLAN AVAILABLE");
		else
		{
			drawThinkingRobot("green");
			road.readRoadFromList( planner->getPath() );
			road.requiresReplanning = false;
			elasticband->addPoints(road);  //SEND THIS TO A RESET
			road.computeDistancesToNext();
		}
	}
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	//qDebug() << QString::fromStdString(params["PointsFile"].value);
	timer.start(100);
	
	return true;
};

void SpecificWorker::moveBoxes()
{
	static bool firstTime = true;
	static bool flip = true;
	static QTime reloj = QTime::currentTime();
	
	RoboCompInnerModelManager::Pose3D pose1,pose2,pose3,pose4, pose5, pose6, pose7, pose8, pose9, pose10, pose11, pose12, pose13, pose14, pose15, pose16;
	bzero(&pose1, sizeof(pose1))  ; pose1.x= 450;  pose1.z=-1000; pose1.y=100;
	bzero(&pose2, sizeof(pose2) ) ; pose2.x= 450;  pose2.z=-300; pose2.y=100;
	bzero(&pose3, sizeof(pose3) ) ; pose3.x= 450;  pose3.z=400; pose3.y=100;
	bzero(&pose4, sizeof(pose4))  ; pose4.x= 450;  pose4.z=1100; pose4.y=100;
	bzero(&pose5, sizeof(pose5))  ; pose5.x= 450;  pose5.z=1800; pose5.y=100;
	
	bzero(&pose6, sizeof(pose6))  ; pose6.x=-300;  pose6.z=1800; pose6.y=100;
	bzero(&pose7, sizeof(pose7))  ; pose7.x=-1000;  pose7.z=1800; pose7.y=100;
	bzero(&pose8, sizeof(pose8))  ; pose8.x=-1700;  pose8.z=1800; pose8.y=100;
	bzero(&pose9, sizeof(pose9))  ; pose9.x=-1700;  pose9.z=1200; pose9.y=100;
	bzero(&pose10, sizeof(pose10)); pose10.x=-1700;  pose10.z=600; pose10.y=100;
	bzero(&pose11, sizeof(pose11)); pose11.x=-1700;  pose11.z=0; pose11.y=100;
	bzero(&pose12, sizeof(pose12)); pose12.x=-1150;  pose12.z=0; pose12.y=100;
	bzero(&pose13, sizeof(pose13)); pose13.x=-1100;  pose13.z=-900; pose13.y=100;
	bzero(&pose14, sizeof(pose14)); pose14.x=-500;  pose14.z= 0; pose14.y=100;
	bzero(&pose15, sizeof(pose15)); pose15.x=-450;  pose15.z= 750; pose15.y=100;
	bzero(&pose16, sizeof(pose16)); pose16.x=-350;  pose16.z=-950; pose16.y=100;
		
// 	RoboCompInnerModelManager::meshType mesh1;
// 	std::string texture = "/home/robocomp/robocomp/Files/osgModels/Textures/Metal.jpg";
// 	mesh1.pose = pose1;
// 	mesh1.scaleX = mesh1.scaleY = mesh1.scaleZ = 400;
	
// 	mesh1.meshPath = texture;
// 	mesh1.render = 0;
	
	if( firstTime == true )
	{
		reloj.start();
		try
		{
			innermodelmanager_proxy->setPoseFromParent("caja1", pose1);
			innerModel->updateTranslationValues("caja1", pose1.x, pose1.y, pose1.z);
			innermodelmanager_proxy->setPoseFromParent("caja2", pose2);
			innerModel->updateTranslationValues("caja2", pose2.x, pose2.y, pose2.z);
			innermodelmanager_proxy->setPoseFromParent("caja3", pose3);
			innerModel->updateTranslationValues("caja3", pose3.x, pose3.y, pose3.z);
			innermodelmanager_proxy->setPoseFromParent("caja4", pose4);
			innerModel->updateTranslationValues("caja4", pose4.x, pose4.y, pose4.z);
			innermodelmanager_proxy->setPoseFromParent("caja5", pose5);
			innerModel->updateTranslationValues("caja5", pose5.x, pose5.y, pose5.z);
			innermodelmanager_proxy->setPoseFromParent("caja6", pose6);
			innerModel->updateTranslationValues("caja6", pose6.x, pose6.y, pose6.z);
			innermodelmanager_proxy->setPoseFromParent("caja7", pose7);
			innerModel->updateTranslationValues("caja7", pose7.x, pose7.y, pose7.z);
			innermodelmanager_proxy->setPoseFromParent("caja8", pose8);
			innerModel->updateTranslationValues("caja8", pose8.x, pose8.y, pose8.z);
			innermodelmanager_proxy->setPoseFromParent("caja9", pose9);
			innerModel->updateTranslationValues("caja9", pose9.x, pose9.y, pose9.z);
			innermodelmanager_proxy->setPoseFromParent("caja10", pose10);
			innerModel->updateTranslationValues("caja10", pose10.x, pose10.y, pose10.z);
			innermodelmanager_proxy->setPoseFromParent("caja11", pose11);
			innerModel->updateTranslationValues("caja11", pose11.x, pose11.y, pose11.z);
			innermodelmanager_proxy->setPoseFromParent("caja12", pose12);
			innerModel->updateTranslationValues("caja12", pose12.x, pose12.y, pose12.z);
			innermodelmanager_proxy->setPoseFromParent("caja13", pose13);
			innerModel->updateTranslationValues("caja13", pose13.x, pose13.y, pose13.z);
			innermodelmanager_proxy->setPoseFromParent("caja14", pose14);
			innerModel->updateTranslationValues("caja14", pose14.x, pose14.y, pose14.z);
			innermodelmanager_proxy->setPoseFromParent("caja15", pose15);
			innerModel->updateTranslationValues("caja15", pose15.x, pose15.y, pose15.z);
			innermodelmanager_proxy->setPoseFromParent("caja16", pose16);
			innerModel->updateTranslationValues("caja16", pose16.x, pose16.y, pose16.z);
			
		} 
		catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
		{ std::cout << e << std::endl; }
		firstTime = false;
	}	
	
	long int maxTime = 9000000;
	static bool first = true;
 
	if( reloj.elapsed() > maxTime and first== true)
	{
		try
		{
			innermodelmanager_proxy->getPoseFromParent("caja3", pose3);		
			if( pose3.x < 2500 ) 
			{
				pose3.x += 35;
				pose3.z -= 10;
			}
			
			innermodelmanager_proxy->setPoseFromParent("caja3", pose3);
			innerModel->updateTranslationValues("caja3", pose3.x, pose3.y, pose3.z);	
		} 
		catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
		{ std::cout << e << std::endl; }
		//first = false;
	}
	
}

void SpecificWorker::cleanWorld()
{
	qDebug() << "SpecificWorker::CleaningWorld()";
	//RcisDraw::removeObject(innermodelmanager_proxy, "nose");
	for (int i = 0 ; i < 80; i++) 
	{
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_line"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_lineA"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_mesh"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i) + "_point"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_mesh"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i) + "_line"));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("b_" + QString::number(i)));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("p_" + QString::number(i)));
		RcisDraw::removeObject(innermodelmanager_proxy, QString("t_" + QString::number(i)));
	}
}

void SpecificWorker::drawThinkingRobot(const QString &color)
{
	RoboCompInnerModelManager::Plane3D p;
	p.px = 0;
	p.py = 210;
	p.pz = -200;
	p.nx = 0;
	p.ny = 1;
	p.nz = 0;
	p.height = 250;
	p.width = 250;
	p.thickness = 20;
	if(color == "red")
		p.texture = "#FF0000";
	else if (color == "green")
		p.texture = "#00FF00";
		
	RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, "redHat", "base",p);
	
}
