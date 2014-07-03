/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "rcisdraw.h"
#include <boost/concept_check.hpp>

RcisDraw::RcisDraw()
{
	
}

RcisDraw::~RcisDraw()
{

}

void RcisDraw::addMesh_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString parent, const RoboCompInnerModelManager::meshType & m)
{
	try
	{
		innermodelmanager_proxy->addMesh(a.toStdString(),parent.toStdString(), m);
	} 
	catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	{
	//	if (e.text != RoboCompInnerModelManager::NodeAlreadyExists)
		if (e.text != "")
			
		{
			std::cout << e << std::endl;
			qFatal("Probably the parent does not exist!");
		}
		else
		{
			innermodelmanager_proxy->removeNode(a.toStdString());
			innermodelmanager_proxy->addMesh(a.toStdString(),parent.toStdString(), m);
		}
	}
}

void RcisDraw::addTransform_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString b, const RoboCompInnerModelManager::Pose3D &m)
{
	 try
	 {
		innermodelmanager_proxy->addTransform(a.toStdString(),"static",b.toStdString(), m);
	 } 
	 catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	 {
		//if (e.text != RoboCompInnerModelManager::NodeAlreadyExists)
		if (e.text != "")
		{
			qFatal("Probably the parent does not exist!");
		}
		else
		{
			innermodelmanager_proxy->setPoseFromParent(a.toStdString(),m);
		}
	 }
}


void RcisDraw::addPlane_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString b, const RoboCompInnerModelManager::Plane3D &p)
{
	 try
	 {
		innermodelmanager_proxy->addPlane(a.toStdString(),b.toStdString(), p);
	 } 
	 catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	 {
		//if (e.text != RoboCompInnerModelManager::NodeAlreadyExists)
		if (e.text != "")
		{
			qFatal("Probably the parent does not exist!");
		}
		else
		{
			innermodelmanager_proxy->removeNode(a.toStdString());
			innermodelmanager_proxy->addPlane(a.toStdString(),b.toStdString(), p);
		}
	 }
}


void RcisDraw::drawLine(InnerModelManagerPrx innermodelmanager_proxy, QString name, QString parent, const QVec& normalVector, float length, float width, QString texture)
{
	RoboCompInnerModelManager::Plane3D plane;
	plane.px = 0;	plane.py = 0;	plane.pz = 0;	
	plane.thickness = width;	
	plane.width = length;	
	plane.height = width;
	plane.nx = normalVector.x(); plane.ny = normalVector.y(); plane.nz = normalVector.z();
	plane.texture = texture.toStdString();
	addPlane_ignoreExisting(innermodelmanager_proxy, name, parent, plane);
	//qDebug() << "Inserting line and transform" << name <<" in RCIS";
	
}

void RcisDraw::removeObject(InnerModelManagerPrx innermodelmanager_proxy, QString name)
{
	try
	 {
		 innermodelmanager_proxy->removeNode(name.toStdString());
	 } 
	 catch (const RoboCompInnerModelManager::InnerModelManagerError &e )
	 {
		//	if( e.err != RoboCompInnerModelManager::NodeAlreadyExists)
				//qDebug() << "Something bad happened when deleting Node " << name;
	 }
}
