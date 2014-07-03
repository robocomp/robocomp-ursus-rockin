
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


#ifndef RCISDRAW_H
#define RCISDRAW_H

#include <QObject>
#include <qmat/QMatAll>
#include "qline2d.h"
#include <InnerModelManager.h>

using namespace RMat;
using namespace RoboCompInnerModelManager;

class RcisDraw
{
public:
	RcisDraw();
	~RcisDraw();

	static void addMesh_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString parent, const RoboCompInnerModelManager::meshType & m);
	static void addTransform_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString b, const RoboCompInnerModelManager::Pose3D & m);
	static void addPlane_ignoreExisting(InnerModelManagerPrx innermodelmanager_proxy, QString a, QString b, const RoboCompInnerModelManager::Plane3D &p);
	static void drawLine(InnerModelManagerPrx innermodelmanager_proxy, QString name, QString parent, const QVec &normalVector, float length, float width, QString texture = "#550000");
	static void removeObject(InnerModelManagerPrx innermodelmanager_proxy, QString name);

};

#endif // RCISDRAW_H
