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

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// http://cdn.intechopen.com/pdfs-wm/465.pdf
	R  = QString::fromStdString(params["UrsusBase.WheelRadius"].value).toFloat();
	l1 = QString::fromStdString(params["UrsusBase.DistAxes"].value   ).toFloat() / 2.;
	l2 = QString::fromStdString(params["UrsusBase.AxesLength"].value ).toFloat() / 2.;

	{
		const float R4 = R/4.;
		const float ill = 1. / (l1 + l2);

		M_wheels_2_vels = QMat(3, 4);
		M_wheels_2_vels(0,0) = +R4;
		M_wheels_2_vels(0,1) = +R4;
		M_wheels_2_vels(0,2) = +R4;
		M_wheels_2_vels(0,3) = +R4;

		M_wheels_2_vels(1,0) = +R4;
		M_wheels_2_vels(1,1) = -R4;
		M_wheels_2_vels(1,2) = -R4;
		M_wheels_2_vels(1,3) = +R4;
		
		M_wheels_2_vels(2,0) = R4*(-ill);
		M_wheels_2_vels(2,1) = R4*(+ill);
		M_wheels_2_vels(2,2) = R4*(-ill);
		M_wheels_2_vels(2,3) = R4*(+ill);
	}

	{
		const float iR = 1./R;
		const float ll = (l1 + l2);

		M_vels_2_wheels = QMat(4);
		M_vels_2_wheels(0,0) = +iR;
		M_vels_2_wheels(1,0) = +iR;
		M_vels_2_wheels(2,0) = +iR;
		M_vels_2_wheels(3,0) = +iR;

		M_vels_2_wheels(0,1) = +iR;
		M_vels_2_wheels(1,1) = -iR;
		M_vels_2_wheels(2,1) = -iR;
		M_vels_2_wheels(3,1) = +iR;
		
		M_vels_2_wheels(0,2) = iR*(-ll);
		M_vels_2_wheels(1,2) = iR*(+ll);
		M_vels_2_wheels(2,2) = iR*(-ll);
		M_vels_2_wheels(3,2) = iR*(+ll);
	}
	
	timer.start(Period);
	return true;
}

void SpecificWorker::SpecificWorker::compute( )
{
	const float w1 = 0; // front left
	const float w2 = 0; // front right
	const float w3 = 0; // back left
	const float w4 = 0; // back right

	QVec delta = M_wheels_2_vels * QVec::vec4(w1, w2, w3, w4);
}


void SpecificWorker::getBaseState(::RoboCompOmniRobot::TBaseState &state)
{
}

void SpecificWorker::getBasePose(::Ice::Int &x, ::Ice::Int &z, ::Ice::Float &alpha)
{
}

void SpecificWorker::setSpeedBase(::Ice::Float advx, ::Ice::Float advz, ::Ice::Float rotv)
{
	QVec wheels = M_vels_2_wheels * QVec::vec3(advz, advx, rotv);
	setWheels(wheels);
}

void SpecificWorker::stopBase()
{
	setWheels(QVec::vec4(0,0,0,0));
}

void SpecificWorker::resetOdometer()
{
}

void SpecificWorker::setOdometer(const ::RoboCompOmniRobot::TBaseState &state)
{
}

void SpecificWorker::setOdometerPose(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
{
}

void SpecificWorker::correctOdometer(::Ice::Int x, ::Ice::Int z, ::Ice::Float alpha)
{
}

void SpecificWorker::setWheels(QVec wheelVels)
{
	wheelVels.print("wheel vels");
}

