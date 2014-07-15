/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef CIRCLE_H
#define CIRCLE_H

#include <math.h>
#include "cdata.h"

class Circle
{
	public:

		// The fields of a Circle
		double a, b, r, s;
		float vector[6]; float puntos[6];
		int i;
		// constructors
		Circle();
		Circle(double aa, double bb, double rr);
		Circle(double aa, double bb, double rr, int ii);
		Circle(double aa, double bb, double rr, float *vectorr, float *puntoss);
		Circle(double aa, double bb, double rr, double ss, int ii);
		Circle(Data points, int size);
		// destructor (added by Ricardo Vazquez)
		~Circle();
	
		// routines
		void print(void);
		void printOld(void);
	
		// no destructor we didn't allocate memory by hand.
};

#endif
