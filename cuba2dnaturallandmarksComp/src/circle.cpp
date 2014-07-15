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
#include "circle.h"

// Default constructor
Circle::Circle(void)
{
	a=0; b=0; r=1; s=0; i=0;
}

// Constructor with assignment of each field but s and iter
Circle::Circle(double aa, double bb, double rr)
{
	a=aa; b=bb; r=rr;
}

// Constructor with assignment of each field 
Circle::Circle(double aa, double bb, double rr, double ss, int ii)
{
	a=aa; b=bb; r=rr; s=ss; i=ii;
}

// Constructor with three different points

// modified by Pedro Nunnez. Nov. 2006

Circle::Circle(double aa, double bb, double rr, float vectorr[6], float puntoss[6])
{ // (added by Ricardo Vazquez)
  int i;

  a=aa; b=bb; r=rr; 
  for (i=0; i<6;  i++) {
    vector[i]=vectorr[i]; 
    puntos[i]=puntoss[i];
  }
}

Circle::Circle(Data points, int size)
{
	// random points and temporal variables.
	double x1,y1,x2,y2,x3,y3;
	double a_, b_, c_, d_, e_, f_; 

	int indice;

	for (int i=0;i<3;i++) {
		indice = (int)(size*(MT19937()));

		if (i==0) {
			x1 = points.X[indice];
			y1 = points.Y[indice];
		}
		if (i==1) {
			x2 = points.X[indice];
			y2 = points.Y[indice];
		}
		if (i==2) {
			x3 = points.X[indice];
			y3 = points.Y[indice];
		}
		
	}

	a_ = 2*(x2-x1);
	b_ = 2*(y2-y1);
	c_ = x1*x1 + y1*y1 - x2*x2 - y2*y2;
	d_ = 2*(x3-x1);
	e_ = 2*(y3-y1);
	f_ = x1*x1 + y1*y1 - x3*x3 - y3*y3;

	b = (a_*f_-c_*d_)/(b_*d_-a_*e_);
	a = -(b*b_/a_ +c_/a_);
	r = sqrt((x1-a)*(x1-a) + (y1-b)*(y1-b));

	// Comment: "[FD] Almacena informacion para calculos posteriores"
	//vector = (float*)malloc(6*sizeof(float)); 
	//puntos = (float*)malloc(6*sizeof(float)); 
	
	puntos[0]=(float)x1;puntos[1]=(float)y1;puntos[2]=(float)x2;
	puntos[0]=(float)y2;puntos[1]=(float)x3;puntos[2]=(float)y3;

	vector[0]=(float)a_;vector[1]=(float)b_;vector[2]=(float)c_;
	vector[3]=(float)d_;vector[4]=(float)e_;vector[5]=(float)f_;

}


// Printing routine
void Circle::printOld(void)
{
}

//  indicators of bad circles
const	Circle BadDiv(0.,0.,1.,10.,0),		// No convergence (too many iterations)
			   BadEsc(0.,0.,2.,20.,0),		// Escape to infinity
			   BadSto(0.,0.,3.,30.,0);		// Stop where it is not suppose to


Circle::~Circle(void) // (added by Ricardo Vazquez)
{

}
