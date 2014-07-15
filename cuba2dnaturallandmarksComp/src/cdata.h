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
#ifndef CDATA_H
#define CDATA_H

#include <boost/shared_array.hpp> // (added by Ricardo Vazquez)
#include <math.h>
#include <iostream>

/************************************************************************
			RANDOM DATA GENERATOR
************************************************************************/
//-----   Parameters for Random Number Generator MT19937

/* Period parameters */ 

#define N_gen 624 
#define M_gen 397 
#define MATRIX_A 0x9908b0df   // constant vector a 
#define UPPER_MASK 0x80000000 // most significant w-r bits 
#define LOWER_MASK 0x7fffffff // least significant r bits 

/* Tempering parameters */ 

#define TEMPERING_MASK_B 0x9d2c5680
#define TEMPERING_MASK_C 0xefc60000 
#define TEMPERING_SHIFT_U(y) (y >> 11) 
#define TEMPERING_SHIFT_S(y) (y << 7) 
#define TEMPERING_SHIFT_T(y) (y << 15)
#define TEMPERING_SHIFT_L(y) (y >> 18)

void seedMT19937(unsigned long seed);
double MT19937();

class Data
{
	public:

		int n;
		boost::shared_array<double> X;	//space is allocated in the constructors //modified by Ricardo Vazquez
		boost::shared_array<double> Y;	//space is allocated in the constructors
		double dmax;		// max distance between two data points
	
		// constructors
		Data();
		Data(int N);
		Data(int N, double X[], double Y[]);
	
		// routines
		void eval_dmax(void);
		void center(void);
		void print(void);
		void printX(void);
		void printY(void);
		
		// destructors
		~Data();
};

#endif
