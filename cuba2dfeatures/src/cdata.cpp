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
#include "cdata.h"

static unsigned long mt[N_gen]; // the array for the state vector
static int mti=N_gen+1;         // mti==N_gen+1 means mt[N_gen] is not initialized

/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor
Data::Data()
{
	n=0;
	X = boost::shared_array<double>(new double[n]); // (modified by Ricardo Vazquez)
	Y = boost::shared_array<double>(new double[n]);
	for (int i=0; i<n; i++)
	{
		X[i]=0;
		Y[i]=0;
	}
}

// Constructor with assignment of the field N
Data::Data(int N)
{
	n = N;
	X = boost::shared_array<double>(new double[n]); // (modified by Ricardo Vazquez)
	Y = boost::shared_array<double>(new double[n]);

	for (int i=0; i<n; i++)
	{
		X[i]=0.;
		Y[i]=0.;
	}
}

// Constructor with assignment of each field
Data::Data(int N, double dataX[], double dataY[])
{
	n=N;
	X = boost::shared_array<double>(new double[n]); // (modified by Ricardo Vazquez)
	Y = boost::shared_array<double>(new double[n]);

	for (int i=0; i<n; i++)
	{
		X[i]=dataX[i];
		Y[i]=dataY[i];
	}
	eval_dmax();
}

// Routine that evaluate dmax for a sample
// where dmax is the max distance between two data points
void Data::eval_dmax(void)
{
	double di,dx,dy;
	dmax=0;
	for (int i=0; i<n; i++)
		for (int j=0; j<n; j++)
		{
			dx = X[i]-X[j];
			dy = Y[i]-Y[j];
			di = dx*dx+dy*dy;		// square of the dist.
			if (dmax<di) dmax=di;
		}
	dmax=sqrt(dmax);
}

// Routine that shifts the data to the center of mass
void Data::center(void)
{
	int i;

	double meanX=0., meanY=0.;

	//Compute the mean
	for ( i=0; i<n; i++)
	{
		meanX += X[i];
		meanY += Y[i];
	}
	meanX=meanX/n;
	meanY=meanY/n;

	// Shift the data to the mean
	for ( i=0; i<n; i++)
	{
		X[i] -= meanX;
		Y[i] -= meanY;
	}
}

// Printing routine
void Data::print(void)
{
	std::cout << "The data set has " << n << " points with coordinates :"<< std::endl;
	for (int i=0; i<n-1; i++)
	{ std::cout << "(" << X[i] << ", "<< Y[i] << "), "; }
	std::cout << "(" << X[n-1] << ", "<< Y[n-1] << ")\n";
}

// Printing routine
void Data::printX(void)
{
	std::cout << "The data set has " << n << " points X-vector:"<< std::endl;
	for (int i=0; i<n-1; i++)
	{ std::cout << X[i] << ", "; }
	std::cout << X[n-1] << "\n";
}

// Printing routine
void Data::printY(void)
{
	std::cout << "The data set has " << n << " points with Y-vector:"<< std::endl;
	for (int i=0; i<n-1; i++)
	{ std::cout << Y[i] << ", "; }
	std::cout << Y[n-1] << "\n";
}

// Destructor
Data::~Data()
{

	// TO IMPLEMENT 
  //delete[](X); // (added by Ricardo Vazquez)
  //delete[] Y;
}

/************************************************************************
			RANDOM DATA GENERATOR
************************************************************************/

//****************** RadMax ********************************

double RadMax (Data data)
{
	int i;
	double Xi,Yi,Zi,Mxx,Myy,Mxy,Mxxy,Mxxxx,Mxxyy,RadiusMax;
	double lambda,A,B,D;

	Mxx=Myy=Mxy=0.;

	for ( i=0; i<data.n; i++)
	{
		Xi = data.X[i];
		Yi = data.Y[i];
		Zi = Xi*Xi + Yi*Yi;

		Mxy += Xi*Yi;
		Mxx += Xi*Xi;
		Myy += Yi*Yi;
	}

	lambda = (Mxx+Myy-sqrt((Mxx-Myy)*(Mxx-Myy)+4.*Mxy*Mxy))/2.;
	D = sqrt((Mxx-lambda)*(Mxx-lambda) + Mxy*Mxy);
	A = Mxy/D;
	B = (lambda - Mxx)/D;

///	if (lpr>0) fprintf(out,"  Best line fit:  A=%f  B=%f  RSS=%f \n",A,B,lambda);

	Mxx=Myy=Mxy=Mxxxx=Mxxyy=Mxxy=0.;

	for ( i=0; i<data.n; i++)
	{
		Xi = B*data.X[i] - A*data.Y[i];
		Yi = A*data.X[i] + B*data.Y[i];

		Mxy += Xi*Yi;
		Mxx += Xi*Xi;
		Myy += Yi*Yi;
		Mxxy += Xi*Xi*Yi;
		Mxxxx += Xi*Xi*Xi*Xi;
		Mxxyy += Xi*Xi*Yi*Yi;
	}

	RadiusMax = (2.*Mxxyy + (Mxxxx-(Mxx*Mxx/data.n))/2.)/fabs(Mxxy);

///	if (lpr>0) fprintf(out,"  Mxy=%f  Mxx=%f  Myy=%f  Mxxy=%f\n  Mxxyy=%f  Mxxxx=%f  RadMax=%f\n",
///		Mxy,Mxx,Myy,Mxxy,Mxxyy,Mxxxx,RadiusMax);

	return RadiusMax;
}

//****************** SimulateArc ******************************
//
//      N points along an arc with Gaussian noise
//
//  input:
//          R        the radius of arc
//          theta1   first  endpoint of the arc (in radians)
//          theta2   second endpoint of the arc (in radians)
//          sigma    noise level (standard deviation of residuals)

Data SimulateArc(int nPoints, double R, double theta1, double theta2, double sigma)
{
	Data data(nPoints);
	double theta,r,angle;	//,displace,Ri

	for (int i=0; i<nPoints; i++)
	{
		theta = theta1 + (theta2-theta1)*MT19937();
//
//			isotropic Gaussian noise
//
		r = sigma*sqrt(-2.*log(1.-MT19937()));
		angle = 2.*M_PI*MT19937();
		data.X[i] = R*cos(theta) + r*cos(angle);
		data.Y[i] = R*sin(theta) + r*sin(angle);
//
//			orthogonal Gaussian noise
//
//		displace = -6.;
//		for (j=0; j<12; j++) displace += MT19937();
//		displace *= sigma;
//		Ri = R + displace;
//		data.X[i] = Ri*cos(theta);
//		data.Y[i] = Ri*sin(theta);
	}
	data.eval_dmax();
	return data;
}

//****************** SimulateRandom ****************************
//
//       uniformly N points in a unit square

Data SimulateRandom(int nPoints)
{
	Data data(nPoints);

	for (int i=0; i<nPoints; i++)
	{
		data.X[i] = MT19937();
		data.Y[i] = MT19937();
	}
	data.eval_dmax();
	return data;
}


//***************** MT19937 ***********************************
//
//     random number generator

double MT19937()
{
	unsigned long y;
    static unsigned long mag01[2]={0x0, MATRIX_A};
    /* mag01[x] = x * MATRIX_A  for x=0,1 */

    if (mti >= N_gen) { /* generate N words at one time */
        int kk;

        if (mti == N_gen+1)   /* if sgenrand() has not been called, */
            seedMT19937(4357); /* a default initial seed is used   */

        for (kk=0;kk<N_gen-M_gen;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+M_gen] ^ (y >> 1) ^ mag01[y & 0x1];
        }
        for (;kk<N_gen-1;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+(M_gen-N_gen)] ^ (y >> 1) ^ mag01[y & 0x1];
        }
        y = (mt[N_gen-1]&UPPER_MASK)|(mt[0]&LOWER_MASK);
        mt[N_gen-1] = mt[M_gen-1] ^ (y >> 1) ^ mag01[y & 0x1];

        mti = 0;
    }

    y = mt[mti++];
    y ^= TEMPERING_SHIFT_U(y);
    y ^= TEMPERING_SHIFT_S(y) & TEMPERING_MASK_B;
    y ^= TEMPERING_SHIFT_T(y) & TEMPERING_MASK_C;
    y ^= TEMPERING_SHIFT_L(y);

    return ( (double)y / (unsigned long)0xffffffff ); /* reals */
    /* return y; */ /* for integer generation */

}

//*************** seedMT19937 ********************************

void seedMT19937(unsigned long seed)
{

    /* setting initial seeds to mt[N] using         */
    /* the generator Line 25 of Table 1 in          */
    /* [KNUTH 1981, The Art of Computer Programming */
    /*    Vol. 2 (2nd Ed.), pp102]                  */

    mt[0]= seed & 0xffffffff;
    for (mti=1; mti<N_gen; mti++)
        mt[mti] = (69069 * mt[mti-1]) & 0xffffffff;

}
