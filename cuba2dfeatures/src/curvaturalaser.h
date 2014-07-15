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
/////////////////////////////////////////////////////////////////////////////
// Nombre	:	curvaturaLASER.h
// Funcion	:     	algoritmo segmentacion array datos laser por curvatura
//			usando k=1, k fijo o k variable (k-salto entre datos del array)
// Autor	:      	A. Bandera
// Fecha	:     	Julio 2005
// Version	:     	v0.1
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nombre	:	curvaturaLASER.h
// Funcion	:     	algoritmo segmentacion array datos laser por curvatura
//			usando k=1, k fijo o k variable (k-salto entre datos del array)
// Autor	:      	A. Bandera
// Fecha	:     	Julio 2005
// Version	:     	v0.1
/////////////////////////////////////////////////////////////////////////////

#ifndef CURVATURALASER_H
#define CURVATURALASER_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <memory.h>
#include <time.h>
#include <sys/types.h>



typedef struct DatLaser
{
  double f;
  double c;
  int kd;
  int ka;
} DatoLASER;

class CURVATURALASER
{

	public:
    	CURVATURALASER(int TamanoLaser_, int K_, int KMAX_, int KMIN_, double UmbralAdapt_,double UmbralEsquina_);
   		~CURVATURALASER();

		DatoLASER *ArrayIN;	/* Array de datos laser de entrada 		*/
		double *Curvatura;	/* Array de datos de curvatura 			*/
		double *CurvDef;
		double *Esquinas;
		double *Circles;
		
	
		int TamanoLaser;
		int K;			/* K=1,...KMax -> Salto entre valores para el 
					calculo de curvatura (K=-1 -> adaptativo) 	*/
		int KMax;		/* Valor máximo de K */
		int KMin;
	
		double UmbralEsquina;	/* Umbral detección esquinas */
		double UmbralAdapt;	/* Umbral cálculo curvatura adaptativa */
	
		int  CalculoCurvatura(int grupo);
		int  ObtenerCurvaturaKcte();
		int  ObtenerCurvaturaAdaptativo(int grupo);
		void  DetectarEsquinas();
		double EsquinaVirtual();
		void DataInPixels(double *pixels, int inicio);
		int DetectarCirculos();
		int DetectarCirculos(int inicio,int fin);
	
		void LeerArray(char *nombre);
		void PintarCurvatura(int grupo, int cont);
		double *PintarCurvaturaScan(int id, int *size);
		int ScanMatchingCurvature(double *fc,int size_fc, int *size_fc_local);
	
		inline double DistEuclidea(double a, double b, double a1, double b1);
		inline double CurvLocal(double f0, double f1, double b0, double b1);
};

#endif
