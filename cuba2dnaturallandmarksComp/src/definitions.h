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
#ifndef __DEFINITIONS__
#define __DEFINITIONS__

//-------------------------------------------------------------------------------
//------------------------ Definicion de variables de uso interno ---------------
//-------------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265358979323846
#endif

// Comment: Array_pixels[_X,_Y,_THETA], breakpoints[_X,_Y,N_GRP]
#define _X          1
#define _Y          2
#define _THETA      3
#define _N_GRP      3
#define _RADIUS     3

// Comment: Tipo de esquina: virtual y real
#define _TYPEc      4 
#define _INDEX      5
#define _ALFA1      6
#define _ALFA2      7
#define _RHO1       8
#define _RHO2       9



// Comment: Array_points[_R,_Q]
#define _R          1
#define _Q          2

// Comment: CUBA Algorithm variables
#define N_CARACT 200 
#define MAX_GROUP 100

//-------------------------------------------------------------------------------
//---------------- Definicion de variables para el algor. segmentacion ----------
//-------------------------------------------------------------------------------
#define LONG_SEGMENTO 10
#define UMB_ESQUINAS  0.15
#define COEF_MINIMO   0.05 

//-------------------------------------------------------------------------------
//---------------- Definicion de variables para el extraccion de caract. --------
//-------------------------------------------------------------------------------
#define SN_Rth 5 //75
#define SN_Rxy 5 //75
#define MAX_DET_Rs 1e-11*(SN_Rxy*SN_Rth) 
#define MAX_DET_Rp 1e3*(SN_Rxy*SN_Rth)

#define offset_Rs       4 // Comment: num de elementos en matriz de incertidumbre para segmentos
#define offset_Rp       6 // Comment: num de elementos en matriz de incertidumbre para puntos (esquinas y extremos)
#define offset_Rc       6 // Comment: num de elementos en matriz de incertidumbre para circulos

#define offset_points   3 // Comment: num de elementos en array de puntos
#define offset_corners  9 // Comment: num de elementos en array de corners
#define offset_circles  3 // Comment: num de elementos en array de circulos
#define offset_segments 6 // Comment: num de elementos en array de segmentos rectos

//-------------------------------------------------------------------------------
//---------------- Estructura de datos usadas en el sistema ---------------------
//-------------------------------------------------------------------------------
typedef struct dataSegment{
 
  // Comment: location of the line segment data
  float x_ini,y_ini;
  float x_fin,y_fin;
  float pho,theta;
  float longitud;
  float a,b;
  float alfa;
 
  // Comment: covariance matrix of the line segment data
  float lalfa, lr;
  float lcalfa,lcr,lcalfar;

} SEGMENTO;

typedef struct dataCircle{
  
  // Comment: location of the circle segment data
  float x, y;
  float r;

  // Comment: covariance of the circle segment data
  float vector[offset_Rc]; float puntos[offset_Rc]; 

} CIRCULO;


typedef struct MapaLaser{
  
  // Comment: mapa of the current scan data
  int n_esquinas;
  double *esquinas;
  double *R_esquinas;

  int n_segmentos;
  double *segmentos;
  double *R_segmentos;

  int n_circulos;
  double *circulos;
  double *R_circulos;

} MAPA_LASER;

#define TIME_C          100 
#define USEC_MSEC       0.001

#define SIGMA_R         10                     // sigma_r = 10mm UTM-30LX
#define SIGMA_THETA     0.007//(0.25/4)*PI/180        // sigma_a = 0.25 UTM-30LX

#define UMBRAL_LONG  10
#define UMBRAL_PIXEL 10

#define _XINI   0 
#define _YINI   1
#define _XFIN   2
#define _YFIN   3
#define _DIST   4
#define _A      5
#define _B      6
#define _ERROR  7

#define _ALFA   8
#define _RO     9
#define _CALFA  10
#define _CR     11
#define _CALFAR 12

#define LINESEGMENTS  0
#define CORNERS       1
#define CURVESEGMETNS 2

#endif

