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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "definitions.h"
#include "curvaturalaser.h"
#include "cdata.h"
#include "circle.h"
#include <innermodel/innermodel.h>
#include <ipp.h>

/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		RoboCompCuba2Dnaturallandmarks::Features computeFeatures( const RoboCompLaser::TLaserData & lData);
		RoboCompCuba2Dnaturallandmarks::Features getFeatures();
		RoboCompCuba2Dnaturallandmarks::Features getLocalFeatures();

	public slots:
		void compute(); 	
		
	private:
	RoboCompDifferentialRobot::TBaseState bState, tmp_state;
	RoboCompLaser::TLaserData laserData;
	RoboCompLaser::TLaserData laserDataMedian;
	InnerModel *innerModel;

	// PMNT Enero 2010
	float laserRange;
	RoboCompLaser::LaserConfData configLaser;
	
	void medianFilter( const RoboCompLaser::TLaserData & laserData );
	//void features();
	
	void features( const RoboCompLaser::TLaserData & lData, MAPA_LASER * scan_mapa, MAPA_LASER *scan_maparef);
	void iniFeaturesVar(int n );

	// [vbles. scan]
	double *array_pixels;			// Array para deteccion de esquinas (x,y)
	double *array_points;			// Array para deteccion de esquinas (r,theta)

	// [vbles. segmentation]
	int *breakpoints;				// Array para segmentacion del scan
	int indice_grupo;				// breakpoints and group index
	int contseg;					// group index
	// [vbles. landmarks]
	int cont_segment, cont_circle;  // landmarks index
	double *array_corners;			// Array para guardar esquinas
	double *virtual_corners;			// Array para guardar esquinas virtuales
	double *segmento;				// Array de segmentos
	SEGMENTO *array_segment_local;
	CIRCULO   circle;				// Circle information
	CIRCULO  *array_circle_local;	// Array de circulos del scan
	// [vbles. statistical]
	double *matriz_virtual;         // Array para guardar las matrices de incertidumbre de las esquinas virtuales
	double *matriz_cov_circulos;    // Array para guardar las matrices de incertidumbre de las esquinas virtuales

	float height;
	// [vbles. debug]
	int debug_cont;
	
	MAPA_LASER scan_mapa, scan_maparef;
  

	// ***************************************************************************
	// Funcion: MapBreakPoints
	//	Encuentra los breakpoints en el scan laser
	// IN:	num_pixels: numero de puntos en array entrada
	//		_array_pixels_lc: array de entrada
	//		robotx,roboty: posicion del robot
	// OUT: fichero de posiciones con puntos de ruptura
	//			+1 : posicion x; +2: posicion y; +3: longitud del tramo entre puntos
	// ***************************************************************************
	void MapBreakPoints (int *indice_grupo, int num_pixels, double * _array_pixels_lc, double robotx, double roboty);
	bool find_breakPoints(int contador, double robotx, double roboty, double * array_pixels_lc);
	double dist_p(int contador, double * array_pixels_lc);
	double dist_r(int contador, double robotx, double roboty, double * array_pixels_lc);
	double angle2rad(double angle);
	double rad2angle ( double angle );
	void segment_new(double *segmento_new, double *pixels, double *points, int inicio, int fin);
	void segmento_Kai(double *segment, double *puntos, int inicio, int fin);
	void extremo_kai(double rho, double theta, double alfa, double r, double *x1, double *y1);
	void incluir_segmento(SEGMENTO *array_segment_local,double *segmento,int _cont);
	void detectar_EsquinaVirtual(SEGMENTO *array_segment_local,double *esquina, int _cont, double *matriz_R);
	void iniciar_mapa_local(MAPA_LASER &mapa);//****************** Sigma ************************************
	void destruir_mapa_local(MAPA_LASER *mapa); // (added by Ricardo Vazquez)
	void incluir_esquina_virtual(MAPA_LASER *mapa, double *esquinas, double *matriz);
	void  obtener_mapa(MAPA_LASER *mapa, SEGMENTO *array_segment_local,int *_cont);
	void incluir_esquina(MAPA_LASER *mapa, double *esquinas,double *points, int inicio);
	void incluir_edge(MAPA_LASER *mapa, double posx, double posy,double r, double theta, double Calfa, double alfa);
	void detectar_Edge(MAPA_LASER *mapa, int contseg, int *breakpoints,double *array_corners, double *array_pixels,double * array_points);
	int MaxDistEuclidea(double a, double b, double c, double d, double e, double f);
	CIRCULO circulo_new(double *array_pixels,int inicio,int fin);
	void incluir_circulo(MAPA_LASER *mapa, CIRCULO *array_circle_local, CIRCULO circle, int cont_circle, double *matriz);
	void  matriz_covarianza_circulo(CIRCULO circle, double *matriz, int cont);
	double Sigma(Data data, ::Circle circle);
	void  matriz_covarianza_virtual(SEGMENTO segmento_2,SEGMENTO segmento_1, double *matriz, int cont);
	double dist_euclidean2D(double x1, double y1, double x2, double y2);
	void  map_copy(MAPA_LASER *mapa, MAPA_LASER *ref);

};

#endif