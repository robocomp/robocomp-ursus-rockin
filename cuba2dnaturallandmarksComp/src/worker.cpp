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
#include "worker.h"


Worker::Worker ( RoboCompCCDAmatching::CCDAmatchingPrx ccdamatchingprx, RoboCompLaser::LaserPrx laserprx,  RoboCompDifferentialRobot::DifferentialRobotPrx baseprx, float h, QObject *parent ) : QObject ( parent )
{
	ccdamatching = ccdamatchingprx;
	laser = laserprx;
	base = baseprx;
	height = h;
	
	debug_cont = 0;
	innerModel = new InnerModel("/home/robocomp/Escritorio/TFG/mundo.xml");	

	// PMNT: Configuración leída por el sensor láser

	printf ( "Reading laser conf data\n" );

	try 
	{
		configLaser = laser->getLaserConfData();
		qDebug() << "MaxRange: " << configLaser.maxRange;
		laserRange = configLaser.maxRange*2;
		qDebug() << "MinRange: " << configLaser.minRange;
		qDebug() << "MaxMeasures: " << configLaser.maxMeasures;
		qDebug() << "MaxDegrees: " << configLaser.maxDegrees;
		qDebug() << "SampleRate: " << configLaser.sampleRate;
		qDebug() << "IniRange: " << configLaser.iniRange;
		qDebug() << "EndRange: " << configLaser.endRange;
		qDebug() << "Cluster: " << configLaser.cluster;
		qDebug() << "Number of measures: " << configLaser.endRange- configLaser.iniRange + 1;
	}
	catch(const Ice::Exception & ex)
	{
		std::cout << ex;
		qFatal(" [LaserVisor::Laservisor] -> Fatal. Could not read laser configuration");
	}


	try
	{
		laserData = laser->getLaserData();
	}
	catch ( const Ice::Exception& ex )
	{
		std::cout << "cuba2dnaturallandmarksComp: Exception talking to LaserComp. " << std::endl;
	}

	iniFeaturesVar ( laserData.size() );
	iniciar_mapa_local( scan_mapa );
	iniciar_mapa_local( scan_maparef );

	connect ( &timer, SIGNAL ( timeout() ), this, SLOT ( compute() ) );
	timer.start ( BASIC_PERIOD );


	qDebug() << "Cuba2DnaturallandmarksComp::Worker constructor finished OK";
	
}

Worker::~Worker()
{
}

void Worker::compute( )
{
 
 
	
	static bool baseFunciona = true;
	try
	{
		base->getBaseState ( bState );
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	}
	catch ( const Ice::Exception& ex )
	{
		if (baseFunciona)
		{
		  std::cout << "cuba2dnaturallandmarksComp: Exception talking to BaseComp. " << std::endl;
		  baseFunciona = false;
		}
	}
	try
	{
		debug_cont ++;
		laserData = laser->getLaserData( );
		medianFilter(laserData);
	}
	catch ( const Ice::Exception& ex )
	{
		std::cout << "cuba2dnaturallandmarksComp: Exception talking to LaserComp. " << std::endl;
	}

	features( laserData, &scan_mapa, &scan_maparef); 

}

void Worker::medianFilter( const RoboCompLaser::TLaserData & laserData )
{
	Ipp32f x[laserData.size()];
	
	for(uint i=0; i<laserData.size(); i++)
	{
		x[i] = laserData.at(i).dist;		
	}
	
	ippsFilterMedian_32f_I(x, laserData.size(), 5);
}



/**
 * Inicialización de variables dinámicas de features
 * @param n
 */
void Worker::iniFeaturesVar ( int n )
{
	array_pixels= ( double* ) malloc ( ( n*offset_points+1 ) *sizeof ( double ) );
	array_points= ( double* ) malloc ( ( n*offset_points+1 ) *sizeof ( double ) );
	breakpoints = ( int* ) malloc ( ( MAX_GROUP*offset_points+1 ) *sizeof ( double ) );
	array_corners= ( double* ) malloc ( ( ( offset_corners*N_CARACT ) +1 ) *sizeof ( double ) );
	virtual_corners= ( double* ) malloc ( ( ( offset_corners*N_CARACT ) +1 ) *sizeof ( double ) );
	matriz_virtual= ( double* ) malloc ( offset_Rp*N_CARACT*sizeof ( double ) );
	matriz_cov_circulos= ( double* ) malloc ( offset_Rc*N_CARACT*sizeof ( double ) );
	array_segment_local= ( SEGMENTO* ) malloc ( ( N_CARACT ) *sizeof ( SEGMENTO ) );
	array_circle_local= ( CIRCULO* ) malloc ( ( N_CARACT ) *sizeof ( CIRCULO ) );
	segmento= ( double* ) malloc ( N_CARACT*sizeof ( double ) );
}

/**
 * Procesa el scan del laser y obtiene el mapa del entorno
 * @param lData scan data
 * @param mapa mapa de la observacion
 */
void Worker::features ( const RoboCompLaser::TLaserData & lData, MAPA_LASER *scan_mapa, MAPA_LASER *scan_maparef )
{
	char nomb1[30];	
	static int cont = 0;
	int inicio, fin;		// Vbles. auxiliares: inicio y fin del posible tramo recto
	int es_circulo;                 // Bool de control de circulos
	
	// [vbles. curvature function]
	CURVATURALASER *curvatureLaser; // Array para calculo de curvatura
	curvatureLaser = ( CURVATURALASER* ) malloc ( sizeof ( CURVATURALASER ) );

	virtual_corners[0]=0;

	// [start]
	indice_grupo=0;

	//[actualizo vbles. a utilizar. scan]
	int px, py, pz;
	pz = height;
	QMat pLaser ( 3 ); 
	QMat punto ( 3 ); 
	
	FILE *puntMAT;	
	
	
	sprintf(nomb1,"scan%d.m",cont);
	puntMAT=fopen(nomb1,"wb");
	fprintf(puntMAT,"figure(1);zoom on;grid on; hold on; axis equal\n");
	
	
	for ( uint i=0; i < lData.size(); i++ )
	{
		pLaser = innerModel->laserToBase ("base", lData[i].dist, lData[i].angle );
		array_pixels[i*offset_points+_X]=  pLaser ( 2 );
		array_pixels[i*offset_points+_Y]=  pLaser ( 0 );
		px = round(pLaser(2)); py = round(pLaser(0));
		if(lData[i].dist < 15000.) fprintf(puntMAT,"plot3(%d,%d,%d,'.k')\n",px, -py, pz);
		
		// Comment: "[LE] Conversion a polares (no será necesario en el futuro)"
		array_points[i*offset_points+_R]= lData[i].dist;
		array_points[i*offset_points+_Q]= lData[i].angle;

	}
	
	fprintf(puntMAT,"h=circle([0 0],200);set(h,'EdgeColor','r');set(h,'LineWidth',1);h=line([0 200], [0 0]);set(h,'Color','r');set(h,'LineWidth',1);\n");

	// [breakpoints]
	MapBreakPoints ( breakpoints,laserData.size(),array_pixels,0,0 );

	// Comment: "[LE] Inicializo curvatureLaser --> breakpoints[contseg+3]: almacena la longitud del scan"
	
	indice_grupo=0;
	cont_segment=0; cont_circle=0;

	scan_mapa->n_esquinas = 0;

	// Comment: "[LE] BUCLE ESTUDIO TROZOS DEL SCAN: contseg - contador del subsegmento o trozo"
	contseg=0;
	
	
	while (indice_grupo < breakpoints[0] ) // ( breakpoints[contseg+1]!=-1 )
	{

	  
	  for (uint i=breakpoints[contseg+1];i<breakpoints[contseg+2];i++)	{

		// Comment: "[LE] Representacion de los grupos en Matlab"
		
		
		
		px = round(array_pixels[i*offset_points+1]); py = round(array_pixels[i*offset_points+2]);
		if(abs(py) < 12000 && abs(px) < 12000){
		fprintf(puntMAT, "figure(1);");
		if ((indice_grupo==0) || (indice_grupo==6))
			fprintf(puntMAT, "plot3(%d,%d,%d,'.r')\n",px,-py,pz);
		if ((indice_grupo==1) || (indice_grupo==7))
			fprintf(puntMAT, "plot3(%d,%d,%d,'.g')\n",px,-py,pz);
		if (indice_grupo==2)
			fprintf(puntMAT, "plot3(%d,%d,%d,'.b')\n",px,-py,pz);
		if (indice_grupo==3)
			fprintf(puntMAT, "plot3(%d,%d,%d,'.y')\n",px,-py,pz);
		if (indice_grupo==4)
			fprintf(puntMAT, "plot3(%d,%d,%d,'.c')\n",px,-py,pz);
		if (indice_grupo==5)
			fprintf(puntMAT, "plot3(%d,%d,%d,'.g')\n",px,-py,pz);
		}
	  }
		//[landmark extraction]
		curvatureLaser=new CURVATURALASER ( breakpoints[contseg+offset_points],-1,15,7,1, 0.2 );

		// Comment: "[LE] Guardar datos en curvatureLaser desde breakpoints[contseg+1]"
		curvatureLaser->DataInPixels ( array_pixels,breakpoints[contseg+1] );

		// Comment: "[LE] Calculo de la funcion de curvatura del scan"
		curvatureLaser->CalculoCurvatura ( indice_grupo );

		// Comment: "[SM] scanProcess"
		curvatureLaser->PintarCurvatura(indice_grupo, cont);
		indice_grupo=indice_grupo+1;

		// Comment: "[LE] Deteccion de esquinas reales del entorno.
		// --> Realiza una subsegmentacion del scan"

		curvatureLaser->DetectarEsquinas();
		array_corners[0]=curvatureLaser->Esquinas[0];

		// [SM ScanmatchingFC]
			
		for ( int m=0; m< (int) array_corners[0]; m++ )
		{
			array_corners[m*offset_corners+_X]=curvatureLaser->Esquinas[m*offset_corners+_X];
			array_corners[m*offset_corners+_Y]=curvatureLaser->Esquinas[m*offset_corners+_Y];
			array_corners[m*offset_corners+_THETA]=curvatureLaser->Esquinas[m*offset_corners+_THETA];
			array_corners[m*offset_corners+_TYPEc]= 0; // Comment: Esquina Real
			array_corners[m*offset_corners+_INDEX]= curvatureLaser->Esquinas[m*offset_corners+_INDEX];
			
		}
	

		// Comment: "[LE] Para cada esquina, analizo el scan data para extraer marcas"
		if ( array_corners[0]!=-1 )
		{
			//  Comment: "[LE] Actualizar inicio y fin de posibles segmentos dentro del trozo"
			inicio=breakpoints[contseg+1];

		
			for ( int k=0;k<=array_corners[0];k++ )
			{
			  //  Comment: "[LE] Si no hay mas esquinas, fin toma el valor breakpoints[..+2]
			  // --> corresponde con el fin del scan"
			  if ( k==array_corners[0] )
			    {
			   
			      fin= (breakpoints[contseg+2]-3);
			    }
			  else
			    {
			      //  Comment: "[LE] si no, fin toma el valor de la esquina.
			      // --> corresponde con otro subgrupo"
			      		      
			      fin= ( int ) ( array_corners[k*offset_corners+1]+breakpoints[contseg+1] );
			      //std::cout << "[BREAKPOINTS] Inicio (Breakpoints): " << inicio << "Fin (breakpoints)" << fin << std::endl;
			     
			    }
			    
			   
		
			  //  Comment: "[LE] Caracterizacion del tramo definido por los breakpoints"
			  if ( ( fin-inicio ) >= UMBRAL_PIXEL ) 
			    {
			      // ------------------------------------------------------------------------------
			      // Comment: "[LE] CIRCLES SEGMENTS"
			      // Comment: "[LE] Deteccion de circulos
			      
			      // es_circulo = curvatureLaser->DetectarCirculos ( inicio, fin );
			      es_circulo = 0;
			      
			      // Comment: "[LE] Analisis de resultados para la deteccion o no de segmentos circulares"
			      if ( es_circulo )
				{
				  circle = circulo_new ( array_pixels,inicio,fin );
				  if ( circle.r!=0 )
				    {
				      es_circulo = 1;
				      
				      fprintf(puntMAT,"figure(1);");
				      fprintf(puntMAT,"r=%f;v=[%f,%f];circle(v,r,0);",circle.r,circle.x,circle.y);
				      
				      incluir_circulo ( scan_mapa, array_circle_local, circle, cont_circle, matriz_cov_circulos );
				      cont_circle++;
				    }
				  else es_circulo=0;
				}
			      // ------------------------------------------------------------------------------
			      // Comment: "[LE] LINE SEGMENTS "
			      if ( es_circulo==0 )
				{
				  
				  segment_new (segmento,array_pixels,array_points,inicio,fin );
				  //Comment: "[LE] Si la longitud del segmento supera el umbral, se incluye"
				  
				  if ( ( segmento[_DIST]>=LONG_SEGMENTO ) )
				    {
				    	      
				      incluir_segmento ( array_segment_local,segmento, cont_segment );
				      cont_segment++;
				      
				      px = round(segmento[0]); py = round(segmento[1]);
				      if(abs(py) < 12000 && abs(px) < 12000){
				      fprintf(puntMAT,"figure(1);");
				      fprintf(puntMAT,"plot3(%d,%d,%d,'sb')\n",px,-py,pz);
				      }
				      px = round(segmento[2]); py = round(segmento[3]);
				      if(abs(py) < 12000 && abs(px) < 12000){
				      fprintf(puntMAT,"figure(1);");
				      fprintf(puntMAT,"plot3(%d,%d,%d,'sb')\n",px,-py,pz);
				      }
				      
				      
				      //Comment: "[LE] CORNERS"
				      //Comment: "[LE] Por cada segmento detectado, se realiza una busq--> de posibles esquinas al cortar con otros segmentos"
				      virtual_corners[0]=0;
				      detectar_EsquinaVirtual ( array_segment_local,virtual_corners,cont_segment,matriz_virtual );
				      incluir_esquina_virtual ( scan_mapa, virtual_corners, matriz_virtual );
				      
				    }
				  
				}
			    }
			  // Comment: "[LE] inicio del nuevo trozo es el fin del anterior"
			  inicio=fin;
			}
		}
		contseg+=3;
		if ( curvatureLaser!= NULL )
			free ( curvatureLaser );
		// Comment: "[LE] BUCLE ESTUDIO TROZOS DEL SCAN"
	}
	
	for (int i = 0; i < breakpoints[0]; i++) 
	{

	  
		// Comment: "Inicio del grupo"
		punto(2) = array_pixels[breakpoints[i*3 +1]*offset_points + _X];
		punto(0) = array_pixels[breakpoints[i*3 +1]*offset_points + _Y];
		punto(1) = 0.;
		
		px = round(punto(2)); py = round(punto(0));
				      if(abs(py) < 12000 && abs(px) < 12000){
				      fprintf(puntMAT,"figure(1);");
				      fprintf(puntMAT,"plot3(%d,%d,%d,'sb')\n",px,-py,pz);
				      }
		
		// Comment: "Fin del grupo"
		punto(2) = array_pixels[breakpoints[i*3 +2]*offset_points + _X];
		punto(0) = array_pixels[breakpoints[i*3 +2]*offset_points + _Y];
		punto(1) = 0.;
		
		px = round(punto(2)); py = round(punto(0));
				      if(abs(py) < 12000 && abs(px) < 12000){
				      fprintf(puntMAT,"figure(1);");
				      fprintf(puntMAT,"plot3(%d,%d,%d,'sb')\n",px,-py,pz);
				      }
		
	}
	
	
	
	// Comment: "[LE] Actualiza mapa con segmentos obtenidos"
	scan_mapa->circulos[0]=cont_circle;
	obtener_mapa ( scan_mapa, array_segment_local,&cont_segment );

	
	printf("\n--------------------------------------------------------------------\n\n");
	printf("-----------------------CIRCLE SEGMENTS-------------------------\n\n");

	for (int z=0;z<scan_mapa->circulos[0];z++) 
	  {
					
	  fprintf(puntMAT,"figure(1);");
	  printf("[MAP LANDMARK] c=[(%f,%f,%f)]; \nP=[%e %e %e %e %e %e]\n",scan_mapa->circulos[z*3+1],scan_mapa->circulos[z*3+2],
	  scan_mapa->circulos[z*3+3],scan_mapa->R_circulos[z*offset_Rc],scan_mapa->R_circulos[z*offset_Rc+1],
	  scan_mapa->R_circulos[z*offset_Rc+2],scan_mapa->R_circulos[z*offset_Rc+3],scan_mapa->R_circulos[z*offset_Rc+4],scan_mapa->R_circulos[z*offset_Rc+5]);
	}


	printf("\n--------------------------------------------------------------------\n\n");
	printf("-----------------------LINE SEGMENTS-------------------------\n\n");

         for (int z=0;z<scan_mapa->segmentos[0];z++) 
	{
	printf("[MAP LANDMARK] s=[(%f,%f)]; \nP=[%e %e;%e]\n",scan_mapa->segmentos[z*6+5],scan_mapa->segmentos[z*6+6],
	scan_mapa->R_segmentos[z*offset_Rs],scan_mapa->R_segmentos[z*offset_Rs+1],scan_mapa->R_segmentos[z*offset_Rs+2]);
	}

	printf("\n------------------------CORNERS AND EDGES----------------------------\n\n");

	for (int z=0;z<scan_mapa->n_esquinas;z++) 
	{
  
	  px = round(-scan_mapa->esquinas[z*4+1]), py = round(scan_mapa->esquinas[z*4]);
	fprintf(puntMAT,"figure(1);plot(%d,%d,%d,'^b')\n",px,py,pz);

    	printf("[MAP LANDMARK] x=[%f,%f,(%f)]; \nP=[%e %e;%e %e; %e %e]\n",-scan_mapa->esquinas[z*4+1],scan_mapa->esquinas[z*4],scan_mapa->esquinas[z*4+2],
		scan_mapa->R_esquinas[z*offset_Rp],scan_mapa->R_esquinas[z*offset_Rp+2],scan_mapa->R_esquinas[z*offset_Rp+1],
		scan_mapa->R_esquinas[z*offset_Rp+3],scan_mapa->R_esquinas[z*offset_Rp+4],scan_mapa->R_esquinas[z*offset_Rp+5]);
	fprintf(puntMAT,"figure(1);");
	fprintf(puntMAT,"x=[%f,%f]; P=[%f %f;%f %f]; L=make_landmark_covariance_ellipse(x,P);x=L(2,:);y=L(1,:);plot(y,x)\n",
		-scan_mapa->esquinas[z*4+1],scan_mapa->esquinas[z*4],scan_mapa->R_esquinas[z*offset_Rp],scan_mapa->R_esquinas[z*offset_Rp+3],scan_mapa->R_esquinas[z*offset_Rp+3],scan_mapa->R_esquinas[z*offset_Rp+1]);
			
	}
			
	printf("\n--------------------------------------------------------------------\n\n");
	
	
	map_copy(scan_mapa, scan_maparef);
	// Comment: "[SM] Fin del No hay error en el matching"
	
	cont ++;
	fclose(puntMAT);
}

// ***************************************************************************
// Funcion: angle2rad
//	Devuelve el angulo en radianes
// IN:	angulo en grados
// OUT: angulo en radianes
// ***************************************************************************
double Worker::angle2rad ( double angle ){
	return angle*M_PI/180;
}

// ***************************************************************************
// Funcion: rad2angle
//	Devuelve el angulo en radianes
// IN:	angulo en radianes
// OUT: angulo en grados
// ***************************************************************************
double Worker::rad2angle ( double angle )
{
	return angle*180/M_PI;
}


// ***************************************************************************
// Funcion: dist_r
//	Calcula la distancia del punto array_pixels_lc[contador] al robot
// IN:	contador: posicion en el scan
//		robotx,roboty: posicion del robot
// OUT: distancia euclidea del punto array_pixels_lc[contador] al robot
// ***************************************************************************
double Worker::dist_r ( int contador, double robotx, double roboty, double * array_pixels_lc )
{
	double x,y;

	x=array_pixels_lc[contador*3+1]-robotx;
	y=array_pixels_lc[contador*3+2]-roboty;

	return sqrt ( x*x + y*y );
}

// ***************************************************************************
// Funcion: dist_r
//	Calcula la distancia del punto array_pixels_lc[contador] al array_pixels_lc[contador-1]
// IN:	contador: posicion en el scan
// OUT: distancia euclidea del punto array_pixels_lc[contador] al array_pixels_lc[contador-1]
// ***************************************************************************
double Worker::dist_p ( int contador, double * array_pixels_lc )
{
	return ( sqrt (
	             ( array_pixels_lc[contador*3+1] - array_pixels_lc[ ( contador-1 ) *3+1] )
	             * ( array_pixels_lc[contador*3+1] - array_pixels_lc[ ( contador-1 ) *3+1] ) +
	             ( array_pixels_lc[contador*3+2] - array_pixels_lc[ ( contador-1 ) *3+2] ) *
	             ( array_pixels_lc[contador*3+2] - array_pixels_lc[ ( contador-1 ) *3+2] )
	         ) );
}

// ***************************************************************************
// Funcion: find_breakPoints
//	Encuentra los breakpoints en el scan laser
// IN:	contador: posicion en el scan
//		robotx,roboty: posicion del robot
// OUT: array_pixels_lc[contador] es un breakpoint/no lo es
// ***************************************************************************
bool Worker::find_breakPoints ( int contador, double robotx, double roboty, double * array_pixels_lc )
{
	double Dmax;
	double sigma;
	double inc_theta, alfa;

	inc_theta = rad2angle(SIGMA_THETA); alfa = 0.8;          // angle in degree
	sigma= SIGMA_R;                				 // distance in mm
	
	Dmax = dist_r ( contador-1,robotx,roboty, array_pixels_lc ) *sin ( angle2rad ( inc_theta ) ) /sin ( angle2rad ( alfa-inc_theta ) )
	     + 3*sigma;
	if ( dist_p ( contador, array_pixels_lc ) > Dmax )
		return true;
	else
		return false;
}

// ***************************************************************************
// Funcion: MapBreakPoints
//	Encuentra los breakpoints en el scan laser
// IN:	num_pixels: numero de puntos en array entrada
//		_array_pixels_lc: array de entrada
//		robotx,roboty: posicion del robot
// OUT: fichero de posiciones con puntos de ruptura
//			+1 : posicion x; +2: posicion y; +3: longitud del tramo entre puntos
// ***************************************************************************
void Worker::MapBreakPoints ( int *indice_grupo, int num_pixels, double * _array_pixels_lc, double robotx,
                              double roboty )
{
	int i; 
	double x,y;
	int grupo, cont_grupo, begin, end;
	int MAX_GRUPO=50;

	for ( i=0;i<MAX_GRUPO*3;i++ )
	{
		indice_grupo[i]=-1;
	}

	//array_pixels_lc=(double*)malloc((num_pixels*3+1)*sizeof(double));
	//array_pixels_lc=_array_pixels_lc;

	cont_grupo=0;  begin=0; grupo=0;

	i=0; 
	
	while ( i<num_pixels )
	{
		x= _array_pixels_lc[3*i+_X]; 
		y= _array_pixels_lc[3*i+_Y];
		
		if ( ! ( find_breakPoints ( i,robotx,roboty,_array_pixels_lc ) ) && i!= ( num_pixels-1 ) )
		{
			grupo++;
		}
		else
		{
			end=i-1;
			if ( grupo>UMBRAL_LONG )
			{
				indice_grupo[cont_grupo*3+1]=begin;     // inicio del bloque de puntos
				indice_grupo[cont_grupo*3+2]=end;		// fin del bloque de puntos
				indice_grupo[cont_grupo*3+3]=grupo;		// numero de elementos
				cont_grupo++;
			}
			grupo=0;
			begin=i;
		}
		i++;
	}

	indice_grupo[0]=cont_grupo;
}

// ***************************************************************************
// Funcion: MaxDistEuclidea
//	Devuelve un 1 si (a,b) estan mas cerca de (e,f) y un 2 en caso contrario
// IN: 
//     int a,b - x,y de un punto
//     int c,d - x,y de un punto	   
//     int e,f - x,y de un punto	   
//
// OUT:
// ***************************************************************************
int Worker::MaxDistEuclidea(double a, double b, double c, double d, double e, double f)
{
	if (sqrt((a-e)*(a-e)+(b-f)*(b-f))<sqrt((c-e)*(c-e)+(d-f)*(d-f)))
		return 1;
	else
		return 2;
}



// ***************************************************************************
// Funcion: extremo_kai
//	Calcula los extremos de los posibles segmentos usando a,b
// IN: 
//     double x,y: punto inicial del segmento
//     double xMed: valor medio x
//     flota *x1,*y1: extremos
// OUT:
// 
// ***************************************************************************
void Worker::extremo_kai(double rho, double theta, double alfa, double r, double *x1, double *y1)
{
	
  double r2, alfa2; double x,y;

  // Comment: punto de corte entre el punto (su recta generada) y la recta pasado como parametro
  alfa2=alfa+(double)(PI/2);
 
  x=rho*cos(theta);
  y=rho*sin(theta);
 
  
  r2=x*cos(alfa2) + y*sin(alfa2);
  
  if (r2<0) {alfa2+=(double)PI;r2=-r2;}

#ifdef __DEBUG__
  printf("[EXT] alfa2,r2: %f,%f x,y: %f,%f \n",alfa2*180/PI,r2,x,y);
#endif

  *y1=(r2*cos(alfa) - r*cos(alfa2))/(sin(alfa2)*cos(alfa) - sin(alfa)*cos(alfa2));
  *x1=(r - *y1*sin(alfa))/cos(alfa);


}

// ***************************************************************************
// Funcion: segmento
//	Actualiza los campos del segmento (segment)
// IN: 
//     double *puntos: puntos del array
//     double inicio,fin: puntos inicial y final
// OUT:
//     double*: devuelve el segmento definitivo. 
// ***************************************************************************

void Worker::segmento_Kai(double *segment, double *puntos, int inicio, int fin)
{

  //double xi,xf,yi,yf;
  double sum_x;double sum_y;
  double w; double w_i;

  double c_alfa; double c_r; double c_ralfa;
  double r,alfa;
  double N,D; //double N_,D_;
  
  int i; //int j;

  double dalfa=0, drho=0;
  int num_puntos;
  
  w_i=1;

  sum_x=sum_y=w=0;
  num_puntos=(fin-inicio);
  
   
  for (i=inicio;i<=fin;i++)
    {
	  
      w+= w_i;
      sum_x = sum_x+w_i*puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]);
      sum_y = sum_y+w_i*puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]);
      
    }
  
  sum_x = (1/w)*sum_x;
  sum_y = (1/w)*sum_y;

  N=0;D=0;

  for (i=inicio;i<=fin;i++)
    {
      N+= w_i*(sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]))*(sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]));
      D+= w_i*((sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]))*(sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q])) - 
	       (sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]))*(sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q])));
    }
  
  // Comment: "usamos la arcontangente cuarto-cuadrante"
  
	N=-2*N;
	alfa=(double)0.5*(double)(atan2(N,D));  
	r=sum_x*cos(alfa) + sum_y*sin(alfa);


   if (r<0) {
		r=-r; 
		alfa+=(double)PI;
		if (alfa>PI) alfa-=2*(double)PI;
	}
	
  segment[_ALFA]=alfa;
  segment[_RO]=r;
  

//   printf("\n[SEGM]: (alfa,r) (%f,%f) inicio(r,th) (%f,%f) (x,y) (%f,%f)\n",alfa*180/PI,r,puntos[3*inicio+1],puntos[3*inicio+2]*180/PI,
//     puntos[3*inicio+1]*cos(puntos[3*inicio+2]),puntos[3*inicio+1]*sin(puntos[3*inicio+2]));


  extremo_kai(puntos[offset_points*inicio+_R],puntos[offset_points*inicio+_Q],alfa,r,&segment[_XINI],&segment[_YINI]); 


//  printf("\n[SEGM]: (alfa,r)  (%f,%f) fin(r,th) (%f,%f) (x,y) (%f,%f)\n",alfa*180/PI,r,puntos[3*fin+1],puntos[3*fin+2]*180/PI,
//	  puntos[3*fin+1]*cos(puntos[3*fin+2]),puntos[3*fin+1]*sin(puntos[3*fin+2]));
  
  extremo_kai(puntos[offset_points*fin+_R],puntos[offset_points*fin+_Q],alfa,r,&segment[_XFIN],&segment[_YFIN]); 
  
  
	segment[_DIST]=sqrt((segment[_XINI]-segment[_XFIN])*(segment[_XINI]-segment[_XFIN]) + 
		      (segment[_YINI]-segment[_YFIN])*(segment[_YINI]-segment[_YFIN]));
  
  
  c_alfa=0; c_r=0; c_ralfa=0;

  
  for (i=inicio;i<=fin;i++)
    {

      dalfa=N*(sum_x*cos(puntos[offset_points*i+_Q]) - sum_y*sin(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*cos(2*puntos[offset_points*i+_Q])) - 
            D*(sum_x*sin(puntos[offset_points*i+_Q]) + sum_y*cos(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*sin(2*puntos[offset_points*i+_Q]));
      
      dalfa=dalfa/(D*D + N*N);

      drho=(1/w)*cos(puntos[offset_points*i+_Q] -alfa) + dalfa*(sum_y*cos(alfa) - sum_x*sin(alfa));

      c_alfa+=w_i*w_i*pow(N*(sum_x*cos(puntos[offset_points*i+_Q]) - sum_y*sin(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*cos(2*puntos[offset_points*i+_Q]))
			  -D*(sum_x*sin(puntos[offset_points*i+_Q]) + sum_y*cos(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*sin(2*puntos[offset_points*i+_Q])),2)*
	          SIGMA_R*SIGMA_R;

      c_r+=pow((w_i/w)*cos((puntos[offset_points*i+_Q])-alfa) + (sum_y*cos(alfa) - sum_x*sin(alfa))*dalfa,2)*SIGMA_R*SIGMA_R;

      c_ralfa+=dalfa*drho*SIGMA_R*SIGMA_R;
      
    }   

  c_alfa=c_alfa/((N*N + D*D)*(N*N + D*D));

  if (c_alfa<0) {printf("[DEBUG line segments: error in covariance matrix. c_alfa is a Negativa value.\n"); 
  }
  segment[_CALFA]=c_alfa*SN_Rth;
  segment[_CR]=c_r*SN_Rxy;
  if (c_r<0) {printf("[DEBUG line segments: error in covariance matrix. c_r is a Negativa value.\n"); 
  }
  segment[_CALFAR]=c_ralfa*SN_Rxy;
  
  
 if ((segment[_CALFA]*segment[_CR] - segment[_CALFAR]*segment[_CALFAR]) >  (MAX_DET_Rs*1e6)) {
    segment[_DIST]=0;
  }
}

// ***************************************************************************
// Funcion: segment_new
//	Caracteriza el segmento entre inicio y fin en pixels
// IN: 
//     double *pixels: puntos del array
//     double inicio,fin: puntos inicial y final
// OUT:
//     double*: devuelve el segmento definitivo. 
// ***************************************************************************
void Worker::segment_new(double *segmento_new, double *pixels, double *points, int inicio, int fin) 
{
  segmento_Kai(segmento_new,points,inicio, fin);
	//pixels=pixels;
}

// ***************************************************************************
// Funcion: incluir_segmento
//	Genera los segmentos detectados por el metodo. 
// IN: 
//     double *segmento: segmento detectado.
//     int _cont: contador de segmentos actuales
//     SEGMENTO: array_segment_local variable que almacena la totalidad del mapa
//
// OUT:
// ***************************************************************************
void Worker::incluir_segmento(SEGMENTO *array_segment_local,double *segmento,int _cont)
{
	int cont_segment;	// Contador de segmentos actuales

	cont_segment=_cont;
  
	// Comment: [LE] Conversion de datos al stma. de referencia utilizado
	array_segment_local[cont_segment].x_ini = segmento[0]; 
	array_segment_local[cont_segment].y_ini = segmento[1]; 
	array_segment_local[cont_segment].x_fin = segmento[2];
	array_segment_local[cont_segment].y_fin = segmento[3];

	// Comment: [LE] Caracterizacion de array_segment_local
	array_segment_local[cont_segment].a = segmento[5];
	array_segment_local[cont_segment].b = segmento[6];

	array_segment_local[cont_segment].longitud = segmento[4];

	array_segment_local[cont_segment].lalfa = segmento[8];
	array_segment_local[cont_segment].lr  = segmento[9];


	array_segment_local[cont_segment].lcalfa  = segmento[10];
	array_segment_local[cont_segment].lcr     = segmento[11];
 	array_segment_local[cont_segment].lcalfar = 0; //segmento[12];
 	
// 	std::cout << "----incluir_segmento----" << std::endl;
// 	std::cout << "segmento: (alfa, d, length)" << array_segment_local[cont_segment].lalfa *180/M_PI << ", " << array_segment_local[cont_segment].lr << ", " << array_segment_local[cont_segment].longitud << std::endl;
// 	std::cout << "segmento: (calfa, crho, calfarho)" << array_segment_local[cont_segment].lcalfa << ", " << array_segment_local[cont_segment].lcr << ", " << array_segment_local[cont_segment].lcalfar << std::endl;
// 	std::cout << "det2R: " << array_segment_local[cont_segment].lcalfa*array_segment_local[cont_segment].lcr - array_segment_local[cont_segment].lcalfar *array_segment_local[cont_segment].lcalfar  << std::endl;
// 	std::cout << "-----------" << std::endl;

}


// ***************************************************************************
// Funcion: incluir_edge
//	Genera los edges detectados por el metodo. 
// IN: 
//     int posx, posy: posicion del edge
//
// OUT:
// ***************************************************************************
void Worker::incluir_edge(MAPA_LASER *mapa, double posx, double posy, double r, double theta, double Calfa, double alfa)
{
	int indice;

	indice=mapa->n_esquinas;
			
	mapa->esquinas[indice*offset_corners +_X]=(double)posx;
	mapa->esquinas[indice*offset_corners+_Y]=(double)posy;
	mapa->esquinas[indice*offset_corners+_THETA]=(double)alfa;
	mapa->esquinas[indice*offset_corners+_TYPEc]=-1;			// Comment: [LE] Edges

	mapa->R_esquinas[indice*offset_Rp]   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + r*r*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
	mapa->R_esquinas[indice*offset_Rp+1] = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - r*r*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
	mapa->R_esquinas[indice*offset_Rp+2] = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + r*r*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

	mapa->R_esquinas[indice*offset_Rp+3] = Calfa;
	

	mapa->n_esquinas+=1;  
}



// ***************************************************************************
// Funcion: detectar_Edge
//	Genera los extremos de los segmentos que generan un borde
// IN: 
//     SEGMENTO *array_segment_local: lista de segmentos
//     double *esquina				: lista de esquinas virtuales
//     int _cont					: contador de segmentos
//
// OUT:
//		Actualiza esquina
// ***************************************************************************

void Worker::detectar_Edge(MAPA_LASER *mapa, int contseg, int *breakpoints,double *array_corners, double *array_pixels,double * array_points)

{

  int inicio, fin;
  double *segmento;

  segmento=(double*)malloc(13*sizeof(double));

  if (breakpoints[contseg+4]!=-1 &&
      (breakpoints[contseg+3+1]-breakpoints[contseg+2]-1)<1
      && MaxDistEuclidea(array_pixels[(breakpoints[contseg+2]-1)*3+1],
			 array_pixels[(breakpoints[contseg+2]-1)*3+2],
			 array_pixels[breakpoints[contseg+3+1]*3+1],
			 array_pixels[breakpoints[contseg+3+1]*3+2],
			 0, 0)==1) // el scan esta en coord locales
    {
      // Posible edge, si esta unido a un segmento...
      // ... que va desde el primer breakpoint o la ultima esquina en ese tramo...

      if (array_corners[0]<=0)
	inicio=breakpoints[contseg+1];
      else
	inicio=((int)array_corners[(((int)array_corners[0])-1)*5+1])+breakpoints[contseg+1];

	  // ... hasta dicho punto
	  fin=breakpoints[contseg+2]-1;
	  // ... y cumple las condiciones de tramo recto
	  if ((fin-inicio)>6)
	    {
	      segment_new(segmento,array_pixels,array_points,inicio,fin);

#ifdef __DEBUG__
		  printf("(alfa,r): (%f,%f)",segmento[_ALFA]*180/PI,segmento[_RO]);
		  printf("(l,error): (%f,%f)",segmento[_DIST]*180/PI,segmento[_ERROR]);
#endif

		  if (segmento[4]>=LONG_SEGMENTO) // && segmento[12]<COEF_MINIMO)
		{
		  // Si todo esta bien, se incluye...
		  incluir_edge(mapa, array_pixels[(breakpoints[contseg+2]-1)*3+2],
			       -array_pixels[(breakpoints[contseg+2]-1)*3+1], array_points[(breakpoints[contseg+2]-1)*3+_R], 
			       array_points[(breakpoints[contseg+2]-1)*3+_Q], segmento[10],segmento[_ALFA]);
		}
	    }
    }
  
  
  // Incluir edges (primer breakpoint desde contseg=3)...
  if (contseg>0
      &&
      (breakpoints[contseg+1]-breakpoints[contseg-3+2]-1)<1
      && MaxDistEuclidea(array_pixels[(breakpoints[contseg-3+2]-1)*3+1],
			 array_pixels[(breakpoints[contseg-3+2]-1)*3+2],
			 array_pixels[breakpoints[contseg+1]*3+1],
			 array_pixels[breakpoints[contseg+1]*3+2],
			 0, 0)==2) // el scan esta en coord locales
    {
      // Posible edge, si está unido a un segmento...
      // ... que va desde el primer breakpoint...
      inicio=breakpoints[contseg+1];
      // ... hasta el ultimo breakpoint o la primera esquina en ese tramo...
      if (array_corners[0]<=0)
	fin=breakpoints[contseg+2]-1;
      else
	fin=((int)array_corners[1])+breakpoints[contseg+1];
      // ... y cumple las condiciones de tramo recto
      if ((fin-inicio)>6)
	{
	  segment_new(segmento,array_pixels,array_points,inicio,fin);
	  if ((segmento[4]>=LONG_SEGMENTO)) // && (segmento[12]<COEF_MINIMO))
	    {
	      incluir_edge(mapa, array_pixels[breakpoints[contseg+1]*3+2],
			   -array_pixels[breakpoints[contseg+1]*3+1],array_points[(breakpoints[contseg+2]-1)*3+_R], 
			   array_points[(breakpoints[contseg+2]-1)*3+_Q],segmento[10],segmento[_ALFA]);
	    }
	}
    }
  free(segmento); 
}

// ***************************************************************************
// Funcion: matriz_covarianza_circulo
//	calcula la matriz de covarianza para un circulo dado tres puntos
// IN: circulo: circulo al que calcular la matriz
//	   matriz:  matriz de covarianza
//     cont:    contardor de circulos en la observacion actual
// OUT:
// 
// ***************************************************************************

void  Worker::matriz_covarianza_circulo(CIRCULO circle, double *matriz, int cont)
{
	double N,D; 
	
	double a, b, c, d, e, f;

	a = circle.vector[0]; b = circle.vector[1]; c = circle.vector[2]; d = circle.vector[3]; e = circle.vector[4]; f = circle.vector[5];

	double dadx1, dbdx1, dcdx1, dddx1, dedx1, dfdx1;
	double dadx2, dbdx2, dcdx2, dddx2, dedx2, dfdx2;
	double dadx3, dbdx3, dcdx3, dddx3, dedx3, dfdx3;

	double dady1, dbdy1, dcdy1, dddy1, dedy1, dfdy1;
	double dady2, dbdy2, dcdy2, dddy2, dedy2, dfdy2;
	double dady3, dbdy3, dcdy3, dddy3, dedy3, dfdy3;
	
	double dRdx1, dRdx2, dRdx3, dRdy1, dRdy2, dRdy3;

	dadx1 = -2; dbdx1 =  0; dcdx1 =  2*circle.puntos[0]; dddx1 = -2; dedx1 = 0; dfdx1 = 2*circle.puntos[0];
	dadx2 =  2; dbdx2 =  0; dcdx2 = -2*circle.puntos[2]; dddx2 =  0; dedx2 = 0; dfdx2 = 0;
	dadx3 =  0; dbdx3 =  0; dcdx3 =  0; dddx3 = 2; dedx3 = 0; dfdx3 = -2*circle.puntos[4];
	dady1 =  0; dbdy1 = -2; dcdy1 =  2*circle.puntos[1]; dddy1 = 0; dedy1 = -2; dfdy1 = 2*circle.puntos[1];
	dady2 =  0; dbdy2 =  2; dcdy2 = -2*circle.puntos[3]; dddy2 = 0; dedy2 = 0;  dfdy2 = 0;
	dady3 =  0; dbdy3 =  0; dcdy3 =  0; dddy3 = 0; dedy3 = 2; dfdy3 = -2*circle.puntos[5];
	
	double dxdx1, dxdx2, dxdx3, dxdy1, dxdy2, dxdy3, dydx1, dydx2, dydx3, dydy1, dydy2, dydy3, drdx1, drdx2, drdx3, drdy1, drdy2, drdy3;

	double SIGMA_X1,SIGMA_Y1,SIGMA_XY1;
	double SIGMA_X2,SIGMA_Y2,SIGMA_XY2;
	double SIGMA_X3,SIGMA_Y3,SIGMA_XY3;

	double theta, rho;

	double R;

	double M11, M12, M13, M14, M15, M16, M21, M22, M23, M24, M25, M26, M31, M32, M33, M34, M35, M36;

#ifdef DEBUG_CR
	printf("(x,y): (%f,%f)|(%f,%f)|(%f,%f)\n",circle.puntos[0],circle.puntos[1],
		circle.puntos[2],circle.puntos[3],circle.puntos[4],circle.puntos[5]);
#endif

	N = a*f - c*d;
	D = b*d - a*e;
	R = (circle.puntos[0] - circle.x)*(circle.puntos[0] - circle.x) 
		+ ((circle.puntos[1] - circle.y)*(circle.puntos[1] - circle.y));

#ifdef DEBUG_CR
	printf("(xc,yc,r): (%f,%f,%f)\n",(N*circle.vector[1]/(D*circle.vector[0]) +circle.vector[2]/circle.vector[0]),N/D,sqrt(R));
	printf("f: %f", circle.puntos[0]*circle.puntos[0] + circle.puntos[1]*circle.puntos[1] - circle.puntos[4]*circle.puntos[4] - circle.puntos[5]*circle.puntos[5]);
#endif

	
	dydx1  = (((dadx1*f + dfdx1*a) - (dcdx1*d + dddx1*c))*D - ((dbdx1*d + dddx1*b) - (dadx1*e + dedx1*a))*N)/(D*D);
	dydx2  = (((dadx2*f + dfdx2*a) - (dcdx2*d + dddx2*c))*D - ((dbdx2*d + dddx2*b) - (dadx2*e + dedx2*a))*N)/(D*D);
	dydx3  = (((dadx3*f + dfdx3*a) - (dcdx3*d + dddx3*c))*D - ((dbdx3*d + dddx3*b) - (dadx3*e + dedx3*a))*N)/(D*D);

	dydy1  = (((dady1*f + dfdy1*a) - (dcdy1*d + dddy1*c))*D - ((dbdy1*d + dddy1*b) - (dady1*e + dedy1*a))*N)/(D*D);
	dydy2  = (((dady2*f + dfdy2*a) - (dcdy2*d + dddy2*c))*D - ((dbdy2*d + dddy2*b) - (dady2*e + dedy2*a))*N)/(D*D);
	dydy3  = (((dady3*f + dfdy3*a) - (dcdy3*d + dddy3*c))*D - ((dbdy3*d + dddy3*b) - (dady3*e + dedy3*a))*N)/(D*D);

	dxdx1  = -(dydx1*b/a + N/D*(dbdx1*a - dadx1*b)/(a*a) + (dcdx1*a - dadx1*c)/(a*a));
	dxdx2  = -(dydx2*b/a + N/D*(dbdx2*a - dadx2*b)/(a*a) + (dcdx2*a - dadx2*c)/(a*a));
	dxdx3  = -(dydx3*b/a + N/D*(dbdx3*a - dadx3*b)/(a*a) + (dcdx3*a - dadx3*c)/(a*a));

	dxdy1  = -(dydx1*b/a + N/D*(dbdy1*a - dady1*b)/(a*a) + (dcdy1*a - dady1*c)/(a*a));
	dxdy2  = -(dydx2*b/a + N/D*(dbdy2*a - dady2*b)/(a*a) + (dcdy2*a - dady2*c)/(a*a));
	dxdy3  = -(dydx3*b/a + N/D*(dbdy3*a - dady3*b)/(a*a) + (dcdy3*a - dady3*c)/(a*a));

	dRdx1 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx1) + 2*(circle.puntos[1] - circle.y)*(0 - dydx1));
	dRdx2 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx2) + 2*(circle.puntos[1] - circle.y)*(0 - dydx2));
	dRdx3 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx3) + 2*(circle.puntos[1] - circle.y)*(0 - dydx3));
	dRdy1 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy1) + 2*(circle.puntos[1] - circle.y)*(0 - dydy1));
	dRdy2 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy2) + 2*(circle.puntos[1] - circle.y)*(0 - dydy2));
	dRdy3 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy3) + 2*(circle.puntos[1] - circle.y)*(0 - dydy3));

	drdx1 = (1/(2*sqrt(R)))*dRdx1; 
	drdx2 = (1/(2*sqrt(R)))*dRdx2;
	drdx3 = (1/(2*sqrt(R)))*dRdx3;
	drdy1 = (1/(2*sqrt(R)))*dRdy1;
	drdy2 = (1/(2*sqrt(R)))*dRdy2;
	drdy3 = (1/(2*sqrt(R)))*dRdy3;

	// Comment: "[LE] parametros para la matriz de covarianza"
	rho   = sqrt( circle.puntos[0]*circle.puntos[0] + circle.puntos[1]*circle.puntos[1]);
	theta = atan2(circle.puntos[0],circle.puntos[1])-PI/2;
	
#ifdef DEBUG_CR
	printf("(r,theta): (%f,%f)\n",rho,theta);		
#endif
	if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
	printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);		
#endif

	
	SIGMA_X1   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
	SIGMA_XY1  = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
	SIGMA_Y1   = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

	rho = sqrt(circle.puntos[2]*circle.puntos[2] + circle.puntos[3]*circle.puntos[3]);
	theta = atan2(circle.puntos[2],circle.puntos[3])-PI/2;

	if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
	printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);
#endif

	SIGMA_X2   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
	SIGMA_XY2  = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
	SIGMA_Y2   = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);
	
	rho=sqrt(circle.puntos[4]*circle.puntos[4]+circle.puntos[5]*circle.puntos[5]);
	theta= atan2(circle.puntos[4],circle.puntos[5])-PI/2;
	if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
	printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);
#endif
	SIGMA_X3   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
	SIGMA_XY3   = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
	SIGMA_Y3  = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

#ifdef DEBUG_CR
	printf("sigmas: (%f,%f,%f) (%f,%f,%f) (%f,%f,%f) ", SIGMA_X1,SIGMA_Y1,SIGMA_XY1,
		SIGMA_X2,SIGMA_Y2,SIGMA_XY2,SIGMA_X3,SIGMA_Y3,SIGMA_XY3);
#endif	

	M11= dxdx1 * SIGMA_X1  + dxdy1 * SIGMA_XY1;
	M12= dxdx1 * SIGMA_XY1 + dxdy1 * SIGMA_Y1;
	M13= dxdx2 * SIGMA_X2  + dxdy2 * SIGMA_XY2;
	M14= dxdx2 * SIGMA_XY2 + dxdy2 * SIGMA_Y2;
	M15= dxdx3 * SIGMA_X3  + dxdy3 * SIGMA_XY3;
	M16= dxdx3 * SIGMA_XY3 + dxdy3 * SIGMA_Y3;

	M21= dydx1 * SIGMA_X1  + dydy1 * SIGMA_XY1;
	M22= dydx1 * SIGMA_XY1 + dydy1 * SIGMA_Y1;
	M23= dydx2 * SIGMA_X2  + dydy2 * SIGMA_XY2;
	M24= dydx2 * SIGMA_XY2 + dydy2 * SIGMA_Y2;
	M25= dydx3 * SIGMA_X3  + dydy3 * SIGMA_XY3;
	M26= dydx3 * SIGMA_XY3 + dydy3 * SIGMA_Y3;

	M31= drdx1 * SIGMA_X1  + drdy1 * SIGMA_XY1;
	M32= drdx1 * SIGMA_XY1 + drdy1 * SIGMA_Y1;
	M33= drdx2 * SIGMA_X2  + drdy2 * SIGMA_XY2;
	M34= drdx2 * SIGMA_XY2 + drdy2 * SIGMA_Y2;
	M35= drdx3 * SIGMA_X3  + drdy3 * SIGMA_XY3;
	M36= drdx3 * SIGMA_XY3 + drdy3 * SIGMA_Y3;

	
	matriz[offset_Rc*cont]  = M11*dxdx1 + M12*dxdy1 + M13*dxdx2 + M14*dxdy2 + M15*dxdx3 + M16*dxdy3; // sigmaxx
	matriz[offset_Rc*cont+1]= M21*dydx1 + M22*dydy1 + M23*dydx2 + M24*dydy2 + M25*dydx3 + M26*dydy3; // sigmayy

	matriz[offset_Rc*cont+2]= M11*dydx1 + M12*dydy1 + M13*dydx2 + M14*dydy2 + M15*dydx3 + M16*dydy3; // sigmaxy
	matriz[offset_Rc*cont+3]= M31*drdx1 + M32*drdy1 + M33*drdx2 + M34*drdy2 + M35*drdx3 + M36*drdy3; // sigmaphopho

	matriz[offset_Rc*cont+4]= M11*drdx1 + M12*drdy1 + M13*drdx2 + M14*drdy2 + M15*drdx3 + M16*drdy3; // sigmaxpho
	matriz[offset_Rc*cont+5]= M21*drdx1 + M22*drdy1 + M23*drdx2 + M24*drdy2 + M25*drdx3 + M26*drdy3; // sigmaypho
}

// ***************************************************************************
// Funcion: circulo_new()
//	calcula el circulo del array de puntos pasado como parametro
// IN:  array_pixels: array de puntos del laser
//	    inicio: inicio del segmento circular
//	    fin: fin del segmento circular
// OUT: devuelve el los parametros del circulo que forman esos puntos
// 
// ***************************************************************************

CIRCULO Worker::circulo_new(double *array_pixels,int inicio,int fin){

	CIRCULO local;
	local.x=0;local.y=0;local.r=0;

	int IterMAX = 10, i; 

	double *x; double *y; double *s;
	double distance;
	double min; int indice_s;
	Circle Ini, circulo;
	
	x=(double*) malloc((fin-inicio)*sizeof(double));
	y=(double*) malloc((fin-inicio)*sizeof(double));
	s=(double*) malloc(IterMAX*sizeof(double));
	
	// Comment: "[LE] Generamos los datos para la parametrizacion del circulo"
	for (int m=0;m<(fin-inicio);m++)
		{
		x[m]=(double)array_pixels[3*(inicio+m)+_X];	
		y[m]=(double)array_pixels[3*(inicio+m)+_Y];
	}

	distance=sqrt((x[0]-x[fin-inicio-1])*(x[0]-x[fin-inicio-1])+(y[0]-y[fin-inicio-1])*(y[0]-y[fin-inicio-1]));

	Data data((fin-inicio),x,y);
	min=10000;

	// Comment: "[LE] Iteramon IterMAX para obtener la circunferencia m�s adecuada con esos puntos
	//          --> s[indice]_minima
	for (int indice=0;indice<IterMAX-1;indice++)
	{
	Ini=Circle(data,fin-inicio);
	s[indice]= Sigma(data,Ini); 
					 
		if	(s[indice]<=min) 
		{
		min=s[indice]; indice_s=indice;
		circulo = Circle(Ini.a, Ini.b,Ini.r,Ini.vector, Ini.puntos);
		}															
	}
								
	free(x); free(y); free(s); // Comment: added by Ricardo Vazquez

	if ((circulo.r < 500) && (min<10) && (circulo.r > 150) && (distance>circulo.r))
	{
		local.x = (double)circulo.a; local.y=(double)circulo.b; local.r=(double)circulo.r;
		
		for (i=0; i< offset_Rc; i++) { // Comment: covariance data added by Ricardo Vazquez
		  local.puntos[i] = circulo.puntos[i];
		  local.vector[i] = circulo.vector[i];
		}
	}

	return local;
}

// ***************************************************************************
// Funcion: incluir_circulo
// incluye el circulo en la lista de features detectadas
// IN: mapa:               mapa local del entorno
//	   array_circle_local: array de circulos de la observacion actual
//     circle:             circulo a introducir
//     cont_circle:        contador de circulos
//     matriz:             matriz de covarianza del circulo
// OUT:
// 
// ***************************************************************************

void Worker::incluir_circulo(MAPA_LASER *mapa, CIRCULO *array_circle_local, CIRCULO circle, int cont_circle, double *matriz)
{
	int i; // Comment: [LE] added by Ricardo Vazquez

	array_circle_local[cont_circle].x = circle.x;
	array_circle_local[cont_circle].y = circle.y;
	array_circle_local[cont_circle].r = circle.r;

	for(i=0; i<6; i++) { 
	  array_circle_local[cont_circle].vector[i] = circle.vector[i];
	  array_circle_local[cont_circle].puntos[i] = circle.puntos[i];
	}

	matriz_covarianza_circulo(circle, matriz, cont_circle);
	
	mapa->circulos[cont_circle*offset_circles+_X]=circle.x;
	mapa->circulos[cont_circle*offset_circles+_Y]=circle.y;
	mapa->circulos[cont_circle*offset_circles+_RADIUS]=circle.r;

	mapa->R_circulos[cont_circle*offset_Rc]  =matriz[0];
	mapa->R_circulos[cont_circle*offset_Rc+1]=matriz[1];
	mapa->R_circulos[cont_circle*offset_Rc+2]=matriz[2];
	mapa->R_circulos[cont_circle*offset_Rc+3]=matriz[3];
	mapa->R_circulos[cont_circle*offset_Rc+4]=matriz[4];
	mapa->R_circulos[cont_circle*offset_Rc+5]=matriz[5];

	mapa->n_circulos++;
}

// ***************************************************************************
// Funcion: matriz_covarianza_virtual
//	calcula la matriz de covarianza para una esquina virtual
// IN: 
//
// OUT:
// 
// ***************************************************************************


void  Worker::matriz_covarianza_virtual(SEGMENTO segmento_2,SEGMENTO segmento_1, double *matriz, int cont)
{

  // definicion de parametros tmps
  double A,B,C,D,E,F,G,H, I, J;
  double A_,B_,C_,D_,E_,F_,G_,H_;

  double senodif_alfa, seno_alfa1, seno_alfa2; 
  double cosenodif_alfa, coseno_alfa1, coseno_alfa2;
  
  // calculo de parametros

  senodif_alfa   = sin(segmento_2.lalfa - segmento_1.lalfa);
  seno_alfa1     = sin(segmento_1.lalfa);
  seno_alfa2     = sin(segmento_2.lalfa);
  cosenodif_alfa = sin(segmento_2.lalfa - segmento_1.lalfa);
  coseno_alfa1   = cos(segmento_1.lalfa);    
  coseno_alfa2   = cos(segmento_2.lalfa);    
    
  A = (seno_alfa2)/(senodif_alfa);
  B = (-segmento_2.lr*coseno_alfa1*senodif_alfa + cosenodif_alfa*(segmento_1.lr*seno_alfa2 - segmento_2.lr*seno_alfa1))/(senodif_alfa*senodif_alfa);
  C = -seno_alfa1/(senodif_alfa);
  D = (segmento_1.lr*coseno_alfa2*senodif_alfa -(segmento_1.lr*seno_alfa2 - segmento_2.lr*seno_alfa1)*cosenodif_alfa)/(senodif_alfa*senodif_alfa);
  E = -coseno_alfa2/senodif_alfa; 
  F = (-segmento_2.lr*senodif_alfa*seno_alfa1 +cosenodif_alfa*(segmento_2.lr*coseno_alfa1 - segmento_1.lr*coseno_alfa2))/(senodif_alfa*senodif_alfa);
  G = coseno_alfa1/(senodif_alfa);
  H = (segmento_1.lr*seno_alfa2*senodif_alfa - cosenodif_alfa*(segmento_2.lr*coseno_alfa1 - segmento_1.lr*coseno_alfa2))/(senodif_alfa*senodif_alfa);
  I = -0.5;
  J = 0.5;

  A_ = A*segmento_1.lcr + B*segmento_1.lcalfar;
  B_ = A*segmento_1.lcalfar + B*segmento_1.lcalfa;

  C_ = C*segmento_2.lcr + D*segmento_2.lcalfar;
  D_ = C*segmento_2.lcalfar + D*segmento_2.lcalfa;
  
  E_ = E*segmento_1.lcr + F*segmento_1.lcalfar;
  F_ = E*segmento_1.lcalfar + F*segmento_1.lcalfa;
  
  G_ = G*segmento_2.lcr + H*segmento_2.lcalfar;
  H_ = G*segmento_2.lcalfar + H*segmento_2.lcalfa;

  matriz[offset_Rp*cont+0] = A*A_ + B*B_ + C*C_ + D*D_;
  matriz[offset_Rp*cont+1] = E*E_ + F*F_ + G*G_ + H*H_;
 
  matriz[offset_Rp*cont+2] = A*E_ + B*F_ + C*G_ + D*H_;
  
  
  // comprobaciones
#ifdef DEBUGVC
  std::cout<<"[VC] sigma_x: "<<matriz[offset_Rp*cont+0]<<" sigma_y: "<<matriz[offset_Rp*cont+1]<<std::endl;
  std::cout<<"[VC] sigma_xy:"<<matriz[offset_Rp*cont+2]<<" sigma_yx:"<<(A_*E + B_*F + C_*G + D_*H)<<std::endl;

  if (matriz[offset_Rp*cont+0]<0 || matriz[offset_Rp*cont +1]<0) {
    std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores negativos)"<<std::endl;
  }

  if (matriz[offset_Rp*cont+2]!= (A_*E + B_*F + C_*G + D_*H)){
    std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
	std::cout<<"[VC ERROR] sigma_xy:"<<matriz[offset_Rp*cont+2]<<" sigma_yx:"<<(A_*E + B_*F + C_*G + D_*H)<<std::endl;
  }
  
 #endif

  matriz[offset_Rp*cont+4] = B_*I+D_*J;
  matriz[offset_Rp*cont+5] = F_*I+H_*J;

  matriz[offset_Rp*cont+3] = I*segmento_1.lcalfa*I + J*segmento_2.lcalfa*J;

#ifdef DEBUGVC
  std::cout<<"[VC] sigma_alfa: "<<matriz[offset_Rp*cont+3]<<std::endl;
  std::cout<<"[VC] sigma_xalfa: "<<matriz[offset_Rp*cont+4]<<std::endl;
  std::cout<<"[VC] sigma_yalfa: "<<matriz[offset_Rp*cont+5]<<std::endl;

	if (matriz[offset_Rp*cont+4]!= (I*segmento_1.lcalfar*A+I*segmento_1.lcalfa*B+J*segmento_2.lcalfar*C+J*segmento_2.lcalfa*D)){
		std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
		std::cout<<"[VC ERROR] sigma_xalfa:"<<matriz[offset_Rp*cont+4]<<" sigma_alfax:"
			 << (I*segmento_1.lcalfar*A+I*segmento_1.lcalfa*B+J*segmento_2.lcalfar*C+J*segmento_2.lcalfa*D)<<std::endl;
	}
 
	if (matriz[offset_Rp*cont+5]!= (I*segmento_1.lcalfar*E+I*segmento_1.lcalfa*F+J*segmento_2.lcalfar*G+J*segmento_2.lcalfa*H)){
		std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
		std::cout<<"[VC ERROR] sigma_yalfa:"<<matriz[offset_Rp*cont+5]<<" sigma_alfay:"
			 <<(I*segmento_1.lcalfar*E+I*segmento_1.lcalfa*F+J*segmento_2.lcalfar*G+J*segmento_2.lcalfa*H)<<std::endl;
	}
#endif

}


// ***************************************************************************
// Funcion: detectar_EsquinaVirtual
//	Genera esquinas virtuales para localizacion. Si rectas paralelas, las rechaza
//	Si rectas secantes y comparten esquinas, las rechaza
// IN: 
//     SEGMENTO *array_segment_local: lista de segmentos
//     double *esquina				: lista de esquinas virtuales
//     int _cont					: contador de segmentos
//
// OUT:
//		Actualiza esquina
// ***************************************************************************
void Worker::detectar_EsquinaVirtual(SEGMENTO *array_segment_local,double *esquina, int _cont, double *matriz_R)
{

	register int i;			
	int n_esquina;					// Numero de esquinas virtuales encontradas

	double pc_x,pc_y;				// Punto de corte entre dos rectas
	int array_c;					// vble. auxiliar

	double inc_angle;

	n_esquina=(int)(esquina[0]);

	array_c=0;
  
	pc_x=0; pc_y=0;

	if (_cont!=1) 
	  { // Comment: [LE] si no existen segmentos todavia"
		for (i=0;i<_cont;i++)
			{
			
				inc_angle=(array_segment_local[_cont-1].lalfa-array_segment_local[i].lalfa)*180/PI;
				
				if (inc_angle<0)
					inc_angle=inc_angle+180;

				if (inc_angle>=180)
					inc_angle=inc_angle-180;
		
			  // Comment: [LE] Se detecta si son rectas casi-paralelas: si es asi, se cancela la busqueda"
			  if ((fabs(inc_angle)<30) ||
			      (fabs(inc_angle)>150))
				{
					
				}
			  else 
			    {
				  // Comment: [LE] El punto de corte define la esquina virtual
			    
			      pc_y=(array_segment_local[i].lr*cos(array_segment_local[(_cont-1)].lalfa) - array_segment_local[(_cont-1)].lr*cos(array_segment_local[i].lalfa));
			      pc_y=pc_y/(sin(array_segment_local[i].lalfa)*cos(array_segment_local[_cont-1].lalfa)- sin(array_segment_local[(_cont-1)].lalfa)*cos(array_segment_local[i].lalfa)); 

				  pc_x= (array_segment_local[(_cont-1)].lr - pc_y*sin(array_segment_local[(_cont-1)].lalfa))/
				(cos(array_segment_local[(_cont-1)].lalfa));

#ifdef DEBUGVC				
				  printf("(Xv,Yv) (%f,%f) - punto de corte entre: \n", pc_x,pc_y);
				  printf("A(rho,theta): (%f,%f) B(rho,theta): (%f,%f) \n",array_segment_local[i].lr,array_segment_local[i].lalfa*180/PI,
					  array_segment_local[(_cont-1)].lr,array_segment_local[(_cont-1)].lalfa*180/PI); 
#endif
			      esquina[n_esquina*offset_corners + _X]= pc_x;
			      esquina[n_esquina*offset_corners + _Y]= pc_y;
			      esquina[n_esquina*offset_corners + _THETA]= (array_segment_local[(_cont-1)].lalfa + array_segment_local[i].lalfa)/2;

				  esquina[n_esquina*offset_corners + _ALFA1]= array_segment_local[(_cont-1)].lalfa;
				  esquina[n_esquina*offset_corners + _ALFA2]= array_segment_local[i].lalfa;
				  esquina[n_esquina*offset_corners + _RHO1] = array_segment_local[(_cont-1)].lr;
				  esquina[n_esquina*offset_corners + _RHO2] = array_segment_local[i].lr;

			      // Comment: [LE] para la inclusion, incertidumbre
			      matriz_covarianza_virtual(array_segment_local[(_cont-1)],array_segment_local[i],matriz_R,n_esquina);

#ifdef DEBUGVC				
				  printf("matriz_R[0]: %f ", matriz_R[n_esquina*offset_Rp]);
				  printf("matriz_R[1]: %f \n", matriz_R[n_esquina*offset_Rp+1]);
				  printf("matriz_R[2]: %f ", matriz_R[n_esquina*offset_Rp+2]);
				  printf("matriz_R[3]: %f \n", matriz_R[n_esquina*offset_Rp+3]);
#endif
				 
			      n_esquina++;
		       
			    }
					
			}
	  }
	esquina[0]=(double)n_esquina;
}

// ***************************************************************************
// Funcion: iniciar_mapa_local
//	Inicializa las caracteristica del mapa que sera construido para la medida actual
// IN: 
//
// OUT:
// 
// ***************************************************************************

void Worker::iniciar_mapa_local( MAPA_LASER & mapa)
{
    
	mapa.n_esquinas=0;
	mapa.esquinas=(double*)malloc(N_CARACT*offset_corners*sizeof(double));             
	mapa.R_esquinas=(double*)malloc(N_CARACT*offset_Rp*sizeof(double));

	mapa.n_segmentos=0;
	mapa.segmentos=(double*)malloc(N_CARACT*offset_segments*sizeof(double));
	mapa.R_segmentos=(double*)malloc(N_CARACT*offset_Rs*sizeof(double));

	mapa.n_circulos=0;
	mapa.circulos=(double*)malloc((N_CARACT*offset_circles+1)*sizeof(double));
	mapa.R_circulos=(double*)malloc(N_CARACT*offset_Rc*sizeof(double));

}

void Worker::destruir_mapa_local(MAPA_LASER *mapa) // (added by Ricardo Vazquez)
{
  free(mapa->esquinas);
  free(mapa->R_esquinas);
  free(mapa->segmentos);
  free(mapa->R_segmentos);
  free(mapa->circulos);
  free(mapa->R_circulos);
}

double Worker::dist_euclidean2D(double x1, double y1, double x2, double y2){

#ifdef DEBUGCCDA
	// printf("[DistEuclidea] %f\n",  sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
#endif


	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

}


// ***************************************************************************
// Funcion: incluir_esquina_virtual
//	Genera las esquinas detectadas por el metodo. 
// IN: 
//     double *esquinas: array con las esquinas virtuales detectadas. Se actualizan en el codigo.
//     mapa:            mapa del entorno actual de observacion
//     matriz:          matriz de covarianza de la esquina virtual
//
// OUT:
//	   Actualizar mapa
// ***************************************************************************
void Worker::incluir_esquina_virtual(MAPA_LASER *mapa, double *esquinas, double *matriz)
{
	int i,j, cont,indice;
	bool exit;

	cont=0;
	indice=(int)mapa->n_esquinas;

	// Comment: [LE] Comprobamos si hay esquinas reales que coinciden con las virtuales

	for (i=0;i<esquinas[0];i++)
		{
		exit =false;

	
		// PMNT Dic. 08
		for (j =0; j< mapa->n_esquinas; j++) {
		  if (dist_euclidean2D(esquinas[i*offset_corners   +_X], esquinas[i*offset_corners   +_Y], 
			  mapa->esquinas[j*offset_corners + _X], mapa->esquinas[j*offset_corners + _Y]) < 100) {
				exit = true;

		  }
		}
	
		
		// Comment: [LE] Adaptamos al stma. de referencia del robot
		if (!exit)
		  {

			  
		    mapa->esquinas[(indice+i)*offset_corners + _X]= esquinas[i*offset_corners   +_X];
		    mapa->esquinas[(indice+i)*offset_corners +_Y]= esquinas[i*offset_corners +_Y];
		    mapa->esquinas[(indice+i)*offset_corners +_THETA] = esquinas[i*offset_corners +_THETA];

			mapa->esquinas[(indice+i)*offset_corners +_ALFA1] = esquinas[i*offset_corners +_ALFA1];
			mapa->esquinas[(indice+i)*offset_corners +_ALFA2] = esquinas[i*offset_corners +_ALFA2];
			mapa->esquinas[(indice+i)*offset_corners +_RHO1] = esquinas[i*offset_corners +_RHO1];
			mapa->esquinas[(indice+i)*offset_corners +_RHO2] = esquinas[i*offset_corners +_RHO2];

			mapa->esquinas[(indice+i)*offset_corners + _TYPEc]= 0; // Comment: Esquina virtual

		    mapa->R_esquinas[(indice+i)*offset_Rp]=matriz[i*offset_Rp];
		    mapa->R_esquinas[(indice+i)*offset_Rp+1]=matriz[i*offset_Rp+1];
		    mapa->R_esquinas[(indice+i)*offset_Rp+2]=matriz[i*offset_Rp+2];
		    mapa->R_esquinas[(indice+i)*offset_Rp+3]=matriz[i*offset_Rp+3];
		    mapa->R_esquinas[(indice+i)*offset_Rp+4]=matriz[i*offset_Rp+4];
		    mapa->R_esquinas[(indice+i)*offset_Rp+5]=matriz[i*offset_Rp+5];

#ifdef DEBUGVC				
		    std::cout << "coord esq.: " << mapa->esquinas[mapa->n_esquinas*offset_corners + _X] << " " << mapa->esquinas[mapa->n_esquinas*offset_corners+_Y] 
			      << " " << mapa->esquinas[mapa->n_esquinas*4+_THETA] << " (" << mapa->esquinas[j*4+2] << ")" << std::endl; 
		    std::cout << "matriz_R[0]: " << mapa->R_esquinas[(indice+i)*offset_Rp] << "\nmatriz_R[1]: " 
			      <<  mapa->R_esquinas[(indice+i)*offset_Rp+1] << "\nmatriz_R[2]: " 
			      << mapa->R_esquinas[(indice+i)*offset_Rp+2] << "\nmatriz_R[3]: " 
			      << mapa->R_esquinas[(indice+i)*offset_Rp+3] << std::endl;
#endif

			if (mapa->R_esquinas[(indice+i)*offset_Rp]<100000) {

#ifdef DEBUGVC				
			printf("VC=[%f,%f]; P=[%f %f;%f %f; %f %f]\n",-mapa->esquinas[mapa->n_esquinas*4+1],mapa->esquinas[mapa->n_esquinas*4],mapa->R_esquinas[(indice+i)*offset_Rp],mapa->R_esquinas[(indice+i)*offset_Rp+2],mapa->R_esquinas[(indice+i)*offset_Rp+1],mapa->R_esquinas[(indice+i)*offset_Rp+3],mapa->R_esquinas[(indice+i)*offset_Rp+4],mapa->R_esquinas[(indice+i)*offset_Rp+5]);		        
#endif			
			}



		    mapa->n_esquinas++;
		  }
		}
	
}

// ***************************************************************************
// Funcion: obtener_mapa
//	Actualiza el mapa local a partir del array de segmentos
// IN: 
//     _cont				 : contador de segmentos actuales
//     array_segment_local   : variable que almacena la totalidad del mapa
//     mapa                  : variable a actualizar
//
// OUT:
// 
// ***************************************************************************
void  Worker::obtener_mapa(MAPA_LASER *mapa, SEGMENTO *array_segment_local,int *_cont)
{
	int i;
	int X_INI,Y_INI,X_FIN,Y_FIN;
	int RO, THETA, ALFA, R;
	int n_segm;

	// Comment: "[LE] Segmentos"
	n_segm=*_cont;
	X_INI=1; Y_INI=2; X_FIN=3; Y_FIN=4;
	RO=1; THETA=2; ALFA=5; R=6;

	mapa->n_segmentos=(int)n_segm;

	for (i=0;i<n_segm;i++) 
		{
		mapa->segmentos[i*offset_segments+X_INI]=array_segment_local[i].x_ini;
		mapa->segmentos[i*offset_segments+Y_INI]=array_segment_local[i].y_ini;	
		mapa->segmentos[i*offset_segments+X_FIN]=array_segment_local[i].x_fin;
		mapa->segmentos[i*offset_segments+Y_FIN]=array_segment_local[i].y_fin;

		mapa->segmentos[i*offset_segments+ALFA]=array_segment_local[i].lalfa;
		mapa->segmentos[i*offset_segments+R]=array_segment_local[i].lr;
		
		mapa->R_segmentos[i*offset_Rs]=array_segment_local[i].lcalfa;    // sigmaalfaalfa
		mapa->R_segmentos[i*offset_Rs+1]=array_segment_local[i].lcr;     // sigmaphopho
		mapa->R_segmentos[i*offset_Rs+2]=array_segment_local[i].lcalfar; // sigmaalfapho
		mapa->R_segmentos[i*offset_Rs+3]=array_segment_local[i].longitud;
		
// 		std::cout << "----obtener_mapa----" << std::endl;
// 		std::cout << "segmento: (alfa, d, length)" << mapa->segmentos[i*offset_segments+ALFA] *180/M_PI << ", " << mapa->segmentos[i*offset_segments+R] << ", " << mapa->R_segmentos[i*offset_Rs+3] << std::endl;
// 		std::cout << "segmento: (calfa, crho, calfarho)" << mapa->R_segmentos[i*offset_Rs] << ", " << mapa->R_segmentos[i*offset_Rs + 1] << ", " << mapa->R_segmentos[i*offset_Rs + 2] << std::endl;
// 		std::cout << "det2R: " << mapa->R_segmentos[i*offset_Rs]*mapa->R_segmentos[i*offset_Rs + 1] - mapa->R_segmentos[i*offset_Rs + 2] * mapa->R_segmentos[i*offset_Rs + 2] << std::endl;
// 		std::cout << "-----------" << std::endl;
			
		}
}


// ***************************************************************************
// Funcion: map_copy
//	copy the map into the reference one
// IN: 
//     ref                   : ref mapa
//     mapa                  : current mapa
//
// OUT:
// 
// ***************************************************************************
void  Worker::map_copy(MAPA_LASER *mapa, MAPA_LASER *ref)
{
	ref->n_segmentos = mapa->n_segmentos;
	ref->n_esquinas  = mapa->n_esquinas;
	ref->n_circulos  = mapa->n_circulos;

	ref->segmentos   = mapa->segmentos;
	ref->R_segmentos = mapa->R_segmentos;
	ref->esquinas    = mapa->esquinas;
	ref->R_esquinas  = mapa->R_esquinas;
	ref->circulos    = mapa->circulos;
	ref->R_circulos  = mapa->R_circulos;
	
}

// ***************************************************************************
// Funcion: incluir_esquina
//	Genera las esquinas detectadas por el metodo. 
// IN: 
//     double *esquinas: array con las esquinas detectadas. Se actualizan en el codigo.
//
// OUT:
// ***************************************************************************
void Worker::incluir_esquina(MAPA_LASER *mapa, double *esquinas, double *points, int inicio)
{
  int indice,i;
  int cont=0;	

  double r, theta;
  
  indice=mapa->n_esquinas;
 
  for(i=0;i<esquinas[0];i++)
    {
      if (esquinas[i*offset_corners+5]>0.9)
	{
	  // Conversion al stma. de referencia del robot" [0]:x [1]:y [2]:conc/conv [3]:R
	  mapa->esquinas[(indice+i)*offset_corners]  = esquinas[i*offset_corners+3];
	  mapa->esquinas[(indice+i)*offset_corners+1]= esquinas[i*offset_corners+2];
	  mapa->esquinas[(indice+i)*offset_corners+2]= esquinas[i*offset_corners+4];
		    
	  // -. 10 de Octubre .- esquinas[i*offset_corners+1]:indice
		  
	  r = points[3*(inicio + (int)esquinas[i*offset_corners+1])+1];
	  theta = points[3*(inicio + (int)esquinas[i*offset_corners+1])+2];
			  
	  mapa->esquinas[(indice+i)*offset_corners+3]=0;
	  
	  mapa->R_esquinas[(indice+i)*offset_Rp] = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + r*r*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
	  mapa->R_esquinas[(indice+i)*offset_Rp+1]=SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - r*r*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
	  mapa->R_esquinas[(indice+i)*offset_Rp+2]=SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + r*r*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);
	  cont++; 
	}
    }
  mapa->n_esquinas+=cont;  
}

//****************** Sigma ************************************
//
//   estimate of Sigma = square root of RSS divided by N

double Worker::Sigma (Data data, Circle circle)
{
	double sum = 0.,dx,dy,di;

	for (int i=0; i<data.n; i++)
	{
		dx = data.X[i] - circle.a;
		dy = data.Y[i] - circle.b;
		di = sqrt(dx*dx+dy*dy) - circle.r;
		sum += di*di;
	}
	return sqrt(sum/data.n);
}

////////////////////// Interface response

RoboCompCuba2Dnaturallandmarks::Features Worker::computeFeatures(const RoboCompLaser::TLaserData & lData)
{
	features(lData, &scan_mapa, &scan_maparef); 
	RoboCompCuba2Dnaturallandmarks::Features features;
	features.p.resize(scan_mapa.n_esquinas);
	for (int i = 0; i < scan_mapa.n_esquinas; i++) 
	{
		//qDebug() << scan_mapa.n_esquinas;
		features.p[i].x = scan_mapa.esquinas[i*offset_corners + _Y];
		features.p[i].z = scan_mapa.esquinas[i*offset_corners + _X];
		features.p[i].y = height;
	}
	features.s.resize(scan_mapa.n_segmentos);
	for (int i = 0; i < scan_mapa.n_segmentos; i++) 
	{
		features.s[i].p1z = scan_mapa.segmentos[i*offset_segments  + 1];
		features.s[i].p1x = scan_mapa.segmentos[i*offset_segments  + 2];
		features.s[i].p1y = height;
		features.s[i].p2z = scan_mapa.segmentos[i*offset_segments  + 3];
		features.s[i].p2x = scan_mapa.segmentos[i*offset_segments  + 4];
		features.s[i].p1y = height;
	}
	features.c.resize(scan_mapa.n_circulos);
	for (int i = 0; i < scan_mapa.n_circulos; i++) 
	{
		features.c[i].cz = scan_mapa.circulos[i*offset_circles + _X];
		features.c[i].cx = scan_mapa.circulos[i*offset_circles + _Y];		
		features.c[i].cy = height;
		features.c[i].radius = scan_mapa.circulos[i*offset_circles + _RADIUS];
	}
	return features;
}

RoboCompCuba2Dnaturallandmarks::Features Worker::getFeatures()
{
  
	int X_INI,Y_INI,X_FIN,Y_FIN;
	int ALPHA, R;
	
	RoboCompCuba2Dnaturallandmarks::Features features;
	QMat point(3);
	features.p.resize(scan_mapa.n_esquinas);
	for (int i = 0; i < scan_mapa.n_esquinas; i++) 
	{
	  
	  point(0) = scan_mapa.esquinas[i*offset_corners + _Y];
	  point(1) = height;
	  point(2) = scan_mapa.esquinas[i*offset_corners + _X];

//	  point = innerModel->robotToWorld( point );
	  
	  //qDebug() << scan_mapa.n_esquinas;
	  
	  features.p[i].x = point(0);
	  features.p[i].z = point(2);
	  features.p[i].y = point(1);
	  
	  //std::cout<<"[VC] sigma_x: "<<matriz[offset_Rp*cont+0]<<" sigma_y: "<<matriz[offset_Rp*cont+1]<<std::endl;
	  //std::cout<<"[VC] sigma_xy:"<<matriz[offset_Rp*cont+2]<<"
	  //std::cout<<"[VC] sigma_alfa: "<<matriz[offset_Rp*cont+3]<<std::endl;
	  //std::cout<<"[VC] sigma_xalfa: "<<matriz[offset_Rp*cont+4]<<std::endl;
	  //std::cout<<"[VC] sigma_yalfa: "<<matriz[offset_Rp*cont+5]<<std::endl;

	}
	
	// Comment: "Segmentos"
	
	X_INI = 1; Y_INI = 2; X_FIN = 3; Y_FIN = 4;
	ALPHA = 5; R = 6;
	
	features.s.resize(scan_mapa.n_segmentos);
	for (int i = 0; i < scan_mapa.n_segmentos; i++) 
	{
	  point(0) = scan_mapa.segmentos[i*offset_segments  + Y_INI];
	  point(1) = height;
	  point(2) = scan_mapa.segmentos[i*offset_segments  + X_INI];
	  
// 	  point = innerModel->robotToWorld( point );
	  	  
	  features.s[i].p1z = point(2);
	  features.s[i].p1x = point(0);
	  features.s[i].p1y = point(1);
		
	  point(0) = scan_mapa.segmentos[i*offset_segments  + Y_FIN];
	  point(1) = height;
	  point(2) = scan_mapa.segmentos[i*offset_segments  + X_FIN];
	  
// 	  point = innerModel->robotToWorld( point );

	  features.s[i].p2z = point(2);
	  features.s[i].p2x = point(0);
	  features.s[i].p2y = point(1);
		
	  features.s[i].d = scan_mapa.segmentos[i*offset_segments  + R];
	  features.s[i].alpha = scan_mapa.segmentos[i*offset_segments  + ALPHA];
	 
	  features.s[i].alpha += innerModel->getBaseAngle();
	  if (features.s[i].alpha <= -M_PI) features.s[i].alpha += 2*M_PI;
	  if (features.s[i].alpha >   M_PI) features.s[i].alpha -= 2*M_PI;

	
	  features.s[i].R.resize(4);
	  features.s[i].R[0] = scan_mapa.segmentos[i*offset_Rs];     // sigmaalfaalfa
	  features.s[i].R[1] = scan_mapa.segmentos[i*offset_Rs + 1]; // sigmaphopho
	  features.s[i].R[2] = scan_mapa.segmentos[i*offset_Rs + 2]; // sigmaalfapho
	  features.s[i].R[3] = scan_mapa.segmentos[i*offset_Rs + 3]; // value used for the lenght of the line segment (mm) 
	  		
	}
	
	features.c.resize(scan_mapa.n_circulos);
	for (int i = 0; i < scan_mapa.n_circulos; i++) 
	{
		features.c[i].cz = scan_mapa.circulos[i*offset_circles + _X];
		features.c[i].cx = scan_mapa.circulos[i*offset_circles + _Y];		
		features.c[i].cy = height;
		features.c[i].radius = scan_mapa.circulos[i*offset_circles + _RADIUS];
	}
	return features;

}

RoboCompCuba2Dnaturallandmarks::Features Worker::getLocalFeatures()
{
  
	int X_INI,Y_INI,X_FIN,Y_FIN;
	int ALPHA, R;
	int n_esquinas;
	
	RoboCompCuba2Dnaturallandmarks::Features features;
	QMat point(3);
	
	std::cout << "n_esquinas: " << scan_mapa.n_esquinas << std::endl;
	n_esquinas = scan_mapa.n_esquinas;
	features.p.resize(scan_mapa.n_esquinas);
	for (int i = 0; i < n_esquinas; i++) 
	{
	  
	  std::cout << "esquina: " << i << std::endl;
	  std::cout << "n_esquinas: " << scan_mapa.n_esquinas << std::endl;
	  point(0) = scan_mapa.esquinas[i*offset_corners + _Y];
	  point(1) = height;
	  point(2) = scan_mapa.esquinas[i*offset_corners + _X];

	  features.p[i].x = point(0);
	  features.p[i].z = point(2);
	  features.p[i].y = point(1);
	  
	  features.p[i].alpha = 0;
	
	  
	  features.p[i].R.resize(6);
	  features.p[i].R[0] = scan_mapa.R_esquinas[i*offset_Rp];      
	  features.p[i].R[1] = scan_mapa.R_esquinas[i*offset_Rp + 1]; 
	  features.p[i].R[2] = scan_mapa.R_esquinas[i*offset_Rp + 2]; 
	  features.p[i].R[3] = scan_mapa.R_esquinas[i*offset_Rp + 3];
	  features.p[i].R[4] = scan_mapa.R_esquinas[i*offset_Rp + 4]; 
	  features.p[i].R[5] = scan_mapa.R_esquinas[i*offset_Rp + 5]; 
	  
	  
	  std::cout << "----getLocal----" << std::endl;
	  std::cout << "corner: (x, y, alpha)" << features.p[i].x << ", " << features.p[i].z << ", " << features.p[i].alpha << std::endl;
	  std::cout << "corner: (cxx, cxy, cxalpha, cyy, cyalpha, calphaalpha)\n" << features.p[i].R[0] << ", " << features.p[i].R[2] << ", " << features.p[i].R[4] << features.p[i].R[1] << features.p[i].R[5] << features.p[i].R[3] << std::endl;
	  std::cout << "-----------" << std::endl;
	    
	}
	
	// Comment: "Segmentos"
	
	X_INI = 1; Y_INI = 2; X_FIN = 3; Y_FIN = 4;
	ALPHA = 5; R = 6;
	
	features.s.resize(scan_mapa.n_segmentos);
	for (int i = 0; i < scan_mapa.n_segmentos; i++) 
	{
	  point(0) = scan_mapa.segmentos[i*offset_segments  + Y_INI];
	  point(1) = height;
	  point(2) = scan_mapa.segmentos[i*offset_segments  + X_INI];
	  
  	  
	  features.s[i].p1z = point(2);
	  features.s[i].p1x = point(0);
	  features.s[i].p1y = point(1);
		
	  point(0) = scan_mapa.segmentos[i*offset_segments  + Y_FIN];
	  point(1) = height;
	  point(2) = scan_mapa.segmentos[i*offset_segments  + X_FIN];


	  features.s[i].p2z = point(2);
	  features.s[i].p2x = point(0);
	  features.s[i].p2y = point(1);
		
	  features.s[i].d = scan_mapa.segmentos[i*offset_segments  + R];
	  features.s[i].alpha = scan_mapa.segmentos[i*offset_segments  + ALPHA];
	
	  features.s[i].R.resize(4);
	  features.s[i].R[0] = scan_mapa.R_segmentos[i*offset_Rs];     // sigmaalfaalfa
	  features.s[i].R[1] = scan_mapa.R_segmentos[i*offset_Rs + 1]; // sigmaphopho
	  features.s[i].R[2] = scan_mapa.R_segmentos[i*offset_Rs + 2]; // sigmaalfapho
	  features.s[i].R[3] = scan_mapa.R_segmentos[i*offset_Rs + 3]; // value used for the lenght of the line segment (mm) 
	  
// 	  std::cout << "----getLocal----" << std::endl;
// 	  std::cout << "segmento: (alfa, d, length)" << features.s[i].alpha *180/M_PI << ", " << features.s[i].d << ", " << features.s[i].R[3] << std::endl;
// 	  std::cout << "segmento: (calfa, crho, calfarho)" << features.s[i].R[0] << ", " << features.s[i].R[1] << ", " << features.s[i].R[2] << std::endl;
// 	  std::cout << "det2R: " << features.s[i].R[0]*features.s[i].R[0] - features.s[i].R[2]*features.s[i].R[2]  << std::endl;
// 	  std::cout << "-----------" << std::endl;

	  		
	}
	
	features.c.resize(scan_mapa.n_circulos);
	for (int i = 0; i < scan_mapa.n_circulos; i++) 
	{
		features.c[i].cz = scan_mapa.circulos[i*offset_circles + _X];
		features.c[i].cx = scan_mapa.circulos[i*offset_circles + _Y];		
		features.c[i].cy = height;
		features.c[i].radius = scan_mapa.circulos[i*offset_circles + _RADIUS];
	}
	

	return features;

}
