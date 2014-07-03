    
    
    
#include "Trajectory.h"
#include <stdio.h>    

    

using namespace std; 


/* -------------------------------------------------------------------
 *                 Métodos CONSTRUCTORES 
 *--------------------------------------------------------------------*/
/**
 * \brief Default constructor
 */ 
Trajectory::Trajectory(QList<QVec> _road, RCDrawRobot *_rc, RoboCompDifferentialRobot::TBaseState &_bState,  InnerModel *_innerModel)
{
  try{
    
    // Iniciamos nuestras variables a 0.
    float x, y, numPoints;
    FILE *fichero;
	char *cadena;
    
    fichero = fopen("/home/salabeta/robocomp/Components/RoboLab/Experimental/localizadorComp/puntos.txt", "r");
    fgets(cadena, 1500, fichero);
	if (!fichero) { cout<< "ERROR AL LEER FICHERO. SALIENDO "<< endl; exit(-1);}
    
    /* RECALCULAR PUNTOS INTERMEDIOS EN EL FOR DE LA LECTURA DE FICHEROS. 
	 
	 UNA VEZ RECALCULADOS, A TRAVÉS DE LAMDA (QUE VA DE 0 A 1, RECORDAR PROPORCION) OBTENER 
	 DISTANCIA/400 PUNTOS DIFERENTES EN ESE SEGMENTO. UNA VEZ LO TENGAMOS, COGEMOS EL SIGUIENTE PUNTO Y LO USAREMOS PARA REPETIR EL PROCESO*/
    //Leemos la primera línea del fichero: un mensaje
	fscanf(fichero, "%f", &numPoints);
	for (int i = 0; i < numPoints; i++){
		fscanf(fichero, "%f %f", &x, &y);  
		cout<<" RECOGIDO: "<< numPoints << " " << x << " " << y<<endl;
  }
    
    tramoActivo = 0;
    data.angle = 0;
    data.d = 0;
    data.D = 0;
	
    rc = _rc;
    //road = _road;
    bState = &_bState;
    innerModel = _innerModel;
	
    QVec P1 = Qvec{};
    QVec P2 = road[1];
	
    Seg segAux;
    segAux.punto = P1;
    segAux.A = - (P2.z() - P1.z());
    segAux.B = (P2.x() - P1.x());
    segAux.C = (-segAux.A*P1.x()) - segAux.B*P1.z();
    segAux.norm = (P2-P1).norm2();
    segAux.activo = false;
      
    segmentos.append(segAux);
	
    for(int i = 1; i < road.size(); i++){
      
      if(i+1 == road.size()) break;
		    
      P1 = road[i];
      P2 = road[i+1];
      
      segAux.punto = P1;
      segAux.A = - (P2.z() - P1.z()); // ES NEGATIVO
      segAux.B = (P2.x() - P1.x());
      segAux.C = (-segAux.A*P1.x()) - segAux.B*P1.z();
      segAux.norm = (P2-P1).norm2();
      segAux.activo = false;
	    
      segmentos.append(segAux);
    }
  }catch(const Ice::Exception &e){cout << "Error en Constructor Trajectory " << e << endl;}
}	
    
/**
* \brief Default destructor
*/
Trajectory::~Trajectory()
{
  road.clear();      // eliminamos los elementos de la carretera
  segmentos.clear(); // eliminamos los elementos de los segmentos
}


/************************************************************************
 *                  MÉTODOS GET
 ************************************************************************/ 
/*
 * Método getDistToSeg.                                                         
 * Devuelve la distancia del robot al segmento de la trayectoria por el        
 * que va moviéndose (la d pequeña)                                         
 */ 
float Trajectory::getDistToSeg()
{
   calcularDistToSeg();
   return data.d;
}

/*
 * Método getDistToEnd
 * Devuelve la distancia del robot al punto final de la trayectoria
 * (la D mayúscula).
 */ 
float Trajectory::getDistToEnd()
{
	  calcularDistToEnd();
      return data.D;
}

/*
 * Método getAngle.
 * Devuelve el ángulo que forma el robot con el segmento activo.
 */ 
float Trajectory::getAngle()
{
  calcularAngle();
  return data.angle;
}


/*********************************************************************
 *          MÉTODOS SET/CALCULAR
 *********************************************************************/
/*
 * Método calcularDistToSeg
 * Calcula la distancia del robot al segmento activo y la
 * almacena en data->d
 */ 
void Trajectory::calcularDistToSeg()
{
  float raiz = (QVec::vec2(segmentos[tramoActivo].A, segmentos[tramoActivo].B)).norm2();
  if (raiz < 0.000001) qFatal("LA HEMOS LIAO PARDA");    
  data.d = (segmentos[tramoActivo].A*bState->x + segmentos[tramoActivo].B*bState->z + segmentos[tramoActivo].C)/ sqrt(pow(segmentos[tramoActivo].A, 2) + pow(segmentos[tramoActivo].B, 2));
	   
  cout<< "Distancia a la recta: " << data.d << " [A " << segmentos[tramoActivo].A << "; B " << segmentos[tramoActivo].B << "; C" << segmentos[tramoActivo].C << "]"<< endl;
	  
}
/*
 * Método calcularDistToEnd.
 * Calcula la distancia del robot al punto final de la trayectoria
 * y la almacena en data.D
 */ 
void Trajectory::calcularDistToEnd()
{
  QVec PuntoDestino;
  int i = 0;
  bool encontrado = false;
  
  while ((i<segmentos.size()-1) && !encontrado){
    if (segmentos[i].activo == false)
      encontrado = true;
    else
      i++;
  }
  
  if(!encontrado)
    PuntoDestino = road.last();  
  else
    PuntoDestino = segmentos[i+1].punto;

  data.D = (PuntoDestino - QVec::vec3(bState->x,0, bState->z)).norm2();
  for (int j = i+1; j < segmentos.size(); j++)
    data.D = data.D + segmentos[j].norm;
  
  cout << "Distancia al final : " << data.D<< endl;
}
    
/* 
 * Método calcularAngle.
 * Calcula el ángulo que forma el robot con el segmento activo.
 * Lo guarda en data.angle.
 */ 
void Trajectory::calcularAngle()
{
  //Calculamos el ángulo que forma la recta actual con el eje Z del mundo real.
  QVec directSeg = QVec::vec3(- segmentos[tramoActivo].B, 0, segmentos[tramoActivo].A);
  
  //Calculamos el ángulo del  robot con el eje Z (bState.alpha) del mundo real
  QVec directRobot = QVec::vec3(- segRobot.B, 0, segRobot.A);
  
  float angle = acos( (directSeg*directRobot) / ( (directSeg).norm2() * (directRobot).norm2() ));
  
  QVec giro = directSeg^directRobot; // calculamos el sentido del giro.
  if(giro.y() < 0)                   // para girar por el camino más corto.
    angle = - angle; 
  
  data.angle = angle;
  cout<<"Ángulo del segmento activo: "<<angle<<endl;

}

/**********************************************************************
 *        OTROS MÉTODOS
 **********************************************************************/
/*
 * Método para saber qué tramo de la trayectoria está activo.
 * Partimos de la idea que NO es una trajectoria que se corta.
 */
bool Trajectory::transito()
{
  static bool enposicion = false;
  static bool anterior = false;
  bool siguiente = false;
  
  if ((!enposicion)){
    
    buscarTramo();
    enposicion = true;
    calcularRectas();
    anterior = calcularPosicion();
  }
  
    calcularRectas();
    siguiente = calcularPosicion();
    
    if(siguiente != anterior)
    {
      enposicion = false;
      segmentos[tramoActivo].activo = true;
    }
    
    if(segmentos.last().activo == true)
      data.D = 0;
    
  return enposicion;
}

/*
 * Método buscarTramo.
 * Busca el siguiente tramo activo.
 */
void Trajectory::buscarTramo()
{ 
  int i = 0;
  bool encontrado = false;
  
  while ((i<segmentos.size()-1) && !encontrado){
    
		if (segmentos[i].activo == false)
			encontrado = true;
	else
	  i++;
  }
  
  if(!encontrado){
	P = road.last();  
	tramoActivo = segmentos.size()-1;
  }
  else{
	P = segmentos[i+1].punto;
	tramoActivo = i;
  }
}

/*
 * Método calcularRectas.
 * Calcula la recta dirección del robot y la recta perpendicular a ella
 * que pasa por el punto objetivo (el del final del tramo)
 */ 
void Trajectory::calcularRectas()
{
  QVec puntoOrigen = QVec::vec3(bState->x,0, bState->z);
  QVec puntoDes = innerModel->transform("world", QVec::vec3(0,0,1000), "base"); 
   
  segRobot.A = - (puntoDes.z() - puntoOrigen.z());
  segRobot.B = (puntoDes.x() - puntoOrigen.x());
  segRobot.C = (-segRobot.A*puntoOrigen.x()) - segRobot.B*puntoOrigen.z();
 
  
  segPerp = {0,-segRobot.B,segRobot.A,(segRobot.B*P.x() - segRobot.A*P.z()),0,false}; //Recta perpendicular.
  
  
  pintarRoad(puntoDes, P);
  /*calcularAngle();
  calcularDistToSeg();
  calcularDistToEnd();*/
}

bool Trajectory::calcularPosicion()
{
 if ((bState->x*segPerp.A + bState->z*segPerp.B + segPerp.C) >= 0)
   return true;
 else
   return false;
  
}



/************************************************************
 *     MÉTODOS PARA MOSTRAR INFORMACIÓN
 ************************************************************/ 
/*
 * Método mostrarSegmentos.
 * Muestra por el terminal el contenido de los segmentos.
 */ 
void Trajectory::mostrarSegmentos()
{
  for(int i = 0; i < segmentos.size(); i++)
    cout << "Segmento: " << i << "; A: " << segmentos[i].A << "; B= " << segmentos[i].B << "; C: " << segmentos[i].C << "; norma: " << segmentos[i].norm <<endl;

}

/*
 * Método pintarRoad.
 * Pinta sobre el simulador la trayectoria.
 */ 
void Trajectory::pintarRoad(QVec puntoDes, QVec P)
{
  QVec norm = QVec::vec3(-segPerp.B, 0, segPerp.A).normalize(); 
  //qDebug()<< "Vector Director Perp "<< segPerp.B << segPerp.A;
  //norm.print("norm");
  
    float k = P.x()/norm.x();
    float k2 = P.z()/norm.z();
    cout<< "Vector de Booleanos : [";
    for (int i = 0; i < segmentos.size(); i++)
      cout<<segmentos[i].activo<< " ";
    cout << "]"<< endl;
      
  
   try{
   rc->drawLine(QLineF(bState->x,bState->z,puntoDes.x(), puntoDes.z()  ),  Qt::red, 50 );
   
   rc->drawLine(QLineF(bState->x, bState->z, road[tramoActivo+1].x(), road[tramoActivo+1].z()), Qt::cyan, 50);

   rc->drawLine(QLineF(bState->x,bState->z, segRobot.B*1000, -segRobot.A*1000),  Qt::black, 50 );
    
   rc->drawLine(QLineF(norm.x()*(k-500),norm.z()*(k2-500), norm.x()*(k+500),norm.z()*(k2+500)),  Qt::magenta, 50 );

   rc->drawSquare(QPointF(P.x(), P.z()), 80, 80, Qt::blue, false, -1, 0, 50);  /* Pintamos el punto objetivo */
   
    QVec punto;
    QVector<QPoint> puntos_virtual;
    for (int i=0; i <road.size(); i++){
     /*
      * Sacamos las coordenadas cartesianas de las polares.
      */   
     punto = road[i];
     puntos_virtual.append(QPoint(punto.x(),punto.z()));
    }
 
    rc->drawPolyLine(puntos_virtual,Qt::darkYellow, 50);
   
   }catch(const Ice::Exception &e){cout << "Error pintando perpendicular " << e << endl;}
  
}
