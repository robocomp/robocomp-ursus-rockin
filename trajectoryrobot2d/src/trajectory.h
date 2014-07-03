/*
 * Clase Trajectory
 *
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <rcdraw/rcdraw.h>
#include <rcdraw/rcdrawrobot.h>
#include <innermodel/innermodel.h>
#include <genericworker.h>
#include <math.h>

typedef struct Seg {
  QVec punto;
  float A;
  float B;
  float C;
  float norm;
  bool activo;
} Seg;


typedef struct Datos{
  float d;
  float D;
  float angle;
} Datos;

class Trajectory 
{

  public:
    /* Métodos CONSTRUCTORES */
  Trajectory(QList<QVec> _road, RCDrawRobot *_rc, RoboCompDifferentialRobot::TBaseState &_bState,  InnerModel *_innerModel);  
  ~Trajectory();
      
 
    /* Métodos SET */
    void calcularDistToSeg();
    void calcularDistToEnd();
    void calcularAngle();
    
    /* Métodos GET */
    float getDistToSeg();
    float getDistToEnd();
    float getAngle();    

    bool transito();

  
  
  void mostrarSegmentos();
  void pintarRoad(QVec puntoDes, QVec P);
    
    
  private:
    QList<QVec> road;  
    QList<Seg> segmentos;
    Datos data;
    RCDrawRobot *rc;
    int tramoActivo;
    QVec P;
    Seg segPerp, segRobot;
    RoboCompDifferentialRobot::TBaseState *bState;
    InnerModel *innerModel;
    
    void buscarTramo();
    void calcularRectas();
    bool calcularPosicion();

  
  protected:
  
};
#endif