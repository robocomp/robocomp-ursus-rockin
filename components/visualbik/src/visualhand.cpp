#include <visualhand.h>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto. 
 */ 
VisualHand::VisualHand()
{
	this->x = this->y = this->z = this->rx = this->ry = this->rz = 0;
	this->lastUpdate = new timeval;
}

/**
 * \brief Destructor por defecto
 */ 
VisualHand::~VisualHand()
{

}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 												METODOS PUT/SET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * \brief Metodo SET VISUAL POSE
 * Almacena en sus atributos [X, Y, Z, RX, RY, RZ] las coordenadas de traslacion y rotacion
 * de la marcda de apriltags que recibe como parametro de entrada. Actualiza el tiempo de
 * lastUpdate.
 * @param tag marca de apriltags con la posicion de la mano.
 */ 
void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
{
	this->x = tag.tx;
	this->y = tag.ty;
	this->z = tag.tz;
	this->rx = tag.rx;
	this->ry = tag.ry;
	this->rz = tag.rz;

	gettimeofday(this->lastUpdate, NULL);
}

/**
 * \brief Metodo SET INTERNAL POSE
 * Almacena en el atributo internalPose las coordenadas de traslacion y de rotacion
 * de la mano en la que cree el robot que esta.
 * @param pose es la pose6D de donde cree el robot que tiene el
 */ 
void VisualHand::setInternalPose(RoboCompBodyInverseKinematics::Pose6D pose)
{
	this->internalPose.x = pose.x;
	this->internalPose.y = pose.y;
	this->internalPose.z = pose.z;
	
	this->internalPose.rx = pose.rx;
	this->internalPose.ry = pose.ry;
	this->internalPose.rz = pose.rz;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo SECONDS ELAPSED
 * Devuelve los segundos que han pasado desde que se actualizo la pose visual por ultima vez.
 * @return double segundos
 */ 
double VisualHand::secondsElapsed()
{
	static timeval currentTimeval;
	gettimeofday(&currentTimeval, NULL);
	
	const double secs  = currentTimeval.tv_sec  - lastUpdate->tv_sec;
	const double usecs = currentTimeval.tv_usec - lastUpdate->tv_usec;
	return secs + usecs/1000000;
}

/**
 * \brief Metodo GET ERROR
 * Calcula el error que existe entre la pose en la que el robot cree que 
 * tiene la mano colocada y la posicion de la mano que la camara esta viendo
 * TODO ARREGLAR EL ERROR ENTRE ANGULOS
 * @return pose6D error
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getError()
{
	RoboCompBodyInverseKinematics::Pose6D error;
	
	error.x = this->internalPose.x - this->x;
	error.y = this->internalPose.y - this->y;
	error.z = this->internalPose.z - this->z;
	
	error.rx = this->internalPose.rx - this->rx;
	error.ry = this->internalPose.ry - this->ry;
	error.rz = this->internalPose.rz - this->rz;
	
	return error;
}

/**
 * \brief Metodo GET VISUAL POSE
 * Devuelve las coordenadas de traslacion y de orientacion de la marca vista
 * por la camara del robot (¿hay que poner algun elapsed time?)
 * @return pose6D
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getVisualPose()
{
	RoboCompBodyInverseKinematics::Pose6D poseVisual;
	poseVisual.x = this->x;
	poseVisual.y = this->y;
	poseVisual.z = this->z;
	
	poseVisual.rx = this->rx;
	poseVisual.ry = this->ry;
	poseVisual.rz = this->rz;
	
	return poseVisual;
}

/**
 * \brief Metodo GET INTERNAL POSE
 * Devuelve las coordenadas de traslacion y de rotacion de la marca
 * que el robot cree tener en cierta posicion.
 * @return pose6d
 */ 
RoboCompBodyInverseKinematics::Pose6D VisualHand::getInternalPose()
{
	return this->internalPose;
}

