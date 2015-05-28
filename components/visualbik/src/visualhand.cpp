#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto. 
 */ 
VisualHand::VisualHand(InnerModel *im_, QString tip_)
{
	this->im = im_;
	this->tip = tip_;
	this->lastUpdate = new timeval;
	gettimeofday(this->lastUpdate, NULL);

	InnerModelNode *nodeParent = this->im->getNode("rgbd_transform");
	if( this->im->getNode("marca-" + this->tip + "-segun-head") == NULL)
	{
		nodeMarca = this->im->newTransform("marca-" + this->tip + "-segun-head", "static", nodeParent, 0, 0, 0,        0, 0., 0,      0.);
		nodeParent->addChild(nodeMarca);
		nodeMarca2 = this->im->newTransform("marca-" + this->tip + "-segun-head2", "static", nodeMarca, 0, 0, 0,       0., 0, 0,      0.);
		nodeMarca->addChild(nodeMarca2);
	}
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
 * lastUpdate. PASA DE RGBD A ROOT!!!!!!!!!
 * @param tag marca de apriltags con la posicion de la mano.
 */ 
void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
{
	QVec tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	
	// Metemos en el InnerModel la marca vista por la RGBD:	
	this->im->updateTransformValues("marca-" + this->tip + "-segun-head", tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	this->im->updateTransformValues("marca-" + this->tip + "-segun-head2", 0,0,0,   -M_PI_2,0,M_PI);
	
	//Pasamos del marca-segun-head al mundo:
	QVec ret = this->im->transform("root", tagPose, "rgbd");
	QVec ret2 = this->im->transform("root", tagPose, "marca-" + this->tip + "-segun-head2");
	this->visualPose.x = ret(0);
	this->visualPose.y = ret(1);
	this->visualPose.z = ret(2);
	
	this->visualPose.rx = ret2(3);
	this->visualPose.ry = ret2(4);
	this->visualPose.rz = ret2(5);
	

	gettimeofday(this->lastUpdate, NULL);

}


void VisualHand::setVisualPose(const RoboCompBodyInverseKinematics::Pose6D& pose)
{
	this->visualPose = pose;
}


/**
 * \brief Metodo SET INTERNAL POSE
 * Almacena en el atributo internalPose las coordenadas de traslacion y de rotacion
 * de la mano en la que cree el robot que esta.
 * @param pose es la pose6D de donde cree el robot que tiene el
 */ 
void VisualHand::setInternalPoseFromInnerModel(QString hand)
{
	QVec pose = this->im->transform("root", QVec::vec6(0,0,0, 0,0,0), hand);

	this->internalPose.x = pose.x();
	this->internalPose.y = pose.y();
	this->internalPose.z = pose.z();
	
	this->internalPose.rx = pose.rx();
	this->internalPose.ry = pose.ry();
	this->internalPose.rz = pose.rz();
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

	static bool first = true;
	if (first)
	{
		first = false;
		return 0;
	}

	const double secs  = currentTimeval.tv_sec  - lastUpdate->tv_sec;
	const double usecs = currentTimeval.tv_usec - lastUpdate->tv_usec;
	return secs + usecs/1000000;
}

/**
 * \brief Metodo GET ERROR
 * Calcula el error que existe entre la pose de la mano que la camara esta viendo
 * y la posicion del target al que quiere ir.
 * TODO ARREGLAR EL ERROR ENTRE ANGULOS
 * @param target target al que quiere ir.
 * @return pose6D error
 */ 
QVec VisualHand::getError(RoboCompBodyInverseKinematics::Pose6D target)
{
	const QVec error = this->im->transform6D("target", "visual_hand");
	return error;

// 	error.x = this->visualPose.x - target.x;
// 	error.y = this->visualPose.y - target.y;
// 	error.z = this->visualPose.z - target.z;
// 	
// 	printf("Visual %f %f %f    [%f %f %f]\n", visualPose.x, visualPose.y, visualPose.z, visualPose.rx, visualPose.ry, visualPose.rz);
// 	printf("Target %f %f %f    [%f %f %f]\n", target.x, target.y, target.z, target.rx, target.ry, target.rz);
// 	
// 	Rot3D visualR(visualPose.rx, visualPose.ry, visualPose.rz);
// 	Rot3D targetR(target.rx, target.ry, target.rz);
// 	QVec errorRR = (visualR.invert()*targetR).extractAnglesR_min();
// 	errorRR.print("errorrrrrrr acho");
// 
// 	error.rx = errorRR.rx();
// 	error.ry = errorRR.ry();
// 	error.rz = errorRR.rz();
// 	
}

QVec VisualHand::getErrorInverse(RoboCompBodyInverseKinematics::Pose6D target)
{
	const QVec error = this->im->transform6D("visual_hand", "target");
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
	return this->visualPose;
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

