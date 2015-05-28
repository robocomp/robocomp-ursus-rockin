#include "target.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto
 */
Target::Target()
{
	this->state = State::IDLE;
	this->bodyPart = "";
	this->pose.x = this->pose.y = this->pose.z = this->pose.rx = this->pose.ry = this->pose.rz = 0;
	this->weights.x = this->weights.y = this->weights.z = this->weights.rx =this->weights.ry = this->weights.rz = 0;
}

 /**
 * \brief Constructor parametrizado
 * Inicializa las estructuras que componen sus atributos de clase. 
 * @param bodyPart_ nombre de la parte del robot a la que pertenece el target.
 * @param pose_ pose de tres traslaciones y tres rotaciones que define las coordenadas y orientacion del target [X, Y, Z, RX, RY, RZ]
 * @param weights_ pesos de las traslaciones y las rotaciones
 */ 
Target::Target(const string bodyPart_, const RoboCompBodyInverseKinematics::Pose6D &pose_, const RoboCompBodyInverseKinematics::WeightVector &weights_)
{
	this->state = State::IDLE;
	this->bodyPart = bodyPart_;
	this->pose = pose_;
	this->weights = weights_;
}

/**
 * \brief Destructor por defecto
 */ 
Target::~Target()
{
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS PUT												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
void Target::setPose(QVec correctTargetV)
{
	this->pose.x = correctTargetV.x();
	this->pose.y = correctTargetV.y();
	this->pose.z = correctTargetV.z();
	this->pose.rx = correctTargetV.rx();
	this->pose.ry = correctTargetV.ry();
	this->pose.rz = correctTargetV.rz();
}

/**
 * \brief Metodo CHANGE STATE:
 * Cambia el valor del atributo STATE por el valor del parametro de entrada.
 * @param state_ nuevo estado
 */ 
void Target::changeState(Target::State state_)		{	this->state = state_; }

/**
 * \brief Metodo CHANGE BODY PART:
 * Cambia el valor del atributo BODY PART por el valor del parametro de entrada.
 * @param bodyPart_ nombre de la parte del robot.
 */ 
void Target::changeBodyPart(string bodyPart_)		{	this->bodyPart = bodyPart_; }

/**
 * \brief Metodo CHANGE POSE 6D:
 * Cambia el valor del atributo POSE 6D por el valor del parametro de entrada.
 * @param pose_ nuevas coordenadas y orientación del target.
 */ 
void Target::changePose6D(Pose6D pose_)			{	this->pose = pose_;	}

/**
 * \brief Metodo CHANGE WEIGHTS:
 * Cambia el valor del atributo WEIGHTS por el valor del parametro de entrada.
 * @param weights_ nuevo vector de pesos para las coordenadas de traslacion y de orientacion
 */ 
void Target::changeWeights(WeightVector weights_)	{	this->weights = weights_; }

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Metodo GET STATE
 * Devuelve el valor del atributo STATE: IDLE, IN PROCESS, RESOLVED
 * @return Target::State valor del state
 */ 
Target::State Target::getState()					{ return this->state; 	}

/**
 * \brief Metodo GET BODY PART
 * Devuelve el nombre de la parte del robot a la que va destinado el target.
 * @return string nombre de la parte del robot.
 */ 
string Target::getBodyPart()						{ return this->bodyPart; }

/**
 * \brief Metodo GET POSE 6D
 * Devuelve las coordenadas de traslacion y orientacion del target.
 * @return Pose6D.
 */
Pose6D Target::getPose6D()							{ return this->pose;}

/**
 * \brief Metodo GET WEIGHTS
 * Devuelve el vector de pesos de las coordenadas de traslacion y orientacion 
 * del target.
 * @return weights
 */ 
WeightVector Target::getWeights()					{return this->weights; }









