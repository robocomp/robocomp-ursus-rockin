
#include <BodyInverseKinematics.h>
#include <AprilTags.h>
#include <sys/time.h>

#include <innermodel/innermodel.h>

/**
 * \class VisualHand
 *
 * \brief Stores the necessary information and provides some helper methods to work with the visal feedback of the position of the robot's hand
 *
 **/
class VisualHand
{
public:
	/**
	* \brief The VisualHand constructor must receive a pointer to an InnerModel object and the name of the arm's tip.
	*/
	VisualHand(InnerModel *im_, QString tip_);

	/**
	 * \brief Destructor.
	 */
	~VisualHand();

	/**
	* \brief Updates the hand's possition according to an April tag and the time.
	*/
	void setVisualPose(RoboCompAprilTags::tag tag);

	/**
	* \brief Updates the hand's possition according to direct kinematics.
	*/
	void setVisualPose(const RoboCompBodyInverseKinematics::Pose6D &pose);


	/**
	* \brief Metodo SECONDS ELAPSED
	* Devuelve los segundos que han pasado desde que se actualizo la pose visual por ultima vez.
	* @return double segundos
	*/
	double secondsElapsed();

	/**
	* \brief Computes the error from the visual position to a target position
	* @param target the target
	* @return pose6D error
	*/
 	QVec getError(RoboCompBodyInverseKinematics::Pose6D target);

	/**
	* \brief Computes the inverse of the error from the visual position to a target position
	* @param target the target
	* @return pose6D error
	*/
	QVec getErrorInverse(RoboCompBodyInverseKinematics::Pose6D target);


	/**
	* \brief Metodo GET VISUAL POSE
	* Devuelve las coordenadas de traslacion y de orientacion de la marca vista
	* por la camara del robot (¿hay que poner algun elapsed time?)
	* @return pose6D
	*/
	RoboCompBodyInverseKinematics::Pose6D getVisualPose();


	RoboCompBodyInverseKinematics::Pose6D getInternalPose();


	/**
	 * \brief returns the name of the hand's tip.
	 * @return QString tip
	 */ 
	QString getTip() { return tip; }

private:
	RoboCompBodyInverseKinematics::Pose6D visualPose;
	timeval *lastUpdate;
	InnerModel *im;
	QString tip;

	InnerModelTransform *nodeMarca, *nodeMarca2;
};