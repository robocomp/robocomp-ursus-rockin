#include <visualhand.h>

#include <qmat/qrtmat.h>
#include <time.h>

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

VisualHand::~VisualHand()
{
	delete this->lastUpdate;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 												METODOS PUT/SET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void VisualHand::setVisualPose(RoboCompAprilTags::tag tag)
{
	QVec tagPose = QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);

	// Metemos en el InnerModel la marca vista por la RGBD:
	this->im->updateTransformValues("marca-" + this->tip + "-segun-head", tag.tx, tag.ty, tag.tz,   tag.rx, tag.ry, tag.rz);
	this->im->updateTransformValues("marca-" + this->tip + "-segun-head2", 0,0,0,   -M_PI_2,0,M_PI);

	//Pasamos del marca-segun-head al mundo:
	QVec ret = this->im->transform6D("root", tagPose, "rgbd");
	this->visualPose.x = ret(0);
	this->visualPose.y = ret(1);
	this->visualPose.z = ret(2);

	QVec ret2 = this->im->transform("root", tagPose, "marca-" + this->tip + "-segun-head2");
	this->visualPose.rx = ret2(3);
	this->visualPose.ry = ret2(4);
	this->visualPose.rz = ret2(5);

	gettimeofday(this->lastUpdate, NULL);
	
	//Calculo el error entre pose interna y pose visual.
	
	
}

void VisualHand::setVisualPose(const RoboCompBodyInverseKinematics::Pose6D& pose)
{
	this->visualPose = pose;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 													METODOS GET												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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

QVec VisualHand::getError(RoboCompBodyInverseKinematics::Pose6D target)
{
	const QVec error = this->im->transform6D("target", "visual_hand");
	return error;
}

QVec VisualHand::getErrorInverse(RoboCompBodyInverseKinematics::Pose6D target)
{
	const QVec error = this->im->transform6D("visual_hand", "target");
	return error;
}

RoboCompBodyInverseKinematics::Pose6D VisualHand::getVisualPose()
{
	return this->visualPose;
}

RoboCompBodyInverseKinematics::Pose6D VisualHand::getInternalPose()
{
	QVec pose = this->im->transform6D("root", this->tip);

	RoboCompBodyInverseKinematics::Pose6D internalPose;

	internalPose.x = pose.x();
	internalPose.y = pose.y();
	internalPose.z = pose.z();
	internalPose.rx = pose.rx();
	internalPose.ry = pose.ry();
	internalPose.rz = pose.rz();

	return internalPose;
}
