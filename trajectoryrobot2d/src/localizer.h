/*
 * Copyright 2014 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <Laser.h>

#include <fcl/collision.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef boost::shared_ptr<FCLModel> FCLModelPtr;

#include <pointmatcher/PointMatcher.h>

class Localizer : public QObject
{
    Q_OBJECT

	public:
		Localizer(InnerModel *inner);
		void localize(const RoboCompLaser::TLaserData &laser, InnerModel *inner, int nLaserRays=16);
		
	private:
		InnerModel *clonModel;
		RoboCompLaser::TLaserData virtualLaser;
		void laserRender(const QVec &point, float alfa, const QVec &angleList);
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
		void estimatePoseWithICP(const RoboCompLaser::TLaserData& laserData, const QVec& robotT, float ang);
		std::vector<QString> restNodes;
		std::vector<QString> robotNodes;
		
		//ICP
		typedef PointMatcher<float> PM;
		typedef PM::DataPoints DP;
		typedef PM::Parameters Parameters;
		Eigen::MatrixXf refM;
		Eigen::MatrixXf dataM;
		PM::ICP icp;
};
#endif // LOCALIZER_H
