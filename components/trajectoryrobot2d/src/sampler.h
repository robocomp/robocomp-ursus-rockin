/*
 * Copyright 2014 <copyright holder> <email>
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

#ifndef SAMPLER_H
#define SAMPLER_H

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <QtCore>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "qline2d.h"
#include <assert.h>
#include <stdio.h>    

class Sampler
{
	public:
		Sampler();
		void initialize(InnerModel* inner, const QRectF& outerRegion_, const QList< QRectF >& innerRegions_);
		std::tuple< bool, QString > checkRobotValidStateAtTarget(const QVec& targetPos, const QVec& targetRot) const;
		QList<QVec> sampleFreeSpaceR2(uint nPoints = 1);
		QList<QVec> sampleFreeSpaceR2Uniform(const QRectF &box, uint32_t i=1);
		QList<QVec> sampleFreeSpaceR2Gaussian(float meanX, float meanY, float sigma1, float sigma2, uint32_t nPoints = 1);
		bool isStateValid(const ompl::base::State *state) ;
		bool checkRobotValidDirectionToTarget(const QVec & origin , const QVec & target, QVec &path);
		bool checkRobotValidDirectionToTargetBinarySearch(const QVec & origin , const QVec & target, QVec &lastPoint) const;
		bool checkRobotValidDirectionToTargetOneShot(const QVec & origin , const QVec & target) const;
		bool searchRobotValidStateCloseToTarget(QVec &target);
		QRectF getOuterRegion() const { return outerRegion;};
		mutable QMutex mutex;
		
	private:
		std::vector<QString> robotNodes;
		std::vector<QString> restNodes;
		std::set<QString> excludedNodes;
		QList<QRectF> innerRegions;
		QRectF outerRegion;
		InnerModel *innerModelSampler;
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded);
		
};

#endif // SAMPLER_H
