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

class Sampler
{
	public:
		Sampler();
		void initialize(InnerModel *inner, const QRectF& outerRegion_, const QList< QRectF >& innerRegions_);
		QVec sampleFreeSpaceR2();
		bool checkRobotValidStateAtTarget(const QVec &targetPos, const QVec &targetRot = QVec::zeros(3)) ;
		bool isStateValid(const ompl::base::State *state) ;
		bool checkRobotValidDirectionToTarget(const QVec & origin , const QVec & target, QVec &path);
		bool checkRobotValidDirectionToTargetBinarySearch(const QVec & origin , const QVec & target, QVec &lastPoint);
	private:
		std::vector<QString> robotNodes;
		std::vector<QString> restNodes;
		QList<QRectF> innerRegions;
		QRectF outerRegion;
		InnerModel *innerModel;
		
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
		
};

#endif // SAMPLER_H
