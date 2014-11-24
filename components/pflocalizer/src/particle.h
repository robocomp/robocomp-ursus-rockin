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

#ifndef PARTICLE_H
#define PARTICLE_H

#include <particleFiltering/particleFilter.h>
#include <QMutex>
#include <qmat/QMatAll>
#include <biasedRandomSelector.h>
#include <omp.h>

struct RCPFControl
{
	
};

struct RCPFInputData
{
	
};
struct RCPFConfig
{
	
};


class Particle : public RCParticleFilter_Particle<RCPFInputData, RCPFControl, RCPFConfig> 
{
	public:
		void computeWeight(const RCPFInputData& data);
		virtual void adapt(const RCPFControl& controlBack, const RCPFControl& controlNew, const bool noValidCandidates);
		virtual void initialize(const RCPFInputData& data, const RCPFControl& control, const RCPFConfig* cfg);
};

#endif // PARTICLE_H
