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

#include "plannerompl.h"



PlannerOMPL::PlannerOMPL(const InnerModel &innerModel_, QObject *parent)
{
// 	xMin = 0.;
// 	xMax = 10000.;
// 	zMin = -10000.;
// 	zMax = 0.;
// 	
// 	innerModel = new InnerModel(innerModel_);
// 	
// 	//Create state space as R2
// 	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
// 	space->addDimension(xMin, xMax);
// 	space->addDimension(zMin, zMax);
// 	
// 	//Setup class
// 	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
// 	
// 	//set Sampler
// 	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
// 	
// 	// set state validity checking for this space
// 	//simpleSetUp->setStateValidityChecker(boost::bind(&PlannerOMPL::isStateValid, this, _1));
// 	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
// 	space->setup();
// 	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
// 	//simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(100 / space->getMaximumExtent());
// 	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
// 	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRT(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::PRMstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::LBTRRT(simpleSetUp->getSpaceInformation())));
}

ob::ValidStateSamplerPtr PlannerOMPL::allocOBValidStateSampler(const ob::SpaceInformation *si)
{
	return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
}

void PlannerOMPL::initialize(Sampler *sampler )
{
	
	xMin = 0.;      ////OJO ARREGLAR ESTO
	xMax = 10000.;
	zMin = -10000.;
	zMax = 0.;
	
	//Create state space as R2
	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
	space->addDimension(xMin, xMax);
	space->addDimension(zMin, zMax);
	
	//Setup class
	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

	//set Sampler
	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
	
	// set state validity checking for this space

	//simpleSetUp->setStateValidityChecker(boost::bind(&PlannerOMPL::isStateValid, this, _1));
	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
	space->setup();
	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
	//simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(100 / space->getMaximumExtent());
	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);
}

// bool PlannerOMPL::isStateValid(const ompl::base::State* state) const
// {
// 	const float x = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], (int)xMax);
// 	const float z = std::max((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], (int)zMin);
// 	
// 	innerModel->updateTransformValues("robot", x, 0, z, 0, 0, 0);
// 	
// 	for (uint32_t in=0; in<robotNodes.size(); in++)
// 	{
// 		for (uint32_t out=0; out<restNodes.size(); out++)
// 		{
// 			if (innerModel->collide(robotNodes[in], restNodes[out]))
// 			{
// 				return false;
// 			}
// 		}
// 	}
// 	return true;
// }

//bool PlannerOMPL::computePath(const QVec& target, InnerModel* inner)
bool PlannerOMPL::computePath(const QVec& origin, const QVec &target, int maxTime)
{
	//Planning proper
	if (simpleSetUp == NULL)
		return false;
	
	simpleSetUp->clear();
	
	ob::ScopedState<> start(simpleSetUp->getStateSpace());
	start[0] = origin.x();	start[1] = origin.z();
	ob::ScopedState<> goal(simpleSetUp->getStateSpace());
	goal[0] = target.x();	goal[1] = target.z();
	simpleSetUp->setStartAndGoalStates(start, goal);
	simpleSetUp->getProblemDefinition()->print(std::cout);
	
	ob::PlannerStatus solved = simpleSetUp->solve(maxTime);

	if (solved)
	{
		std::cout << __FILE__ << __FUNCTION__ << "RRT, found solution with " << simpleSetUp->getSolutionPath().getStateCount() << " waypoints" << std::endl;;
	
		//if (simpleSetUp->haveSolutionPath())	
		//simpleSetUp->simplifySolution();
		og::PathGeometric &p = simpleSetUp->getSolutionPath();
		std::cout << __FILE__ << __FUNCTION__ << "Solution after simplify: " << p. getStateCount() << ". Path length: " << p.length() << std::endl;
		p.print(std::cout);

 		simpleSetUp->getPathSimplifier()->simplify(p,5);
 		//simpleSetUp->getPathSimplifier()->smoothBSpline(p);
//		p.interpolate();
		
		for (std::size_t i = 0; i < p.getStateCount(); ++i)
		{
			currentPath.append( QVec::vec3( p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0], 
											0, 
											p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]));
		}
		return true;
	}
	else
		return false;
}

// void PlannerOMPL::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
// {
// 	if (node->id == robotId)
// 	{
// 		inside = true;
// 	}
// 	
// 	InnerModelMesh *mesh;
// 	InnerModelPlane *plane;
// 	InnerModelTransform *transformation;
// 
// 	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
// 	{
// 		for (int i=0; i<node->children.size(); i++)
// 		{
// 			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out);
// 		}
// 	}
// 	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
// 	{
// 		if (inside)
// 		{
// 			in.push_back(node->id);
// 		}
// 		else
// 		{
// 			out.push_back(node->id);
// 		}
// 	}
// }