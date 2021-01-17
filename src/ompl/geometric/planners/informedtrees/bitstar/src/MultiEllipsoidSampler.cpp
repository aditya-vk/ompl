#include "ompl/geometric/planners/informedtrees/bitstar/MultiEllipsoidSampler.h"

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <iostream>
#include <memory>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        using InformedSampler = ompl::base::InformedSampler;

        MultiEllipsoidSampler::MultiEllipsoidSampler(const ompl::base::ProblemDefinitionPtr &pdef,
                                                     unsigned int maxNumberCalls)
          : ompl::base::InformedSampler(pdef, maxNumberCalls)
        {
            mDimension = InformedSampler::space_->getDimension();
        }

        bool MultiEllipsoidSampler::sampleUniform(ompl::base::State *state, const ompl::base::State *focus,
                                                  double startCost, double goalCost)
        {
            // Construct the start and goal ellipsoids.
            std::vector<double> startVector;
            InformedSampler::space_->copyToReals(startVector, probDefn_->getStartState(0));

            std::vector<double> focusVector;
            InformedSampler::space_->copyToReals(focusVector, focus);

            std::vector<double> goalVector;
            InformedSampler::space_->copyToReals(goalVector,
                                                 probDefn_->getGoal()->as<ompl::base::GoalState>()->getState());

            startPhs = std::make_shared<ompl::ProlateHyperspheroid>(mDimension, &startVector[0], &focusVector[0]);
            goalPhs = std::make_shared<ompl::ProlateHyperspheroid>(mDimension, &focusVector[0], &goalVector[0]);

            // Compute the summed measure.
            startCost = std::max(startCost, InformedSampler::space_->distance(probDefn_->getStartState(0), focus));
            goalCost = std::max(goalCost, InformedSampler::space_->distance(
                                              focus, probDefn_->getGoal()->as<ompl::base::GoalState>()->getState()));
            startPhs->setTransverseDiameter(startCost);
            goalPhs->setTransverseDiameter(goalCost);
            summedMeasure_ = startPhs->getPhsMeasure() + goalPhs->getPhsMeasure();

            // Sample and return the state.
            return samplePhsRejectBounds(state);
        }

        bool MultiEllipsoidSampler::samplePhsRejectBounds(ompl::base::State *state)
        {
            bool foundSample = false;

            while (!foundSample)
            {
                std::vector<double> informedVector(mDimension);
                ompl::ProlateHyperspheroidPtr phs = randomPhsPtr();
                rng_.uniformProlateHyperspheroid(phs, &informedVector[0]);
                foundSample = keepSample(informedVector);

                if (foundSample)
                {
                    InformedSampler::space_->copyFromReals(state, informedVector);
                    foundSample = InformedSampler::space_->satisfiesBounds(state);
                }
            }
            return foundSample;
        }

    }  // namespace geometric
}  // namespace ompl
