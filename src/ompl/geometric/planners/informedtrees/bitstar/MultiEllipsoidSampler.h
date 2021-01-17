/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_MULTIELLIPSOIDSAMPLESH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_MULTIELLIPSOIDSAMPLESH_

#include <ompl/base/samplers/InformedStateSampler.h>

namespace ompl
{
    namespace geometric
    {
        class MultiEllipsoidSampler : public ompl::base::InformedSampler
        {
        public:
            MultiEllipsoidSampler(const ompl::base::ProblemDefinitionPtr &pdef, unsigned int maxNumberCalls);
            // TODO(avk): I think overriding base class dtor is bad here.
            ~MultiEllipsoidSampler() override = default;

            bool sampleUniform(ompl::base::State * /*statePtr*/, const ompl::base::Cost & /*maxCost*/) override
            {
                return false;
            }
            bool sampleUniform(ompl::base::State * /*statePtr*/, const ompl::base::Cost & /*minCost*/,
                               const ompl::base::Cost & /*maxCost*/) override
            {
                return false;
            }

            bool sampleUniform(ompl::base::State *state, const ompl::base::State *focus, double startCost,
                               double goalCost);

            bool hasInformedMeasure() const override
            {
                return true;
            }

            double getInformedMeasure(const ompl::base::Cost &currentCost) const override
            {
                return startPhs->getPhsMeasure(currentCost.value()) + goalPhs->getPhsMeasure(currentCost.value());
            }

            ompl::base::Cost heuristicSolnCost(const ompl::base::State * /*state*/) const override
            {
                return ompl::base::Cost(0);
            }

        private:
            bool samplePhsRejectBounds(ompl::base::State *statePtr);

            ompl::ProlateHyperspheroidPtr randomPhsPtr()
            {
                const double startPhsMeasureFraction = startPhs->getPhsMeasure() / summedMeasure_;
                if (rng_.uniform01() < startPhsMeasureFraction)
                {
                    return startPhs;
                }
                return goalPhs;
            }

            bool keepSample(const std::vector<double> &informedVector)
            {
                if (startPhs->isInPhs(&informedVector[0]) && goalPhs->isInPhs(&informedVector[0]))
                {
                    return (rng_.uniform01() <= 0.5);
                }
                return true;
            }

            std::size_t mDimension;
            ompl::ProlateHyperspheroidPtr startPhs;
            ompl::ProlateHyperspheroidPtr goalPhs;
            double summedMeasure_;
            ompl::RNG rng_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_MULTIELLIPSOIDSAMPLERH_
