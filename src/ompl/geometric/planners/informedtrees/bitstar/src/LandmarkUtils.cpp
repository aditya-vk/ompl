#include "ompl/geometric/planners/informedtrees/bitstar/LandmarkUtils.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{
    namespace geometric
    {
        HaltonSampler::HaltonSampler(const std::vector<int> primes,
                                     const ompl::base::SpaceInformationPtr &spaceInformation)
          : primes_(primes), spaceInformation_(spaceInformation)
        {
            // Do nothing.
        }

        void HaltonSampler::sample(const int index, ompl::base::State *state) const
        {
            std::vector<double> position;
            for (const auto &prime : primes_)
            {
                auto tempIndex = index;
                double result = 0.0;
                double f = 1.0;
                while (tempIndex > 0)
                {
                    f /= prime;
                    result += f * (tempIndex % prime);
                    tempIndex /= prime;
                }
                position.push_back(result);
            }
            auto bounds = spaceInformation_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
            std::vector<double> difference = bounds.getDifference();
            for (int i = 0; i < difference.size(); ++i)
            {
                position[i] = bounds.low[i] + difference[i] * position[i];
            }

            // Now scale the halton sample appropriately.
            spaceInformation_->getStateSpace()->copyFromReals(state, position);
        }
    }  // namespace geometric
}  // namespace ompl
