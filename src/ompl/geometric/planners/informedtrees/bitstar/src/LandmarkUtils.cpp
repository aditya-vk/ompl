#include "ompl/geometric/planners/informedtrees/bitstar/LandmarkUtils.h"

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
            // TODO(avk): Scale the halton sample.
            // spaceInformation_->getStateSpace()->getLowerLimits();

            // Now scale the halton sample appropriately.
            spaceInformation_->getStateSpace()->copyFromReals(state, position);
        }
    }  // namespace geometric
}  // namespace ompl
