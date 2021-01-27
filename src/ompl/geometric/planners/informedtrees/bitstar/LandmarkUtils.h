/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_LANDMARKUTILSH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_LANDMARKUTILSH_

#include <vector>
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace geometric
    {
        class HaltonSampler
        {
        public:
            HaltonSampler(std::vector<int> primes, const ompl::base::SpaceInformationPtr &spaceInformation);
            ~HaltonSampler() = default;
            void sample(const int index, ompl::base::State *state) const;

        private:
            const std::vector<int> primes_;
            const ompl::base::SpaceInformationPtr &spaceInformation_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_LANDMARKUTILSH_
