/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Selector.h"

namespace ompl
{
    namespace geometric
    {
        VertexPtrPair IGLS::Selector::edgeToEvaluate(const VertexConstPtrVector &reversePath)
        {
            // TODO(avk): Make a better assert similar to Gammell's.
            assert(reversePath.size() >= 2);
            VertexPtrPair edge = std::make_pair(reversePath[0], reversePath[1]);
            return edge;
        }

    }  // namespace geometric
}  // namespace ompl
