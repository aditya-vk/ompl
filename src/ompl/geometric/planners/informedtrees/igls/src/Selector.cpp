/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Selector.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::Selector::Selector()
        {
            // Do nothing.
        }
        IGLS::VertexPtrPair IGLS::Selector::edgeToEvaluate(const VertexPtrVector &reversePath) const
        {
            assert(reversePath.size() >= 2);

            // Iterate through edges in the reverse order for a forward selector.
            VertexPtrPair edge;
            for (std::size_t i = reversePath.size() - 1; i > 0; i--)
            {
                // If the edge has been evaluated, continue.
                const VertexPtr &u = reversePath[i];
                const VertexPtr &v = reversePath[i - 1];
                if (!u->hasEvaluatedChild(v) && !v->hasEvaluatedChild(u))
                {
                    edge = std::make_pair(u, v);
                    break;
                }
            }
            return edge;
        }
    }  // namespace geometric
}  // namespace ompl
