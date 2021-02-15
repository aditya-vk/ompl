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
                if (!u->hasWhitelistedChild(v) && !v->hasWhitelistedChild(u))
                {
                    edge = std::make_pair(u, v);
                    break;
                }
            }
            return edge;
        }

        IGLS::FailfastSelector::FailfastSelector(const IGLS::ExistenceGraph& existenceGraph)
          : existenceGraph_(std::move(existenceGraph))
        {
            // Do nothing.
        }

        IGLS::VertexPtrPair IGLS::FailfastSelector::edgeToEvaluate(const VertexPtrVector &reversePath) const
        {
            assert(reversePath.size() >= 2);

            // Iterate through edges.
            VertexPtrPair edge;
            double minimumProbability = 1.0;
            for (std::size_t i = 0; i < reversePath.size() - 1; i++)
            {
                // If the edge has been evaluated, continue.
                const VertexPtr &u = reversePath[i + 1];
                const VertexPtr &v = reversePath[i];
                if (u->hasWhitelistedChild(v) || v->hasWhitelistedChild(u))
                {
                    continue;
                }
                double edgeProbability = existenceGraph_.edgeExistence(u, v);
                if (edgeProbability <= minimumProbability)
                {
                    edge = std::make_pair(u, v);
                    minimumProbability = edgeProbability;
                }
            }
            return edge;
        }
    }  // namespace geometric
}  // namespace ompl
