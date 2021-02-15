/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::Event::Event()
        {
            // Do nothing.
        }

        void IGLS::Event::setup(ImplicitGraph *graphPtr)
        {
            graphPtr_ = graphPtr;
        }

        bool IGLS::Event::isTriggered(const VertexPtr &vertex) const
        {
            return (vertex == graphPtr_->getGoalVertex());
        }

        IGLS::ConstantDepthEvent::ConstantDepthEvent(std::size_t depth) : IGLS::Event(), depth_(depth)
        {
            // Do nothing.
        }

        bool IGLS::ConstantDepthEvent::isTriggered(const VertexPtr &vertex) const
        {
            if (vertex->isRoot())
            {
                return false;
            }
            if (vertex->getId() == graphPtr_->getGoalVertex()->getId())
            {
                return true;
            }
            // Compute the number of lazy edges in the subpath from start to \c vertex.

            // Iterate backwards from the current vertex.
            VertexPtr curVertex = vertex;
            int lazyDepth = 0;
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
                // Check if this edge is evaluated. It is enough to check for collision-free edges.
                if (!curVertex->hasWhitelistedChild(curVertex->getParent()) &&
                    !curVertex->getParent()->hasWhitelistedChild(curVertex))
                {
                    lazyDepth++;
                    if (lazyDepth == depth_)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        IGLS::SubpathExistenceEvent::SubpathExistenceEvent(
            const double threshold,
            const IGLS::ExistenceGraph& existenceGraph)
          : IGLS::Event(), threshold_(threshold), existenceGraph_(std::move(existenceGraph))
        {
            // Do nothing.
        }

        bool IGLS::SubpathExistenceEvent::isTriggered(const VertexPtr &vertex) const
        {
            if (vertex->isRoot())
            {
                return false;
            }
            if (vertex->getId() == graphPtr_->getGoalVertex()->getId())
            {
                return true;
            }
            // Get the probability of the subpath.
            VertexPtr currVertex = vertex;
            double existenceProbability = 1.0;
            while (currVertex->getParent())
            {
                // If the edge is evaluated, continue to the next edge in the subpath.
                if (vertex->hasWhitelistedChild(vertex->getParent()) ||
                    vertex->getParent()->hasWhitelistedChild(vertex))
                {
                    continue;
                }
                existenceProbability *= existenceGraph_.edgeExistence(vertex->getParent(), vertex);
                if (existenceProbability < threshold_)
                {
                    return true;
                }
            }
            return false;
        }
    }  // namespace geometric
}  // namespace ompl
