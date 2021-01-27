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
            // TODO(avk): Placeholder event for constantDepth(1)
            if (!vertex->hasEvaluatedChild(vertex->getParent()) && !vertex->getParent()->hasEvaluatedChild(vertex))
            {
                return true;
            }
            return false;
        }

        IGLS::SubpathExistenceEvent::SubpathExistenceEvent(
            const double threshold,
            const std::function<double(const VertexPtr &, const VertexPtr &)> &probabilityFunction)
          : IGLS::Event(), threshold_(threshold), probabilityFunction_(probabilityFunction)
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
                existenceProbability *= probabilityFunction_(vertex->getParent(), vertex);
                if (existenceProbability < threshold_)
                {
                    return true;
                }
            }
            return false;
        }
    }  // namespace geometric
}  // namespace ompl
