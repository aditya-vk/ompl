/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::Event::Event(ImplicitGraph *const graphPtr) : graphPtr_(graphPtr)
        {
            // Do nothing.
        }

        bool IGLS::Event::isTriggered(const VertexPtr &vertex) const
        {
            return (vertex == graphPtr_->getGoalVertex());
        }

        IGLS::ConstantDepthEvent::ConstantDepthEvent(ImplicitGraph *const graphPtr, std::size_t depth)
          : IGLS::Event(graphPtr), depth_(depth)
        {
            // Do nothing.
        }

        bool IGLS::ConstantDepthEvent::isTriggered(const VertexPtr &vertex) const
        {
            if (vertex->isRoot())
            {
                return false;
            }
            // TODO(avk): Placeholder event for constantDepth(1)
            if (!vertex->getParent()->hasEvaluatedChild(vertex) ||
                (vertex->getId() == graphPtr_->getGoalVertex()->getId()))
            {
                return true;
            }
            return false;
        }
    }  // namespace geometric
}  // namespace ompl
