/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/geometric/planners/informedtrees/igls/ExistenceGraph.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::Event::Event()
        {
            // Do nothing.
        }

        void IGLS::Event::setup(ImplicitGraph *const graphPtr, ExistenceGraph *const existenceGraphPtr)
        {
            graphPtr_ = graphPtr;
            existenceGraphPtr_ = existenceGraphPtr;
        }

        bool IGLS::Event::isTriggered(const VertexPtr &vertex) const
        {
            return (vertex->getId() == graphPtr_->getGoalVertex()->getId());
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
            if (vertex->getLazyDepth() == depth_)
            {
                return true;
            }
            return false;
        }

        IGLS::SubpathExistenceEvent::SubpathExistenceEvent(const double threshold)
          : IGLS::Event(), threshold_(threshold)
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
                const auto &parent = currVertex->getParent();
                const bool evaluatedEdge =
                    currVertex->hasWhitelistedChild(parent) || parent->hasWhitelistedChild(currVertex);
                if (!evaluatedEdge)
                {
                    existenceProbability *= existenceGraphPtr_->edgeExistence(parent, currVertex);
                    if (existenceProbability < threshold_)
                    {
                        return true;
                    }
                }
                currVertex = parent;
            }
            return false;
        }
    }  // namespace geometric
}  // namespace ompl
