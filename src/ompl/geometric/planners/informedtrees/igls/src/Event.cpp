/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"

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

    }  // namespace geometric
}  // namespace ompl
