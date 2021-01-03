/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::Event::Event(ImplicitGraph *const graphPtr) : graphPtr_(graphPtr)
        {
            // Do nothing.
        }

        IGLS::Event::isTriggered(VertexConstPtr &vertex)
        {
            return (vertex == graphPtr_->getGoalVertex());
        }

    }  // namespace geometric
}  // namespace ompl
