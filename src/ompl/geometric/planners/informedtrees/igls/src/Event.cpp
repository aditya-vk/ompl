/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Event.h"

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

        IGLS::ConstantDepthEvent::ConstantDepthEvent(std::size_t depth) : IGLS::Event(), depth_(depth)
        {
            // Do nothing.
        }

        IGLS::SubpathExistenceEvent::SubpathExistenceEvent(const double threshold)
          : IGLS::Event(), threshold_(threshold)
        {
            // Do nothing.
        }
    }  // namespace geometric
}  // namespace ompl
