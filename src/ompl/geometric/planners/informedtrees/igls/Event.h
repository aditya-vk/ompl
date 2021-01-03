/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EVENT_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EVENT_

#include "ompl/geometric/planners/informedtrees/IGLS.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor Event
        @par Short description
        A class to determine if the search need to pause for collision checking edges.

        /** \brief An Event.*/
        class IGLS::Event
        {
        public:
            explicit Event(ImplicitGraph *const graphPtr);
            virtual ~Event() = default;

            /** \brief Returns true if the vertex triggers the event.
             * Default behavior implements LazySP i.e. triggers when vertex is goal. */
            bool isTriggered(VertexConstPtr &vertex) const;

        private:
            ImplicitGraph *const graphPtr_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EVENT_
