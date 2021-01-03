/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_

#include "ompl/geometric/planners/informedtrees/IGLS.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor Event
        @par Short description
        A class to determine if the search need to pause for collision checking edges.

        /** \brief An Event.*/
        class IGLS::Selector
        {
        public:
            Selector();
            virtual ~Selector() = default;

            /** \brief Returns true if the vertex triggers the event.
             * Default behavior implements forward selector i.e. returns
             * unevaluated edge closest to the start */
            VertexPtrPair edgeToEvaluate(const VertexConstPtrVector &path) const;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_
