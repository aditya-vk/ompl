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
        A class to determine if the search need to pause for collision checking edges. */

        /** \brief An Event.*/
        class IGLS::Event
        {
        public:
            Event();
            virtual ~Event() = default;

            /** \brief Setup the event to provide it with the graph. */
            void setup(ImplicitGraph *const graphPtr);

            /** \brief Returns true if the vertex triggers the event.
             * Default behavior implements LazySP i.e. triggers when vertex is goal. */
            virtual bool isTriggered(const VertexPtr &vertex) const;

        protected:
            ImplicitGraph *graphPtr_;
        };

        /** \brief An Event.*/
        class IGLS::ConstantDepthEvent : public IGLS::Event
        {
        public:
            explicit ConstantDepthEvent(std::size_t depth);

            /** \brief Returns true if the vertex triggers the event.
             * Default behavior implements LazySP i.e. triggers when vertex is goal. */
            bool isTriggered(const VertexPtr &vertex) const override;

        private:
            const std::size_t depth_;
        };

    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EVENT_
