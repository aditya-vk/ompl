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

        /** \brief ShortestPathEvent.*/
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

        /** \brief ConstantDepthEvent.*/
        class IGLS::ConstantDepthEvent : public IGLS::Event
        {
        public:
            explicit ConstantDepthEvent(const std::size_t depth);

            /** \brief Returns true if the vertex triggers the event.
             * Triggers when the lazy depth of the vertex in the tree is \c depth. */
            bool isTriggered(const VertexPtr &vertex) const override;

        private:
            /** \brief Denotes the depth at which a vertex should trigger. */
            const std::size_t depth_;
        };

        /** \brief SubpathExistenceEvent.*/
        class IGLS::SubpathExistenceEvent : public IGLS::Event
        {
        public:
            explicit SubpathExistenceEvent(
                const double threshold,
                const IGLS::ExistenceGraph& existenceGraph);

            /** \brief Returns true if the vertex triggers the event.
             * Triggers when the lazy subpath to vertex has existence probability less than \c threshold. */
            bool isTriggered(const VertexPtr &vertex) const override;

        private:
            /** \brief Denotes the threshold below which the the event triggers. */
            const double threshold_;

            /** \brief Returns probability of existence of edge between two vertices. */
            const IGLS::ExistenceGraph& existenceGraph_;
        };

    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EVENT_
