/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_

#include "ompl/geometric/planners/informedtrees/IGLS.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor Selector
        @par Short description
        A class to define the strategy to choose edge to evaluate for collision. */

        /** \brief A selector.*/
        class IGLS::Selector
        {
        public:
            Selector();
            virtual ~Selector() = default;

            /** \brief Returns edge to evaluate.
             * Default behavior implements forward selector i.e. returns
             * unevaluated edge closest to the start */
            virtual VertexPtrPair edgeToEvaluate(const VertexPtrVector &reversePath) const;
        };

        /** \brief A selector.*/
        class IGLS::FailfastSelector : public Selector
        {
        public:
            FailfastSelector(const std::function<double(const VertexPtr &, const VertexPtr &)> &probabilityFunction);
            virtual ~FailfastSelector() = default;

            /** \brief Returns edge with least existence probability to evaluate. */
            VertexPtrPair edgeToEvaluate(const VertexPtrVector &reversePath) const override;

        private:
            /** \brief Returns probability of existence of edge between two vertices. */
            const std::function<double(const VertexPtr &, const VertexPtr &)> &probabilityFunction_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SELECTOR_
