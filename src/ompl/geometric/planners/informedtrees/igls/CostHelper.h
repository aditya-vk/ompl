/* Authors: Jonathan Gammell, Marlin Strub */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_COSTHELPER_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_COSTHELPER_

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/util/Exception.h"

#include "ompl/geometric/planners/informedtrees/IGLS.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor CostHelper
        \par Short Description
        A header-only class that consolidates all the various heuristic calculations for vertices/edges in a graph into
        one place. Most of these functions are simply combinatorial pass-throughs to the OptimizationObjective. */

        /** \brief A helper class to handle the various heuristic functions in one place. */
        class IGLS::CostHelper
        {
        public:
            ////////////////////////////////
            // Public functions:
            /** \brief Construct the heuristic helper, must be setup before use. */
            CostHelper() = default;

            virtual ~CostHelper() = default;

            /** \brief Setup the CostHelper, must be called before use */
            inline void setup(const ompl::base::OptimizationObjectivePtr &opt, ImplicitGraph *graph)
            {
                opt_ = opt;
                graphPtr_ = graph;
            };

            /** \brief Reset the CostHelper, returns to state at construction. */
            inline void reset()
            {
                opt_.reset();
                graphPtr_ = nullptr;
            };

            /** \brief Get the underling OptimizationObjective */
            inline ompl::base::OptimizationObjectivePtr getOptObj() const
            {
                return opt_;
            };

            //////////////////
            // Heuristic helper functions
            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex,
             * independent of the current cost-to-come. I.e., combines the heuristic estimates of the cost-to-come and
             * cost-to-go. */
            inline ompl::base::Cost lowerBoundHeuristicVertex(const VertexConstPtr &vertex) const
            {
#ifdef BITSTAR_DEBUG
                if (vertex->isPruned())
                {
                    throw ompl::Exception("Computing the lower bound heuristic through a pruned vertex.");
                }
#endif  // BITSTAR_DEBUG
                return this->combineCosts(this->costToComeHeuristic(vertex), this->costToGoHeuristic(vertex));
            };

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex,
             * dependent on the current cost-to-come. I.e., combines the current cost-to-come with a heuristic estimate
             * of the cost-to-go. */
            inline ompl::base::Cost currentHeuristicVertex(const VertexConstPtr &vertex) const
            {
                return this->combineCosts(vertex->getCost(), this->costToGoHeuristic(vertex));
            };

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge,
             * independent of the cost-to-come of the parent state. I.e., combines the heuristic estimates of the
             * cost-to-come, edge cost, and cost-to-go. */
            inline ompl::base::Cost lowerBoundHeuristicEdge(const VertexConstPtrPair &edgePair) const
            {
                return this->combineCosts(this->lowerBoundHeuristicToTarget(edgePair),
                                          this->costToGoHeuristic(edgePair.second));
            };

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge,
             * dependent on the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic
             * estimates of the edge cost, and cost-to-go. */
            inline ompl::base::Cost currentHeuristicEdge(const VertexConstPtrPair &edgePair) const
            {
                return this->combineCosts(this->currentHeuristicToTarget(edgePair),
                                          this->costToGoHeuristic(edgePair.second));
            };

            /** \brief Calculates a heuristic estimate of the cost of a path to the \e target of an edge, independent of
             * the current cost-to-come of the parent state. I.e., combines heuristics estimates of the cost-to-come and
             * the edge cost. */
            inline ompl::base::Cost lowerBoundHeuristicToTarget(const VertexConstPtrPair &edgePair) const
            {
                return this->combineCosts(this->costToComeHeuristic(edgePair.first), this->edgeCostHeuristic(edgePair));
            };

            /** \brief Calculates a heuristic estimate of the cost of a path to the \e target of an edge, dependent on
             * the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic estimates of
             * the edge cost. */
            inline ompl::base::Cost currentHeuristicToTarget(const VertexConstPtrPair &edgePair) const
            {
                return this->combineCosts(edgePair.first->getCost(), this->edgeCostHeuristic(edgePair));
            };

            /** \brief Calculate a heuristic estimate of the cost-to-come for a Vertex */
            inline ompl::base::Cost costToComeHeuristic(const VertexConstPtr &vertex) const
            {
#ifdef BITSTAR_DEBUG
                if (vertex->isPruned())
                {
                    throw ompl::Exception("Computing the cost to come heuristic to a pruned vertex.");
                }
#endif  // BITSTAR_DEBUG

                return this->motionCostHeuristic(graphPtr_->getStartVertex()->state(), vertex->state());
            };

            /** \brief Calculate a heuristic estimate of the cost of an edge between two Vertices */
            inline ompl::base::Cost edgeCostHeuristic(const VertexConstPtrPair &edgePair) const
            {
                return this->motionCostHeuristic(edgePair.first->state(), edgePair.second->state());
            };

            /** \brief Calculate a heuristic estimate of the cost-to-go for a Vertex */
            inline ompl::base::Cost costToGoHeuristic(const VertexConstPtr &vertex) const
            {
                return this->motionCostHeuristic(vertex->state(), graphPtr_->getGoalVertex()->state());
            };
            //////////////////

            //////////////////
            // Cost-calculation functions
            /** \brief The true cost of an edge, including constraints.*/
            inline ompl::base::Cost trueEdgeCost(const VertexConstPtrPair &edgePair) const
            {
                return this->motionCost(edgePair.first->state(), edgePair.second->state());
            };

            inline ompl::base::Cost trueEdgeCost(const VertexPtr &source, const VertexPtr &target) const
            {
                return this->motionCost(source->state(), target->state());
            };

            /** \brief Combine multiple costs. */
            template <typename... Costs>
            inline ompl::base::Cost combineCosts(const ompl::base::Cost &cost, const Costs &... costs) const
            {
                return this->combineCosts(cost, this->combineCosts(costs...));
            }

            /** \brief Inflate a cost by a given factor. */
            inline ompl::base::Cost inflateCost(const ompl::base::Cost &cost, double factor) const
            {
                return ompl::base::Cost(factor * cost.value());
            }

            //////////////////
            // Cost-comparison functions
            /** \brief Compare whether cost a is worse than cost b by checking whether b is better than a. */
            inline bool isCostWorseThan(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                // If b is better than a, then a is worse than b
                return this->isCostBetterThan(b, a);
            };

            /** \brief Compare whether cost a and cost b are not equivalent by checking if either a or b is better than
             * the other. */
            inline bool isCostNotEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                // If a is better than b, or b is better than a, then they are not equal
                return this->isCostBetterThan(a, b) || this->isCostBetterThan(b, a);
            };

            /** \brief Compare whether cost a is better or equivalent to cost b by checking that b is not better than a.
             */
            inline bool isCostBetterThanOrEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                // If b is not better than a, then a is better than, or equal to, b
                return !this->isCostBetterThan(b, a);
            };

            /** \brief Compare whether cost a is worse or equivalent to cost b by checking that a is not better than b.
             */
            inline bool isCostWorseThanOrEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                // If a is not better than b, than a is worse than, or equal to, b
                return !this->isCostBetterThan(a, b);
            };

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "oldCost", i.e.,
             * (newCost - oldCost)/oldCost. */
            inline double fractionalChange(const ompl::base::Cost &newCost, const ompl::base::Cost &oldCost) const
            {
                return this->fractionalChange(newCost, oldCost, oldCost);
            };

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "refCost", i.e.,
             * (newCost - oldCost)/refCost. */
            inline double fractionalChange(const ompl::base::Cost &newCost, const ompl::base::Cost &oldCost,
                                           const ompl::base::Cost &refCost) const
            {
                // If the old cost is not finite, than we call that infinite percent improvement
                if (!this->isFinite(oldCost))
                {
                    // Return infinity (but not beyond)
                    return std::numeric_limits<double>::infinity();
                }
                // Calculate and return
                return (newCost.value() - oldCost.value()) / refCost.value();
            };
            ////////////////////////////////

            //////////////////
            // Straight pass-throughs to OptimizationObjective
            inline bool isSatisfied(const ompl::base::Cost &a) const
            {
                return opt_->isSatisfied(a);
            };
            inline bool isFinite(const ompl::base::Cost &a) const
            {
                return opt_->isFinite(a);
            };
            inline bool isCostEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->isCostEquivalentTo(a, b);
            };
            inline bool isCostBetterThan(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->isCostBetterThan(a, b);
            };
            inline ompl::base::Cost betterCost(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->betterCost(a, b);
            };
            inline ompl::base::Cost combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->combineCosts(a, b);
            };
            inline ompl::base::Cost infiniteCost() const
            {
                return opt_->infiniteCost();
            };
            inline ompl::base::Cost identityCost() const
            {
                return opt_->identityCost();
            };
            inline ompl::base::Cost motionCostHeuristic(const ompl::base::State *a, const ompl::base::State *b) const
            {
                return opt_->motionCostHeuristic(a, b);
            };
            inline ompl::base::Cost motionCost(const ompl::base::State *a, const ompl::base::State *b) const
            {
                return opt_->motionCost(a, b);
            };
            //////////////////
            ////////////////////////////////

        private:
            ////////////////////////////////
            // Member variables:
            /** \brief A local pointer to the optimization objective */
            ompl::base::OptimizationObjectivePtr opt_;

            /** \brief A local pointer to the samples/vertices viewed as an implicit graph. As this is a copy of the
             * version owned by BITstar.cpp it can be reset in a clear(). */
            ImplicitGraph *graphPtr_;
            ////////////////////////////////
        };  // class CostHelper
    }       // namespace geometric
}  // namespace ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_COSTHELPER_
