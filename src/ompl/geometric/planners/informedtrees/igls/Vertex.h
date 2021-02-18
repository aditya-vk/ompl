/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_VERTEX_

#include <memory>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/planners/informedtrees/IGLS.h"
#include "ompl/geometric/planners/informedtrees/igls/SearchQueue.h"
#include "ompl/geometric/planners/informedtrees/igls/ExistenceGraph.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor gVertex
        @par Short description
        A class to store a state as a vertex in a (tree) graph. Allocates and frees it's own memory on
        construction/destruction. Parent vertices are owned by their children as shared pointers, assuring that a parent
        vertex will not be deleted while the child exists. Child vertices are owned by their parents as weak pointers,
        assuring that the shared-pointer ownership loop is broken.

        @par Note
        Add/Remove functions should almost always update their children's cost. The only known exception is when a
        series of operations are being performed and it would be beneficial to delay the update until the last
        operation. In this case, make sure that the last call updates the children and is on the highest ancestor that
        has been changed. Updates only flow downstream.
        */

        /** \brief The vertex of the underlying graphs in \ref IGLS. */
        class IGLS::Vertex
        {
        public:
            // ---
            // Construction and destruction.
            // ---

            /** \brief Construct a vertex using space information, and helpers to compute various costs. */
            Vertex(ompl::base::SpaceInformationPtr spaceInformation, const CostHelper *const costHelpPtr,
                   SearchQueue *const queuePtr, const std::shared_ptr<const unsigned int> &approximationId,
                   ExistenceGraph *const existenceGraphPtr, bool root = false);

            /** \brief Destruct a vertex. */
            virtual ~Vertex();

            // ---
            // State access.
            // ---

            /** \brief The (unique) vertex ID. */
            IGLS::VertexId getId() const;

            /** \brief The state of a vertex as a pointer. */
            ompl::base::State *state();

            /** \brief The state of a vertex as a pointer to const. */
            ompl::base::State const *state() const;

            // ---
            // Graph information access.
            // ---

            /** \brief Returns whether the vertex is the root of the search tree. */
            bool isRoot() const;

            /** \brief Returns whether this vertex has a parent. */
            bool hasParent() const;

            /** \brief Get whether a vertex is in the search tree or a sample (i.e., a vertex of the RRG). */
            bool isInTree() const;

            /** \brief Get the depth of the vertex from the root. */
            unsigned int getDepth() const;

            /** \brief Get the depth of the vertex from the root. */
            unsigned int getLazyDepth() const;

            /** \brief Get the depth of the vertex from the root. */
            double getExistenceProbability() const;

            /** \brief Get a const pointer to the parent of this vertex. */
            VertexConstPtr getParent() const;

            /** \brief Get a pointer to the parent of this vertex. */
            VertexPtr getParent();

            /** \brief Whether the vertex has been pruned. */
            bool isPruned() const;

            /** \brief Returns whether the vertex is expanded on current approximation. */
            bool isExpandedOnCurrentApproximation() const;

            /** \brief Returns whether the vertex is expaned on current search. */
            bool isExpandedOnCurrentSearch() const;

            /** \brief Returns whether the vertex has ever been expanded to vertices. */
            bool hasEverBeenExpandedToVertices() const;

            // ---
            // Graph modification.
            // ---

            /** \brief Set the parent of this vertex, cannot be used to replace a previous parent. Will always update
             * this vertex's cost, and can update descendent costs. */
            void addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost, bool incomingEdgeEvaluated);

            /** \brief Remove the parent of this vertex. Will always update this vertex's cost, and can update the
             * descendent costs. */
            void removeParent(bool updateChildCosts, bool updateLazyParameters);

            /** \brief Get whether this vertex has any children. */
            bool hasChildren() const;

            /** \brief Get the children of a vertex as constant pointers. */
            void getChildren(VertexConstPtrVector *children) const;

            /** \brief Get the children of a vertex as mutable pointers. */
            void getChildren(VertexPtrVector *children);

            /** \brief Add a child to this vertex. Does not change this vertex's cost or those of its descendants.
             * Child must already have this vertex listed as it's parent. */
            void addChild(const VertexPtr &newChild);

            /** \brief Remove a child from this vertex. Does not change this vertex's cost or those of its descendants.
             * Child must still have this vertex listed as its parent and it will also throw an exception if the given
             * vertex pointer is not found. */
            void removeChild(const VertexPtr &oldChild);

            /** \brief Clears children. */
            void clearChildren();

            /** \brief Adds a cached neighbor */
            void cacheNeighbor(const VertexPtr &neighbor);

            /** \brief Removes a cached neighbor */
            void uncacheNeighbor(const VertexPtr &neighbor);

            /** \brief Returns the list of cached neighbors */
            void getCachedNeighbors(VertexPtrVector *neighbors) const;
            void getCachedNeighbors(VertexConstPtrVector *neighbors) const;

            /** \brief Returns true if vertex is a cached neighbor. */
            bool hasCachedNeighbor(const VertexConstPtr &vertex) const;

            /** \brief Clears cached neighbors. */
            void clearCachedNeighbors();

            /** \brief Put the vertex on the blacklist of children. */
            void blacklistChild(const VertexConstPtr &vertex);

            /** \brief Put the vertex on the whitelist of children. */
            void whitelistChild(const VertexConstPtr &vertex);

            /** \brief Returns true if the vertex is blacklisted as a child of this vertex. */
            bool hasBlacklistedChild(const VertexConstPtr &vertex) const;

            /** \brief Returns true if the vertex is blacklisted as a child of this vertex. */
            bool hasWhitelistedChild(const VertexConstPtr &vertex) const;

            /** \brief Clears the blacklist. */
            void clearBlacklist();

            /** \brief Clears the whitelist. */
            void clearWhitelist();

            /** \brief Check if this vertex has an edge evaluated to the given vertex */
            bool hasEvaluatedChild(const VertexConstPtr &vertex) const;

            /** \brief Get the cost-to-come of a vertex. Return infinity if the edge is disconnected. */
            ompl::base::Cost getCost() const;

            /** \brief Get the incremental cost-to-come of a vertex. */
            ompl::base::Cost getEdgeInCost() const;

            /** \brief Mark the vertex as expanded. */
            void registerExpansion();

            /** \brief Mark expansion to vertices. */
            void registerExpansionToVertices(const bool status);

            /** \brief Mark the vertex as pruned. */
            void markPruned();

            /** \brief Mark the vertex as unpruned. */
            void markUnpruned();

            /** \brief Mark the vertex as inconsistent. */
            void markInconsistent();

            /** \brief Mark the vertex as consistent. */
            void markConsistent();

            /** \brief Returns true if the vertex is consistent. */
            bool isConsistent() const;

            // ---
            // Vertex queue lookups. Although IGLS has two different queues for search and repair,
            // a vertex can only be in one queue at any given time and it's iterator in that queue
            // is held by the lookup.
            // ---
            /** \brief Sets the iterator in the queue that corresponds to this vertex. */
            void setVertexQueueLookup(const SearchQueue::VertexQueueElemPtr &element);

            /** \brief Returns the iterator in the queue corresponding to this vertex. */
            SearchQueue::VertexQueueElemPtr getVertexQueueLookup();

            // Clear a vertex's search queue iterator.
            void clearVertexQueueLookup();

            // Updates the lazy parameters.
            void updateLazyParametersOnEdgeEvaluation(bool incomingEdgeEvaluated);

        private:
            // ---
            // Internal bookkeeping.
            // ---

            /** \brief Calculates the updated cost and depth of the current state, optionally calls itself on all
             * children. */
            void updateCostAndDepth(bool cascadeUpdates = true, bool incomingEdgeEvaluated = true);

            // ---
            // Member variables.
            // ---

            /** \brief The vertex id. */
            IGLS::VertexId id_;

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr si_;

            /** \brief The helper class to compute different costs. */
            const CostHelper *const costHelpPtr_;

            /** \brief The search queue used by the algorithms. */
            SearchQueue *const queuePtr_;

            ExistenceGraph *const existenceGraphPtr_;

            /** \brief The state itself. */
            ompl::base::State *state_;

            /** \brief Whether the vertex is a root. */
            bool isRoot_;

            /** \brief Whether the vertex is pruned. Vertices throw if any member function other than isPruned() is
             * access after they are pruned. */
            bool isPruned_{false};

            /** \brief Whether the vertex is consistent. */
            bool isConsistent_{true};

            /** \brief The depth of the state.  */
            unsigned int depth_{0u};

            /** \brief The lazy depth in the subpath to this vertex. */
            unsigned int lazyDepth_{0u};

            /** \brief The existence probability of the subpath to this vertex. */
            double existenceProbability_{1.0};

            /** \brief The parent state as a shared pointer such that the parent will not be deleted until all the
             * children are. */
            VertexPtr parentPtr_;

            /** \brief The incremental cost to get to the state. I.e., the cost of the parent -> state edge. */
            ompl::base::Cost edgeCost_;

            /** \brief The cost-to-come to this vertex. */
            ompl::base::Cost cost_;

            /** \brief The child states as weak pointers, such that the ownership loop is broken and a state can be
             * deleted once it's children are. */
            std::vector<VertexWeakPtr> children_;

            /** \brief The cached neighbors. */
            std::vector<VertexWeakPtr> cachedNeighbors_;

            /** \brief A pointer to the position in the search queue. */
            SearchQueue::VertexQueueElemPtr vertexQueueLookup_;

            /** \brief A collection of potential child vertex ids that are blacklisted for edges (due to a collision).
             */
            std::set<IGLS::VertexId> childIdBlacklist_;

            /** \brief A collection of potential child vertex ids that are whitelisted for edges. */
            std::set<IGLS::VertexId> childIdWhitelist_;

            /** \brief The id number associated with the search in which the lookups are up to date. */
            // TODO(avk): Why are these IDs required??
            unsigned int lookupApproximationId_{0u};

            /** \brief The id number associated with the approximation on which this vertex was last expanded. */
            unsigned int expansionApproximationId_{0u};

            /** \brief The id number associated with the search in which this vertex was last expanded. */
            unsigned int expansionSearchId_{0u};

            /** \brief Whether this sample has ever been expanded to vertices. */
            bool hasEverBeenExpandedToVertices_{false};

            /** \brief A pointer to the shared memory that holds the current search id. */
            const std::shared_ptr<const unsigned int> currentSearchId_;

            /** \brief A pointer to the shared memory that holds the current approximation id. */
            const std::shared_ptr<const unsigned int> currentApproximationId_;

            /** \brief A helper function to clear existing lookups if they are out of date (i.e., created at a different
             * id than the one given). */
            void clearLookupsIfOutdated();
        };  // class Vertex
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_VERTEX_
