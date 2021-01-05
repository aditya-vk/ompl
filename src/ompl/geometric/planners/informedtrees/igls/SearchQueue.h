/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SEARCHQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SEARCHQUEUE_

#include <array>
#include <functional>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/informedtrees/IGLS.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor SearchQueue
        \par Short Description
        A search queue holding vertices ordered on a sort key, i.e., a cost triple with a lexicographical comparison.
        The queue is implemented as a binary heap.
        */

        /** \brief A queue of vertices, sorted according to a sort key. */
        class IGLS::SearchQueue
        {
        public:
            // ---
            // Aliases.
            // ---

            /** \brief A triplet of costs, i.e., the vertex queue sorting key. */
            // TODO(avk): What are the three values?
            using SortKey = std::array<ompl::base::Cost, 2u>;

            /** \brief The data stored in the vertex-queue binary heap. */
            using SortKeyAndVertexPtr = std::pair<SortKey, VertexPtr>;

            /** \brief The function signature of the sorting function for the vertex queue*/
            using VertexComparisonFunction =
                std::function<bool(const SortKeyAndVertexPtr &, const SortKeyAndVertexPtr &)>;

            /** \brief The underlying vertex queue. */
            using VertexQueue = ompl::BinaryHeap<SortKeyAndVertexPtr, VertexComparisonFunction>;

            /** \brief An element pointer into the vertex queue binary heap */
            using VertexQueueElemPtr = VertexQueue::Element *;

            /** \brief A vector of vertex queue pointers */
            using VertexQueueElemPtrVector = std::vector<VertexQueueElemPtr>;

            // ---
            // Construction, initialization, and destruction.
            // ---

            /** \brief Construct a search queue. It must be setup before use. */
            SearchQueue(NameFunc nameFunc);

            /** \brief Destruct the search queue using the default deconstructor. */
            virtual ~SearchQueue() = default;

            /** \brief Setup the SearchQueue, must be called before use. */
            void setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr);

            /** \brief Reset the queue to the state of construction. */
            void reset();

            /** \brief Set whether cascading of rewirings is enabled. */
            void enableCascadingRewirings(bool enable);

            // ---
            // Insertion.
            // ---

            /** \brief Update the vertex queue by adding all the potential neighbors from the vertex.*/
            void insertNeighborVertices(const VertexPtr &vertex);

            // ---
            // Access.
            // ---

            /** \brief Get the best vertex on the queue, leaving it at the front of the queue. */
            VertexPtr getFrontVertex();

            /** \brief Get the value of the best vertex on the queue, leaving it at the front of the queue. */
            SortKey getFrontVertexValue();

            /** \brief Pop the best vertex off the queue. */
            VertexPtr popFrontVertex();

            // ---
            // Modification.
            // ---

            /** \brief Clears the queue. */
            void clear();

            // TODO(avk): Disabled rebuilding the vertex queue.

            /** \brief Update the sort key of a particular vertex and its position in the queue. */
            void update(const VertexQueueElemPtr elementPtr);

            /** \brief Mark that a solution has been found */
            void registerSolutionCost(const ompl::base::Cost &solutionCost);

            // ---
            // Information access.
            // ---

            /** \brief Allow access to the current search id. */
            std::shared_ptr<const unsigned int> getSearchId() const;

            /** \brief The condition used to insert vertices into the queue. Compares lowerBoundHeuristicVertex to the
             * given threshold. Returns true if the vertex's best cost is lower than the internally set threshold.*/
            bool canPossiblyImproveCurrentSolution(const VertexPtr &state) const;

            /** \brief Returns the number of vertices in the queue. */
            unsigned int numVertices();

            /** \brief Returns true if the queue is empty. */
            bool isEmpty();

            /** \brief Get a copy of the vertex queue. This is expensive and is only meant for animations/debugging. */
            void getVertices(VertexConstPtrVector *vertexQueue);

            /** \brief The number of vertices popped off the queue for processing. */
            unsigned int numVerticesPopped() const;

        private:
            // ---
            // High level primitives.
            // ---

            /** \brief Iterate through the list of neighbouring samples and add potential neighbors to queue. */
            void enqueueVertices(const VertexPtr &parent, const VertexPtrVector &possibleChildren);

            /** \brief Attempt to add a vertex to the queue. Checks that the vertex meets the queueing condition. */
            void enqueueNeighborConditionally(const VertexPtr &parent, const VertexPtr &neighbor);

            /** \brief Insert a vertex into the queue. */
            void enqueueVertex(const VertexPtr &vertex);

            // ---
            // Sorting.
            // ---

            /** \brief Constructs a sort key for the given vertex. */
            SortKey createSortKey(const VertexPtr &vertex) const;

            /** A lexicographical comparison function for the std::arrays of costs. */
            bool lexicographicalBetterThan(const std::array<ompl::base::Cost, 2> &lhs,
                                           const std::array<ompl::base::Cost, 2> &rhs) const;

            // ---
            // Debug helpers.
            // ---

            /** \brief Test if the class is setup and throw if not. */
            void assertSetup() const;

            // ---
            // Member variables (Make all are configured in setup() and reset in reset()).
            // ---

            /** \brief A function pointer to the planner name, for better OMPL_INFO, etc. output. */
            NameFunc nameFunc_;

            /** \brief Whether the class is setup */
            bool isSetup_{false};

            /** \brief Whether cascading of rewirings is enabled. */
            bool isCascadingOfRewiringsEnabled_{false};

            /** \brief A cost/heuristic helper class. As I am a copy of the version owned by IGLS.cpp, I can be reset
             * in a clear().*/
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The samples represented as an edge-implicit graph. */
            ImplicitGraph *graphPtr_{nullptr};

            /** \brief The underlying queue of vertices. Sorted by vertexQueueComparison. */
            VertexQueue vertexQueue_;

            /** \brief The cost of the best solution, which is the maximum heuristic value allowed for vertices in
             * the queue. */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief Whether the problem has a solution. */
            bool hasExactSolution_{false};

            /** \brief A counter for the number of times we've reset the vertex queue. Used to efficiently reset the
             * vertex queue lookups stored in vertices. */
            const std::shared_ptr<unsigned int> searchId_;

            /** \brief The number of vertices processed. */
            unsigned int numVerticesPopped_{0u};

        };  // class SearchQueue
    }       // namespace geometric
}  // namespace ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_SEARCHQUEUE_
