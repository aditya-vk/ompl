/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_PRIORITYQUEUEH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_PRIORITYQUEUEH_

#include <functional>
#include <set>
#include <utility>
#include <vector>

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/blaze/blaze/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            /// A queue that consists of vertices ordered by their key. The queue is
            /// implemented as a static ordered list of the vertex IDs in the graph.
            /// TODO(avk): It is expected that Blaze stores with it a vector of vertices
            /// generated. Each of these vertices has a key/id with it that refers to the
            /// position in the vector.
            class PriorityQueue
            {
            public:
                /// \brief The function signature of the sorting function for the queue.
                using PriorityQueueComparator = std::function<bool(const int, const int)>;

                /// \brief The underlying vertex queue.
                using PriorityQueueMMap = std::set<int, PriorityQueueComparator>;

                /// \brief Constructor.
                /// Construct a search queue.
                PriorityQueue(const std::vector<VertexPtr> &generatedVertices, const std::string name = "PriorityQueue")
                  : mGeneratedVertices(generatedVertices)
                  , mName(name)
                  , mVertexQueue([this](const int lhs, const int rhs) { return queueComparison(lhs, rhs); })
                {
                    // Do nothing.
                }

                /// \brief Destructor.
                virtual ~PriorityQueue() = default;

                /// \brief Print the queue.
                void print() const
                {
                    for (auto iter = mVertexQueue.begin(); iter != mVertexQueue.end(); ++iter)
                    {
                        std::cout << mGeneratedVertices[*iter]->getID() << " "
                                  << mGeneratedVertices[*iter]->getCostToCome() << std::endl;
                    }
                }

                /// \brief Clears the queue to state of construction.
                void clear()
                {
                    mVertexQueue.clear();
                }

                /// \brief Insert a vertex into the vertex expansion queue.
                void enqueueVertex(const VertexID &newVertex)
                {
                    mVertexQueue.insert(newVertex);
                }

                /// \brief Remove a vertex from the vertex expansion queue.
                void dequeueVertex(const VertexID &oldVertex)
                {
                    auto iter = mVertexQueue.find(oldVertex);
                    if (iter != mVertexQueue.end())
                    {
                        mVertexQueue.erase(iter);
                    }
                }

                /// \brief Get the best vertex on the queue, leaving it at the front of the
                /// vertex queue.
                VertexID getFrontVertex() const
                {
                    if (this->isEmpty())
                    {
                        throw std::invalid_argument("Queue is empty. No front vertex.");
                    }
                    return *(mVertexQueue.begin());
                }

                /// \brief Pop the best vertex off the queue.
                VertexID popFrontVertex()
                {
                    auto topVertex = getFrontVertex();
                    mVertexQueue.erase(mVertexQueue.begin());
                    return topVertex;
                }

                /// \brief Returns the name of the queue.
                std::string getName() const
                {
                    return mName;
                }

                /// \brief Returns the size of the underling queue.
                std::size_t getSize() const
                {
                    return mVertexQueue.size();
                }

                /// \brief Returns true if the queue is empty.
                bool isEmpty() const
                {
                    return mVertexQueue.empty();
                }

            protected:
                /// A lexicographical comparison function for the std::pair of costs. This is
                /// the sorting function for the queue and is just a wrapper to
                /// std::lexicographical_compare.
                virtual bool queueComparison(const int lhs, const int rhs) const = 0;

                /// A datastructure holding all the generated vertices.
                const std::vector<VertexPtr> &mGeneratedVertices;

                /// Name of the search queue.
                const std::string mName;

                /// \brief The underlying queue of vertices. Sorted by
                /// PriorityQueueComparator.
                PriorityQueueMMap mVertexQueue;
            };

            /// A queue that consists of vertices to be expanded during search. Vertices are
            /// expanded as needed. The queue is implemented as a static ordered list of the
            /// vertices in the graph with a token (i.e., an iterator) pointing to the
            /// vertex that needs to be expanded. Queue ordered on vertex cost to come.
            class RepairQueue final : public PriorityQueue
            {
            public:
                /// \brief Constructor.
                /// Construct a search queue. It must be setup before use.
                RepairQueue(const std::vector<VertexPtr> &generatedVertices, const std::string name = "RepairQueue")
                  : PriorityQueue(generatedVertices, name)
                {
                    // Do nothing.
                }

                bool queueComparison(const int lhs, const int rhs) const override
                {
                    return (mGeneratedVertices[lhs]->getCostToCome() >= mGeneratedVertices[rhs]->getCostToCome());
                }
            };

            /// A queue that consists of vertices to be expanded during search. Vertices
            /// are expanded as needed. The queue is implemented as a static ordered list
            /// of the vertices in the graph with a token (i.e., an iterator) pointing to
            /// the vertex that needs to be expanded. Queue ordered on vertex total cost.
            class SearchQueue final : public PriorityQueue
            {
            public:
                /// \brief Constructor.
                /// Construct a search queue. It must be setup before use.
                SearchQueue(const std::vector<VertexPtr> &generatedVertices, const std::string name = "SearchQueue")
                  : PriorityQueue(generatedVertices, name)
                {
                    // Do nothing.
                }

                bool queueComparison(const int lhs, const int rhs) const override
                {
                    return (mGeneratedVertices[lhs]->getTotalCost() >= mGeneratedVertices[rhs]->getTotalCost());
                }
            };

            // Type aliases to simplify code.
            using SearchQueuePtr = std::shared_ptr<SearchQueue>;
            using RepairQueuePtr = std::shared_ptr<RepairQueue>;

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_PRIORITYQUEUE_H_
