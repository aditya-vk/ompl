/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/SearchQueue.h"

#include <algorithm>
#include <iterator>
#include <utility>

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/geometric/planners/informedtrees/igls/CostHelper.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"

// Debug macros
#ifdef IGLS_DEBUG
/** \brief A debug-only call to assert that the object is setup. */
#define ASSERT_SETUP this->assertSetup();
#else
#define ASSERT_SETUP
#endif  // IGLS_DEBUG

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        IGLS::SearchQueue::SearchQueue(NameFunc nameFunc)
          : nameFunc_(std::move(nameFunc))
          , vertexQueue_([this](const SortKeyAndVertexPtr &lhs, const SortKeyAndVertexPtr &rhs) {
              return lexicographicalBetterThan(lhs.first, rhs.first);
          })  // This tells the vertexQueue_ to use lexicographical comparison for sorting.
          , searchId_(std::make_shared<unsigned int>(1u))
        {
        }

        void IGLS::SearchQueue::setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr)
        {
            // Store that I am setup.
            isSetup_ = true;

            // Get access to the cost helper and the graph.
            costHelpPtr_ = costHelpPtr;
            graphPtr_ = graphPtr;

            // Set the the cost threshold to infinity to start.
            solutionCost_ = costHelpPtr_->infiniteCost();
        }

        void IGLS::SearchQueue::reset()
        {
            // Reset everything to the state of construction except for the planner name.
            // Keep this in the order of the constructors for easy verification.

            // The queue is not ready for handling data after resetting.
            isSetup_ = false;

            // Reset the pointers to external information.
            costHelpPtr_ = nullptr;
            graphPtr_ = nullptr;

            // Clear the queue.
            vertexQueue_.clear();

            // Reset the number of queues that have been searched.
            *searchId_ = 1u;

            // Reset the cost threshold to infinite cost.
            solutionCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());

            // Make sure the queue doesn't think it has a solution.
            hasExactSolution_ = false;

            // Finally, reset the progress info.
            numVerticesPopped_ = 0u;
        }

        void IGLS::SearchQueue::enableCascadingRewirings(bool enable)
        {
            isCascadingOfRewiringsEnabled_ = enable;
        }

        void IGLS::SearchQueue::enqueueVertex(const VertexPtr &vertex)
        {
            ASSERT_SETUP

            // If we already have the vertex in the queue, we need to update its value.
            VertexQueueElemPtr updateVertex = vertex->getVertexQueueLookup();
            if (updateVertex)
            {
                updateVertex->data.first = this->createSortKey(vertex);
                vertexQueue_.update(updateVertex);
            }
            else
            {
                // The iterator to the new vertex in the queue:
                VertexQueueElemPtr vertexElemPtr;
                vertexElemPtr = vertexQueue_.insert(std::make_pair(this->createSortKey(vertex), vertex));
                vertex->setVertexQueueLookup(vertexElemPtr);
            }
        }

        IGLS::VertexPtr IGLS::SearchQueue::getFrontVertex()
        {
            ASSERT_SETUP

#ifdef IGLS_DEBUG
            if (vertexQueue_.empty())
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // IGLS_DEBUG

            // Return the a copy of the front vertex.
            return vertexQueue_.top()->data.second;
        }

        IGLS::SearchQueue::SortKey IGLS::SearchQueue::getFrontVertexValue()
        {
            ASSERT_SETUP

#ifdef IGLS_DEBUG
            if (vertexQueue_.empty())
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // IGLS_DEBUG

            // Return a copy of the front value.
            return vertexQueue_.top()->data.first;
        }

        IGLS::VertexPtr IGLS::SearchQueue::popFrontVertex()
        {
            ASSERT_SETUP
#ifdef IGLS_DEBUG
            if (vertexQueue_.empty())
            {
                throw ompl::Exception("Attempted to pop an empty SearchQueue.");
            }
#endif  // IGLS_DEBUG

            // Increment the counter of popped vertices.
            ++numVerticesPopped_;

            // Get the front element in the edge queue.
            VertexQueueElemPtr frontVertexQueueElement = vertexQueue_.top();
            VertexPtr frontVertex = frontVertexQueueElement->data.second;

#ifdef IGLS_DEBUG
            if (frontVertex.first->isPruned() || frontVertex.second->isPruned())
            {
                throw ompl::Exception("The edge queue contains an edge with a pruned vertex.");
            }
#endif  // IGLS_DEBUG

            // Remove the vertex from the respective lookups.
            frontVertex->clearVertexQueueLookup();

            // Remove it from the queue.
            vertexQueue_.pop();

            // Return the vertex.
            return frontVertex;
        }

        void IGLS::SearchQueue::registerSolutionCost(const ompl::base::Cost &solutionCost)
        {
            ASSERT_SETUP

            // Flag
            hasExactSolution_ = true;

            // Store
            solutionCost_ = solutionCost;
        }

        void IGLS::SearchQueue::clear()
        {
            ASSERT_SETUP

            // Clear the edge queue.
            vertexQueue_.clear();

            // Increment the queue processing number.
            ++(*searchId_);
        }

        std::shared_ptr<const unsigned int> IGLS::SearchQueue::getSearchId() const
        {
            return searchId_;
        }

        bool IGLS::SearchQueue::canPossiblyImproveCurrentSolution(const VertexPtr &state) const
        {
            ASSERT_SETUP

#ifdef IGLS_DEBUG
            if (state->isPruned())
            {
                throw ompl::Exception("Asking whether pruned state can possibly improve current solution.");
            }
#endif  // IGLS_DEBUG

            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                                solutionCost_);
        }

        unsigned int IGLS::SearchQueue::numVertices()
        {
            ASSERT_SETUP

            return vertexQueue_.size();
        }

        bool IGLS::SearchQueue::isEmpty()
        {
            ASSERT_SETUP

            return vertexQueue_.empty();
        }

        void IGLS::SearchQueue::getVertices(VertexConstPtrVector *vertexQueue)
        {
            ASSERT_SETUP

            // Clear the inout argument.
            vertexQueue->clear();

            // Get the contents on the binary heap (key and edge).
            std::vector<SortKeyAndVertexPtr> queueContents;
            vertexQueue_.getContent(queueContents);

            // Fill the inout argument.
            for (const auto &queueElement : queueContents)
            {
                vertexQueue->push_back(queueElement.second);
            }
        }

        void IGLS::SearchQueue::insertNeighborVertices(const VertexPtr &vertex)
        {
#ifdef IGLS_DEBUG
            if (vertex->isPruned())
            {
                throw ompl::Exception("Inserting outgoing edges of pruned vertex.");
            }
#endif  // IGLS_DEBUG
        // Should we expand this vertex?
            if (this->canPossiblyImproveCurrentSolution(vertex))
            {
                // Get the neighbouring samples.
                VertexPtrVector neighbourSamples;
                graphPtr_->nearestSamples(vertex, &neighbourSamples);

                // Add all outgoing edges to neighbouring vertices and samples.
                this->enqueueVertices(vertex, neighbourSamples);
            }
            // No else
        }

        void IGLS::SearchQueue::removeVertexFromQueue(const VertexPtr &vertex)
        {
            if (!vertexQueue_.empty())
            {
                const auto &elementPtr = vertex->getVertexQueueLookup();
                if (elementPtr)
                {
                    vertexQueue_.remove(elementPtr);
                }
                // Clear the lookup.
                vertex->clearVertexQueueLookup();
            }
            // No else, nothing to remove from.
        }

        void IGLS::SearchQueue::update(const VertexQueueElemPtr elementPtr)
        {
            assert(elementPtr);

            // Create the up-to-date sort key for this edge.
            elementPtr->data.first = createSortKey(elementPtr->data.second);

            // Update its position in the queue.
            vertexQueue_.update(elementPtr);
        }

        // TODO(avk): Check this function thoroughly again.
        void IGLS::SearchQueue::enqueueVertices(const VertexPtr &parent, const VertexPtrVector &possibleChildren)
        {
#ifdef IGLS_DEBUG
            if (!parent->isInTree())
            {
                auto msg = "Trying to enqueue edges from a parent (" + std::to_string(parent->getId()) +
                           ") that's not in the tree."s;
                throw std::runtime_error(msg);
            }
#endif
            // Start with this vertex' current kiddos.
            VertexPtrVector currentChildren;
            parent->getChildren(&currentChildren);
            for (const auto &child : currentChildren)
            {
                this->enqueueNeighborConditionally(parent, child);
            }

            // We need to store whether an outgoing edge is a rewiring.
            bool isExpandedAsRewiring = false;

            // Now consider all neighbouring vertices that are not already my kids.
            for (auto &child : possibleChildren)
            {
                // If this sample is not connected to the search tree, just enqueue the edge if it's useful.
                if (!child->isInTree())
                {
                    this->enqueueNeighborConditionally(parent, child);
                }
                else  // If this sample is part of the tree, we need to be a little more careful.
                {
                    if (isCascadingOfRewiringsEnabled_ || !parent->hasEverBeenExpandedAsRewiring())
                    {
                        // Remember that this parent is expanded as a rewiring.
                        isExpandedAsRewiring = true;

                        // Make sure the child is not the root and distinct from this vertex (which is the parent).
                        if (!child->isRoot() && child->getId() != parent->getId())
                        {
                            // Make sure edges to kiddos aren't added twice.
                            if (child->getParent()->getId() != parent->getId())
                            {
                                // Make sure the neighbour vertex is not already my parent.
                                if (parent->isRoot() || child->getId() != parent->getParent()->getId())
                                {
                                    // The neighbour is not my parent, attempt to queue the edge.
                                    this->enqueueNeighborConditionally(parent, child);
                                }
                                // No else, this vertex is my parent.
                            }
                            // No else
                        }
                        // No else
                    }
                }
            }

            // If the parent is expanded to a vertex in the tree, it is a rewiring. This needs to be registered.
            if (isExpandedAsRewiring)
            {
                parent->registerRewiringExpansion();
            }
        }

        // TODO(avk): This function needs thorough check since the event will be involved.
        void IGLS::SearchQueue::enqueueNeighborConditionally(const VertexPtr &parent, const VertexPtr &child)
        {
            // Don't enqueue the edge if it's blacklisted.
            // TODO(avk): Or the other way around.
            if (parent->hasBlacklistedChild(child))
            {
                return;
            }
            else
            {
                // Enqueue the vertex only if it can possibly improve the current solution.
                if (this->canPossiblyImproveCurrentSolution(child))
                {
                    this->enqueueVertex(child);
                }
            }
        }

        IGLS::SearchQueue::SortKey IGLS::SearchQueue::createSortKey(const VertexPtr &vertex) const
        {
            // The sort key of a vertex u is [ g_t(u) + h^hat(v); g_t(u) ].
            return {{costHelpPtr_->combineCosts(vertex->getCost(), costHelpPtr_->costToGoHeuristic(vertex)),
                     vertex->getCost()}};
        }

        bool IGLS::SearchQueue::lexicographicalBetterThan(const std::array<ompl::base::Cost, 2> &lhs,
                                                          const std::array<ompl::base::Cost, 2> &rhs) const
        {
            return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                                [this](const ompl::base::Cost &a, const ompl::base::Cost &b) {
                                                    return costHelpPtr_->isCostBetterThan(a, b);
                                                });
        }

        void IGLS::SearchQueue::assertSetup() const
        {
            if (isSetup_ == false)
            {
                throw ompl::Exception("IGLS::SearchQueue was used before it was setup.");
            }
        }

        unsigned int IGLS::SearchQueue::numVerticesPopped() const
        {
            return numVerticesPopped_;
        }

    }  // namespace geometric
}  // namespace ompl
