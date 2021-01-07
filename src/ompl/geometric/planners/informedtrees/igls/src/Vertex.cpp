/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/geometric/planners/informedtrees/igls/HelperFunctions.h"
#include <utility>
#include "ompl/util/Exception.h"
#include "ompl/geometric/planners/informedtrees/igls/IdGenerator.h"
#include "ompl/geometric/planners/informedtrees/igls/CostHelper.h"

// Debug macros.
#ifdef IGLS_DEBUG
// Debug setting. The id number of a vertex to track. Requires IGLS_DEBUG to be defined in BITstar.h
#define TRACK_VERTEX_ID 0

/** \brief A helper function to print out every function called on vertex "TRACK_VERTEX_ID" that changes it */
#define PRINT_VERTEX_CHANGE                                                                                            \
    if (id_ == TRACK_VERTEX_ID)                                                                                        \
    {                                                                                                                  \
        std::cout << "Vertex " << id_ << ": " << __func__ << "" << std::endl;                                          \
    }

/** \brief Assert that the vertex is not pruned. */
#define ASSERT_NOT_PRUNED                                                                                              \
    if (this->isPruned_)                                                                                               \
    {                                                                                                                  \
        std::cout << "Vertex " << id_ << ": " << __func__ << std::endl;                                                \
        throw ompl::Exception("Attempting to access a pruned vertex.");                                                \
    }
#else
#define PRINT_VERTEX_CHANGE
#define ASSERT_NOT_PRUNED
#endif  // IGLS_DEBUG

// An anonymous namespace to hide the instance:
namespace
{
    // Global variables:
    // The initialization flag stating that the ID generator has been created:
    std::once_flag g_IdInited;
    // A pointer to the actual ID generator
    boost::scoped_ptr<ompl::geometric::IGLS::IdGenerator> g_IdGenerator;

    // A function to initialize the ID generator pointer:
    void initIdGenerator()
    {
        g_IdGenerator.reset(new ompl::geometric::IGLS::IdGenerator());
    }

    // A function to get the current ID generator:
    ompl::geometric::IGLS::IdGenerator &getIdGenerator()
    {
        std::call_once(g_IdInited, &initIdGenerator);
        return *g_IdGenerator;
    }
}  // namespace

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        IGLS::Vertex::Vertex(ompl::base::SpaceInformationPtr spaceInformation, const CostHelper *const costHelpPtr,
                             SearchQueue *const queuePtr, const std::shared_ptr<const unsigned int> &approximationId,
                             bool root)
          : id_(getIdGenerator().getNewId())
          , si_(std::move(spaceInformation))
          , costHelpPtr_(std::move(costHelpPtr))
          , queuePtr_(queuePtr)
          , state_(si_->allocState())
          , isRoot_(root)
          , edgeCost_(costHelpPtr_->infiniteCost())
          , cost_(costHelpPtr_->infiniteCost())
          , currentSearchId_(queuePtr->getSearchId())
          , currentApproximationId_(approximationId)
        {
            PRINT_VERTEX_CHANGE

            if (this->isRoot())
            {
                cost_ = costHelpPtr_->identityCost();
            }
            // No else, infinite by default
        }

        IGLS::Vertex::~Vertex()
        {
            PRINT_VERTEX_CHANGE

            // Free the state on destruction
            si_->freeState(state_);
        }

        IGLS::VertexId IGLS::Vertex::getId() const
        {
            ASSERT_NOT_PRUNED

            return id_;
        }

        ompl::base::State const *IGLS::Vertex::state() const
        {
            ASSERT_NOT_PRUNED

            return state_;
        }

        ompl::base::State *IGLS::Vertex::state()
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            return state_;
        }

        /////////////////////////////////////////////
        // The vertex's graph properties:
        bool IGLS::Vertex::isRoot() const
        {
            ASSERT_NOT_PRUNED

            return isRoot_;
        }

        bool IGLS::Vertex::hasParent() const
        {
            ASSERT_NOT_PRUNED

            return static_cast<bool>(parentPtr_);
        }

        bool IGLS::Vertex::isInTree() const
        {
            ASSERT_NOT_PRUNED

            return this->isRoot() || this->hasParent();
        }

        unsigned int IGLS::Vertex::getDepth() const
        {
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            if (!this->isRoot() && !this->hasParent())
            {
                throw ompl::Exception("Attempting to get the depth of a vertex that does not have a parent yet is not "
                                      "root.");
            }
#endif  // IGLS_DEBUG

            return depth_;
        }

        IGLS::VertexConstPtr IGLS::Vertex::getParent() const
        {
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            if (!this->hasParent())
            {
                if (this->isRoot())
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }
#endif  // IGLS_DEBUG

            return parentPtr_;
        }

        IGLS::VertexPtr IGLS::Vertex::getParent()
        {
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            if (!this->hasParent())
            {
                if (this->isRoot())
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }
#endif  // IGLS_DEBUG

            return parentPtr_;
        }

        void IGLS::Vertex::addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            // Assert I can take a parent
            if (this->isRoot())
            {
                throw ompl::Exception("Attempting to add a parent to the root vertex, which cannot have a parent.");
            }
            if (this->hasParent())
            {
                throw ompl::Exception("Attempting to add a parent to a vertex that already has one.");
            }
#endif  // IGLS_DEBUG

            // Store the parent.
            parentPtr_ = newParent;

            // Store the edge cost.
            edgeCost_ = edgeInCost;

            // Update my cost and that of my children.
            this->updateCostAndDepth(true);
        }

        void IGLS::Vertex::removeParent(bool updateChildCosts)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            // Assert I have a parent
            if (this->isRoot())
            {
                throw ompl::Exception("Attempting to remove the parent of the root vertex, which cannot have a "
                                      "parent.");
            }
            if (!this->hasParent())
            {
                throw ompl::Exception("Attempting to remove the parent of a vertex that does not have a parent.");
            }
#endif  // IGLS_DEBUG

            // Clear my parent
            parentPtr_.reset();

            // Update my cost and possibly the cost of my descendants:
            this->updateCostAndDepth(updateChildCosts);
        }

        bool IGLS::Vertex::hasChildren() const
        {
            ASSERT_NOT_PRUNED

            return !children_.empty();
        }

        void IGLS::Vertex::getChildren(VertexConstPtrVector *children) const
        {
            ASSERT_NOT_PRUNED

            children->clear();

            for (const auto &child : children_)
            {
#ifdef IGLS_DEBUG
                // Check that the weak pointer hasn't expired
                if (child.expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while collecting the "
                                          "children of a vertex.");
                }
#endif  // IGLS_DEBUG

                // Lock and push back
                children->push_back(child.lock());
            }
        }

        void IGLS::Vertex::getChildren(VertexPtrVector *children)
        {
            ASSERT_NOT_PRUNED

            children->clear();

            for (const auto &child : children_)
            {
#ifdef IGLS_DEBUG
                // Check that the weak pointer hasn't expired
                if (child.expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while collecting the "
                                          "children of a vertex.");
                }
#endif  // IGLS_DEBUG

                // Lock and push back
                children->push_back(child.lock());
            }
        }

        void IGLS::Vertex::addChild(const VertexPtr &child)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            // Assert that I am this child's parent
            if (child->isRoot())
            {
                throw ompl::Exception("Attempted to add a root vertex as a child.");
            }
            if (!child->hasParent())
            {
                throw ompl::Exception("Attempted to add child that does not have a listed parent.");
            }
            if (child->getParent()->getId() != id_)
            {
                throw ompl::Exception("Attempted to add someone else's child as mine.");
            }
#endif  // IGLS_DEBUG

            // Push back the shared_ptr into the vector of weak_ptrs, this makes a weak_ptr copy
            children_.push_back(child);

            // Leave the costs of the child out of date.
        }

        void IGLS::Vertex::removeChild(const VertexPtr &child)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            // Assert that I am this child's parent
            if (child->isRoot())
            {
                throw ompl::Exception("Attempted to remove a root vertex as a child.");
            }
            if (!child->hasParent())
            {
                throw ompl::Exception("Attempted to remove a child that does not have a listed parent.");
            }
            if (child->getParent()->getId() != id_)
            {
                throw ompl::Exception("Attempted to remove a child vertex from the wrong parent.");
            }
#endif  // IGLS_DEBUG

            // Variables
            // Whether the child has been found (and then deleted);
            bool foundChild;

            // Iterate over the vector of children pointers until the child is found. Iterators make erase easier
            foundChild = false;
            for (auto it = children_.begin(); it != children_.end() && !foundChild; ++it)
            {
#ifdef IGLS_DEBUG
                // Check that the weak pointer hasn't expired
                if (it->expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while removing a "
                                          "child from a vertex.");
                }
#endif  // IGLS_DEBUG

                // Check if this is the child we're looking for
                if (it->lock()->getId() == child->getId())
                {
                    // It is, mark as found
                    foundChild = true;

                    // First, clear the entry in the vector
                    it->reset();

                    // Then remove that entry from the vector efficiently
                    swapPopBack(it, &children_);
                }
                // No else.
            }

            // Leave the costs of the child out of date.
#ifdef IGLS_DEBUG
            if (!foundChild)
            {
                throw ompl::Exception("Attempting to remove a child vertex not present in the vector of children "
                                      "stored in the (supposed) parent vertex.");
            }
#endif  // IGLS_DEBUG
        }

        void IGLS::Vertex::blacklistChild(const VertexConstPtr &vertex)
        {
            childIdBlacklist_.emplace(vertex->getId());
        }

        void IGLS::Vertex::whitelistChild(const VertexConstPtr &vertex)
        {
            childIdWhitelist_.emplace(vertex->getId());
        }

        bool IGLS::Vertex::hasBlacklistedChild(const VertexConstPtr &vertex) const
        {
            return childIdBlacklist_.find(vertex->getId()) != childIdBlacklist_.end();
        }

        bool IGLS::Vertex::hasWhitelistedChild(const VertexConstPtr &vertex) const
        {
            return childIdWhitelist_.find(vertex->getId()) != childIdWhitelist_.end();
        }

        void IGLS::Vertex::clearBlacklist()
        {
            childIdBlacklist_.clear();
        }

        void IGLS::Vertex::clearWhitelist()
        {
            childIdWhitelist_.clear();
        }

        bool IGLS::Vertex::hasEvaluatedChild(const VertexConstPtr &vertex) const
        {
            return (hasBlacklistedChild(vertex) || hasWhitelistedChild(vertex));
        }

        ompl::base::Cost IGLS::Vertex::getCost() const
        {
            ASSERT_NOT_PRUNED

            return cost_;
        }

        ompl::base::Cost IGLS::Vertex::getEdgeInCost() const
        {
            ASSERT_NOT_PRUNED

#ifdef IGLS_DEBUG
            if (!this->hasParent())
            {
                throw ompl::Exception("Attempting to access the incoming-edge cost of a vertex without a parent.");
            }
#endif  // IGLS_DEBUG

            return edgeCost_;
        }

        bool IGLS::Vertex::isPruned() const
        {
            return isPruned_;
        }

        bool IGLS::Vertex::isExpandedOnCurrentApproximation() const
        {
            return expansionApproximationId_ == *currentApproximationId_;
        }

        bool IGLS::Vertex::isExpandedOnCurrentSearch() const
        {
            return expansionSearchId_ == *currentSearchId_;
        }

        bool IGLS::Vertex::hasEverBeenExpandedAsRewiring() const
        {
            return hasEverBeenExpandedAsRewiring_;
        }

        void IGLS::Vertex::registerExpansion()
        {
            // Remember the search and approximation ids.
            expansionApproximationId_ = *currentApproximationId_;
            expansionSearchId_ = *currentSearchId_;
        }

        void IGLS::Vertex::registerRewiringExpansion()
        {
            hasEverBeenExpandedAsRewiring_ = true;
        }

        void IGLS::Vertex::markPruned()
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            isPruned_ = true;
        }

        void IGLS::Vertex::markUnpruned()
        {
            PRINT_VERTEX_CHANGE

            isPruned_ = false;
        }

        void IGLS::Vertex::markInconsistent()
        {
            isConsistent_ = false;
        }

        void IGLS::Vertex::markConsistent()
        {
            isConsistent_ = true;
        }

        bool IGLS::Vertex::isConsistent() const
        {
            return isConsistent_;
        }

        void IGLS::Vertex::setVertexQueueLookup(const SearchQueue::VertexQueueElemPtr &elementPtr)
        {
            vertexQueueLookup_ = elementPtr;
        }

        IGLS::SearchQueue::VertexQueueElemPtr IGLS::Vertex::getVertexQueueLookup() const
        {
            return vertexQueueLookup_;
        }

        void IGLS::Vertex::clearVertexQueueLookup()
        {
            vertexQueueLookup_ = nullptr;

            // TODO(avk): Update the counter?
            // lookupApproximationId_ = *currentApproximationId_;
        }

        void IGLS::Vertex::updateCostAndDepth(bool cascadeUpdates /*= true*/)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            if (this->isRoot())
            {
                // Am I root? -- I don't really know how this would ever be called, but ok.
                cost_ = costHelpPtr_->identityCost();
                depth_ = 0u;
            }
            else if (!this->hasParent())
            {
                // Am I disconnected?
                cost_ = costHelpPtr_->infiniteCost();

                // Set the depth to 0u, getDepth will throw in this condition
                depth_ = 0u;

#ifdef IGLS_DEBUG
                // Assert that I have not been asked to cascade this bad data to my children:
                if (this->hasChildren() && cascadeUpdates)
                {
                    throw ompl::Exception("Attempting to update descendants' costs and depths of a vertex that does "
                                          "not have a parent and is not root. This information would therefore be "
                                          "gibberish.");
                }
#endif  // IGLS_DEBUG
            }
            else
            {
                // I have a parent, so my cost is my parent cost + my edge cost to the parent
                cost_ = costHelpPtr_->combineCosts(parentPtr_->getCost(), edgeCost_);

                // Update the position in the queue if already in the queue.
                if (vertexQueueLookup_)
                {
                    queuePtr_->update(vertexQueueLookup_);
                }

                // I am one more than my parent's depth:
                depth_ = (parentPtr_->getDepth() + 1u);
            }

            // Am I updating my children?
            if (cascadeUpdates)
            {
                // Now, iterate over my vector of children and tell each one to update its own damn cost:
                for (auto &child : children_)
                {
#ifdef IGLS_DEBUG
                    // Check that it hasn't expired
                    if (child.expired())
                    {
                        throw ompl::Exception("A (weak) pointer to a child has was found to have expired while "
                                              "updating the costs and depths of descendant vertices.");
                    }
#endif  // IGLS_DEBUG

                    // Get a lock and tell the child to update:
                    child.lock()->updateCostAndDepth(true);
                }
            }
            // No else, do not update the children. Let's hope the caller knows what they're doing.
        }
    }  // namespace geometric
}  // namespace ompl
