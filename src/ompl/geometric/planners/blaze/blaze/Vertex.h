/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXH_

#include <assert.h>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/planners/blaze/blaze/VertexIDGenerator.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            namespace
            {
                // Global variables:
                // The initialization flag stating that the ID generator has been created:
                std::once_flag kGeneratorInitiated;

                // A pointer to the actual ID generator.
                std::unique_ptr<VertexIDGenerator> kIDGenerator;

                // A function to initialize the ID generator pointer:
                void initIDGenerator()
                {
                    // Reset the unique pointer with a new instance.
                    kIDGenerator.reset(new VertexIDGenerator());
                }

                VertexIDGenerator &getIDGenerator()
                {
                    // Register a call and return the reference to generator.
                    std::call_once(kGeneratorInitiated, &initIDGenerator);
                    return *kIDGenerator;
                }

            }  // namespace

            // Aliases for simplifying code.
            using VertexIDVector = std::vector<VertexID>;
            using Edge = std::pair<VertexID, VertexID>;

            ///
            /// \brief A class to store a state as a vertex in a graph. Allocates and frees
            /// memory for the underlying state.
            ///
            class Vertex
            {
            public:
                /// \brief Constructor
                Vertex(ompl::base::SpaceInformationPtr si, const ompl::base::State *state)
                  : mSpaceInformation(si)
                  , mID(getIDGenerator().getNewID())
                  , mState(mSpaceInformation->cloneState(state))
                {
                    // Do nothing.
                }

                explicit Vertex(ompl::base::SpaceInformationPtr si)
                  : mSpaceInformation(si), mID(getIDGenerator().getNewID())
                {
                    // Do nothing.
                }

                /// \brief Destructor
                ~Vertex()
                {
                    // Free the state on destruction
                    if (mState)
                    {
                        mSpaceInformation->freeState(mState);
                    }
                }

                /// \brief Get the unique ID of the vertex.
                VertexID getID() const
                {
                    return mID;
                }

                /// \brief Return the state associated with the vertex.
                ompl::base::State *getState() const
                {
                    return mState;
                }

                /// \brief Sets the state associated with the vertex.
                void setState(ompl::base::State *state)
                {
                    mState = mSpaceInformation->cloneState(state);
                }

                /// \brief Sets the parent to the vertex.
                void setParent(const VertexID id)
                {
                    mParent = id;
                    mInSearchTree = true;
                }
                void setParent(const std::shared_ptr<Vertex> &parent)
                {
                    setParent(parent->getID());
                }

                /// \brief Returns the parent to the vertex.
                VertexID getParent() const
                {
                    return mParent;
                }

                /// \brief Remove parent.
                void removeParent()
                {
                    mParent = -1;
                    mInSearchTree = false;
                }

                /// \brief Returns true if vertex has parent.
                bool hasParent() const
                {
                    return (mParent != -1);
                }

                /// \brief Adds child to the vertex.
                void addChild(const VertexID child)
                {
                    mChildren.push_back(child);
                }
                void addChild(const std::shared_ptr<Vertex> &child)
                {
                    mChildren.push_back(child->getID());
                }

                /// \brief Removes a child.
                void removeChild(const VertexID child)
                {
                    for (auto iterV = mChildren.begin(); iterV != mChildren.end(); ++iterV)
                    {
                        if (*iterV == child)
                        {
                            *iterV = mChildren.back();
                            mChildren.pop_back();
                            return;
                        }
                    }
                }
                void removeChild(const std::shared_ptr<Vertex> &child)
                {
                    removeChild(child->getID());
                }

                /// \brief Returns true if it has the child.
                bool hasChild(const VertexID child) const
                {
                    for (auto iterV = mChildren.begin(); iterV != mChildren.end(); ++iterV)
                    {
                        if (*iterV == child)
                        {
                            return true;
                        }
                    }
                    return false;
                }
                bool hasChild(const std::shared_ptr<Vertex> &child)
                {
                    return hasChild(child->getID());
                }

                /// \brief Retreives the children.
                std::vector<VertexID> getChildren() const
                {
                    return mChildren;
                }

                /// \brief Returns true if it has any children.
                bool hasChildren() const
                {
                    return !mChildren.empty();
                }

                /// \brief Clears the children.
                void removeChildren()
                {
                    mChildren.clear();
                }

                /// \brief The cost-to-come of the vertex.
                double getCostToCome() const
                {
                    return mCostToCome;
                }

                /// \brief The heuristic associated with the vertex.
                double getCostToComeHeuristic() const
                {
                    return mCostToComeHeuristic;
                }

                /// \brief The heuristic associated with the vertex.
                double getCostToGoHeuristic() const
                {
                    return mCostToGoHeuristic;
                }

                /// \brief Set the cost-to-come of the vertex.
                void setCostToCome(const double cost)
                {
                    mCostToCome = cost;
                }

                /// \brief Set the cost-to-come heuristic associated with the vertex.
                /// Useful for pruning in subsequent batches.
                void setCostToComeHeuristic(const double cost)
                {
                    mCostToComeHeuristic = cost;
                }

                /// \brief Set the heuristic associated with the vertex.
                /// Useful for pruning in subsequent batches and in search.
                void setCostToGoHeuristic(const double cost)
                {
                    mCostToGoHeuristic = cost;
                }

                /// \brief The total cost associated with the vertex.
                double getTotalCost() const
                {
                    return mCostToCome + mCostToGoHeuristic;
                }

                /// \brief Mark the vertex to have been expanded.
                void markAsExpanded()
                {
                    mExpanded = true;
                }

                /// \brief Returns true if the vertex has been expanded.
                bool expanded() const
                {
                    return mExpanded;
                }

                /// \brief Returns true if the vertex is in the search tree.
                bool inSearchTree() const
                {
                    return mInSearchTree;
                }

                /// \brief Adds a neighbor to which edge is in collision.
                void addCollisionNeighbor(const VertexID vertex)
                {
                    assert(hasEvaluatedNeighbor(vertex) == false);
                    mCollisionNeighbors.push_back(vertex);
                }

                /// \brief Checks if edge to vertex is in collision.
                bool hasCollisionNeighbor(const VertexID vertex) const
                {
                    for (const auto &v : mCollisionNeighbors)
                    {
                        if (v == vertex)
                        {
                            return true;
                        }
                    }
                    return false;
                }

                /// \brief Adds a neighbor to which edge is collision-free.
                void addFreeNeighbor(const VertexID vertex)
                {
                    assert(hasEvaluatedNeighbor(vertex) == false);
                    mFreeNeighbors.push_back(vertex);
                }

                /// Checks if edge to neighbor is collision-free.
                bool hasFreeNeighbor(const VertexID vertex) const
                {
                    for (const auto &v : mFreeNeighbors)
                    {
                        if (v == vertex)
                        {
                            return true;
                        }
                    }
                    return false;
                }

                /// \brief Returns true if edge to \c vertex has been evaluated.
                // TODO(avk): Possibility for efficient processing here if we call both
                // hasCollision+hasEvaluated in the same function.
                bool hasEvaluatedNeighbor(const VertexID vertex) const
                {
                    return hasCollisionNeighbor(vertex) || hasFreeNeighbor(vertex);
                }

            private:
                /// Space Information.
                ompl::base::SpaceInformationPtr mSpaceInformation;

                /// Vertex ID.
                const VertexID mID;

                /// Associated state
                ompl::base::State *mState{nullptr};

                /// Indicator for expansion.
                bool mExpanded{false};

                /// Indicator for vertex in search tree.
                bool mInSearchTree{false};

                /// Parent.
                VertexID mParent{-1};

                /// Children.
                std::vector<VertexID> mChildren;

                /// Cost-to-come.
                double mCostToCome{std::numeric_limits<double>::infinity()};

                /// Cost to come heuristic.
                double mCostToComeHeuristic{std::numeric_limits<double>::infinity()};

                /// Cost to go heuristic.
                double mCostToGoHeuristic{std::numeric_limits<double>::infinity()};

                /// List of evaluated neighbors.
                VertexIDVector mCollisionNeighbors;
                VertexIDVector mFreeNeighbors;
            };

            // Type aliases to simplify code.
            using VertexPtr = std::shared_ptr<Vertex>;
            using VertexUPtr = std::unique_ptr<Vertex>;

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXH_
