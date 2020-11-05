/* Authors: Aditya Vamsikrishna Mandalika */

#include <fstream>
#include <unordered_set>

#include "ompl/geometric/planners/blaze/Blaze.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            // ============================================================================================
            Blaze::Blaze(const ompl::base::SpaceInformationPtr &si, const std::string &name)
              : ompl::base::Planner(si, name)
            {
                // Do nothing.
            }

            // ============================================================================================
            void Blaze::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
            {
                // Call the base setup and problem definition.
                if (!static_cast<bool>(ompl::base::Planner::setup_))
                {
                    ompl::base::Planner::setup();
                }
                ompl::base::Planner::setProblemDefinition(pdef);

                // Default to path length objective function.
                if (!pdef_->getOptimizationObjective())
                {
                    pdef_->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_));
                }

                // Set the underlying datastructures.
                mImplicitGraph = std::make_shared<ImplicitGraph>(si_, pdef_, this);
                mSearchQueue = std::make_shared<SearchQueue>(mImplicitGraph->getGeneratedSamples());
                mRepairQueue = std::make_shared<RepairQueue>(mImplicitGraph->getGeneratedSamples());
                mEvent->setup(mImplicitGraph);
                mSelector->setup(mImplicitGraph);
            }

            // ============================================================================================
            void Blaze::clear()
            {
                // Clear the datastructures.
                mImplicitGraph->clear();
                mSearchQueue->clear();
                mRepairQueue->clear();

                // Reset the logs. TODO(avk): Any more?
                mBestPathCost = std::numeric_limits<double>::infinity();

                // Call the base functions to clear.
                ompl::base::Planner::clear();
            }

            // ============================================================================================
            ompl::base::PlannerStatus Blaze::solve(const ompl::base::PlannerTerminationCondition &ptc)
            {
                // Check if planner is setup, else setup.
                ompl::base::Planner::checkValidity();

                // Throw an exception if setup failed.
                if (!ompl::base::Planner::setup_)
                {
                    throw ompl::Exception("%s: Planner failed to be setup successfully.",
                                          ompl::base::Planner::getName().c_str());
                }

                // Run the search iterations.
                return iterate(ptc);
            }

            // ============================================================================================
            ompl::base::PlannerStatus Blaze::iterate(const ompl::base::PlannerTerminationCondition &ptc)
            {
                while (ptc == false)
                {
                    // Begin each batch by populating the search queue with start.
                    // Generate required samples for the roadmap batch.
                    mSearchQueue->enqueueVertex(mImplicitGraph->getStartVertex());

                    // Interleave search and evaluation to compute the shortest path.
                    while (ptc == false && !mSearchQueue->isEmpty())
                    {
                        // Search the graph lazily. Search pauses when Event triggers.
                        search();

                        // Evaluate the search tree. Selector is invoked for evaluation.
                        evaluate();

                        // If evaluation results in a solution, publish and sample a new batch.
                        if (mImplicitGraph->solutionFound())
                        {
                            // Solution logged into ompl::base::problemDefinition.
                            mImplicitGraph->publishSolution();

                            // Reset to sample a new batch. TODO(avk): Prune.
                            mImplicitGraph->clearSolution();

                            // TODO(avk): Should all the vertices be added back into the search
                            // queue?
                            break;
                        }
                    }
                    if (!ptc)
                    {
                        mImplicitGraph->generateSamples(mBatchSize);
                    }
                }

                // Termination condition met. Check if we have a solution.
                if (ompl::base::Planner::pdef_->hasExactSolution())
                {
                    // Print to console.
                    OMPL_INFORM("%s: Finished with a solution of cost %.4f.", Planner::getName().c_str(),
                                mImplicitGraph->getUpperCostBound());
                    return ompl::base::PlannerStatus::EXACT_SOLUTION;
                }
                OMPL_INFORM("%s: Failed to compute a solution.", Planner::getName().c_str());
                return ompl::base::PlannerStatus::TIMEOUT;
            }

            // ============================================================================================
            void Blaze::search()
            {
                // Search until the event triggers or the search queue is empty.
                while (!mSearchQueue->isEmpty() && !mEvent->isTriggered(mSearchQueue->getFrontVertex()))
                {
                    // Pop the front vertex from the queue.
                    auto topVertex = mSearchQueue->popFrontVertex();
                    const auto &generatedSamples = mImplicitGraph->getGeneratedSamples();
                    const auto &topVertexPtr = generatedSamples.at(topVertex);

                    // Get neighbors.
                    const VertexIDVector samples = mImplicitGraph->nearestSamples(topVertex);
                    const VertexIDVector vertices = mImplicitGraph->nearestVertices(topVertex);

                    const auto costToCome = topVertexPtr->getCostToCome();
                    for (const auto &sample : samples)
                    {
                        auto &samplePtr = generatedSamples.at(sample);
                        samplePtr->setCostToCome(costToCome + mImplicitGraph->distanceFunction(topVertex, sample));
                        samplePtr->setParent(topVertex);
                        mSearchQueue->enqueueVertex(sample);

                        // Remove from samples and add to vertices since the sample is in the
                        // search tree.
                        mImplicitGraph->removeSample(sample);
                        mImplicitGraph->addVertex(sample);
                    }

                    // TODO(avk): Do the following only if the topVertex is from a new batch.
                    // BIT* marks vertices belonging to previous search tree as old.
                    for (const auto &vertex : vertices)
                    {
                        if (topVertexPtr->hasCollisionNeighbor(vertex))
                        {
                            continue;
                        }

                        auto &vertexPtr = generatedSamples.at(vertex);
                        const double newCostToCome = costToCome + mImplicitGraph->distanceFunction(topVertex, vertex);
                        if (newCostToCome >= vertexPtr->getCostToCome())
                        {
                            continue;
                        }
                        mSearchQueue->dequeueVertex(vertex);
                        vertexPtr->setCostToCome(newCostToCome);
                        vertexPtr->setParent(topVertex);
                        mSearchQueue->enqueueVertex(vertex);
                    }
                }
            }

            // ============================================================================================
            void Blaze::evaluate()
            {
                // TODO(avk): Run evaluation in a loop if greediness has been provided.
                // If the search queue is empty, nothing to do.
                if (mSearchQueue->isEmpty())
                {
                    return;
                }

                // Retreive the path connecting start to top vertex in search queue.
                // NOTE: path begins at the top vertex and goes BACK to the start.
                const auto path = mImplicitGraph->getPathToStart(mSearchQueue->getFrontVertex());

                // Select an edge along this path to evaluate.
                const auto edge = mSelector->selectEdgeToEvaluate(path);

                // If the selector returned an empty edge, solution must have been found.
                if (edge == Edge())
                {
                    mImplicitGraph->setSolutionFound();
                    return;
                }

                // Evaluate the edge. checkMotion() returns true if edge is valid.
                // TODO(avk): We can reduce space by storing with only one of the two.
                const auto &generatedSamples = mImplicitGraph->getGeneratedSamples();
                if (false)
                {  // (!mImplicitGraph->evaluateEdge(edge)) {
                    generatedSamples.at(edge.first)->addCollisionNeighbor(edge.second);
                    generatedSamples.at(edge.second)->addCollisionNeighbor(edge.first);
                    repair(edge.second);
                }
                else
                {
                    generatedSamples.at(edge.first)->addFreeNeighbor(edge.second);
                    generatedSamples.at(edge.second)->addFreeNeighbor(edge.first);
                }
            }

            // ============================================================================================
            void Blaze::repair(const VertexID repairRoot)
            {
                // TODO(avk): Modify this to behave like TLPA*.
                // 1. Collect all the vertices that need to be rewired.
                mRepairQueue->enqueueVertex(repairRoot);
                const auto &generatedSamples = mImplicitGraph->getGeneratedSamples();
                std::vector<VertexID> repairVertices;
                while (!mRepairQueue->isEmpty())
                {
                    const auto topVertex = mRepairQueue->popFrontVertex();
                    const auto &topVertexPtr = generatedSamples.at(topVertex);

                    // Add all the children of the current node to mRepairQueue.
                    const auto children = topVertexPtr->getChildren();
                    for (const auto &child : children)
                    {
                        mRepairQueue->enqueueVertex(child);
                    }
                    generatedSamples.at(topVertex)->removeChildren();

                    // Add the vertex to track.
                    repairVertices.push_back(topVertex);

                    // Remove from search queue.
                    mSearchQueue->dequeueVertex(topVertex);

                    // Assign default values
                    topVertexPtr->removeParent();
                    topVertexPtr->setCostToCome(std::numeric_limits<double>::max());

                    // TODO(avk): Should I update event characteristics here?
                }

                // Collect all of the subtree back into the queue.
                for (auto &vertex : repairVertices)
                {
                    const auto &vertexPtr = generatedSamples.at(vertex);

                    // Try connecting from other close neighboring vertices.
                    auto parents = mImplicitGraph->nearestVertices(vertex);
                    for (const auto &parent : parents)
                    {
                        const auto &parentPtr = generatedSamples.at(parent);
                        // If the parent can trigger the event, avoid connection.
                        // Event is always triggered by the goal, so we skip goal by definition.
                        if (mEvent->isTriggered(parent))
                        {
                            continue;
                        }

                        // If this edge was evaluated as invalid, ignore expansion.
                        if (vertexPtr->hasCollisionNeighbor(parent))
                        {
                            continue;
                        }

                        // If there was no valid path to the parent, ignore it.
                        // This suggests that the parent is also in the repair vertices.
                        if (!parentPtr->hasParent())
                        {
                            continue;
                        }

                        const double potentialCostToCome =
                            parentPtr->getCostToCome() + mImplicitGraph->distanceFunction(vertex, parent);
                        if (potentialCostToCome < vertexPtr->getCostToCome())
                        {
                            vertexPtr->setParent(parent);
                            vertexPtr->setCostToCome(potentialCostToCome);
                        }
                    }

                    // Push the vertex back to the repair queue.
                    mRepairQueue->enqueueVertex(vertex);
                }

                // Finally, process the vertices to see if they can be better parents
                // to other vertices undergoing repair.
                while (!mRepairQueue->isEmpty())
                {
                    auto topVertex = mRepairQueue->popFrontVertex();
                    auto topVertexPtr = generatedSamples.at(topVertex);

                    // If the vertex did not find a parent for itself, ignore processing.
                    if (!topVertexPtr->hasParent())
                    {
                        // This is an expensive operation.
                        mImplicitGraph->removeVertex(topVertex);
                        mImplicitGraph->addSample(topVertex);
                        continue;
                    }
                    // Now that it has found a valid parent, let the parent know.
                    generatedSamples.at(topVertexPtr->getParent())->addChild(topVertex);

                    // If the top vertex triggers event, push to search queue.
                    if (mEvent->isTriggered(topVertex))
                    {
                        mSearchQueue->enqueueVertex(topVertex);
                        continue;
                    }

                    // Expand to neigboring vertices under repair.
                    auto children = mImplicitGraph->nearestVertices(topVertex);
                    for (const auto &child : children)
                    {
                        if (topVertexPtr->hasCollisionNeighbor(child))
                        {
                            continue;
                        }

                        const auto &childPtr = generatedSamples.at(child);
                        const double potentialCostToCome =
                            topVertexPtr->getCostToCome() + mImplicitGraph->distanceFunction(topVertex, child);
                        if (potentialCostToCome < childPtr->getCostToCome())
                        {
                            childPtr->setParent(topVertex);
                            childPtr->setCostToCome(potentialCostToCome);
                        }
                    }
                }
            }

            // ============================================================================================
            void Blaze::getPlannerData(ompl::base::PlannerData &data) const
            {
                // Get the base planner class data.
                Planner::getPlannerData(data);

                // Add the samples (the graph)
                mImplicitGraph->getGraphAsPlannerData(data);

                // Did we find a solution?
                const auto &generatedSamples = mImplicitGraph->getGeneratedSamples();
                if (mImplicitGraph->solutionFound())
                {
                    data.markGoalState(generatedSamples.at(mImplicitGraph->getGoalVertex())->getState());
                }
            }

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl
