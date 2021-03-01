/* Authors: Aditya Mandalika */

#include "ompl/geometric/planners/informedtrees/IGLS.h"

#include <sstream>
#include <iomanip>
#include <memory>
#include <boost/range/adaptor/reversed.hpp>

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/String.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include "ompl/geometric/planners/informedtrees/igls/IdGenerator.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/geometric/planners/informedtrees/igls/CostHelper.h"
#include "ompl/geometric/planners/informedtrees/igls/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/igls/SearchQueue.h"
#include "ompl/geometric/planners/informedtrees/igls/Event.h"
#include "ompl/geometric/planners/informedtrees/igls/ExistenceGraph.h"
#include "ompl/geometric/planners/informedtrees/igls/Selector.h"

#ifdef IGLS_DEBUG
#warning Compiling IGLS with debug-level asserts.
#endif  // IGLS_DEBUG

namespace ompl
{
    namespace geometric
    {
        IGLS::IGLS(const ompl::base::SpaceInformationPtr &si, const std::string &name /*= "IGLS"*/)
          : ompl::base::Planner(si, name)
        {
#ifdef IGLS_DEBUG
            OMPL_WARN("%s: Compiled with debug-level asserts.", Planner::getName().c_str());
#endif  // IGLS_DEBUG

            // Allocate my helper classes, they hold settings and must never be deallocated. Give them a pointer to my
            // name, so they can output helpful error messages
            costHelpPtr_ = std::make_shared<CostHelper>();
            graphPtr_ = std::make_shared<ImplicitGraph>([this]() { return getName(); });
            queuePtr_ = std::make_shared<SearchQueue>([this]() { return getName(); });
            repairQueuePtr_ = std::make_shared<SearchQueue>([this]() { return getName(); });

            // Specify my planner specs:
            Planner::specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
            Planner::specs_.multithreaded = false;

            // Approximate solutions are not supported currently.
            Planner::specs_.approximateSolutions = false;
            Planner::specs_.optimizingPaths = true;
            Planner::specs_.directed = true;
            Planner::specs_.provingSolutionNonExistence = false;
            Planner::specs_.canReportIntermediateSolutions = true;

            // Register my setting callbacks
            Planner::declareParam<double>("rewire_factor", this, &IGLS::setRewireFactor, &IGLS::getRewireFactor,
                                          "1.0:0.01:3.0");
            Planner::declareParam<unsigned int>("samples_per_batch", this, &IGLS::setSamplesPerBatch,
                                                &IGLS::getSamplesPerBatch, "1:1:1000000");
            Planner::declareParam<bool>("use_k_nearest", this, &IGLS::setUseKNearest, &IGLS::getUseKNearest,
                                        "0,"
                                        "1");
            Planner::declareParam<bool>("use_graph_pruning", this, &IGLS::setPruning, &IGLS::getPruning,
                                        "0,"
                                        "1");
            Planner::declareParam<double>("prune_threshold_as_fractional_cost_change", this,
                                          &IGLS::setPruneThresholdFraction, &IGLS::getPruneThresholdFraction,
                                          "0.0:0.01:1.0");
            Planner::declareParam<bool>("stop_on_each_solution_improvement", this, &IGLS::setStopOnSolnImprovement,
                                        &IGLS::getStopOnSolnImprovement, "0,1");

            // Register my progress info:
            addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProgressProperty(); });
            addPlannerProgressProperty("number of segments in solution path INTEGER",
                                       [this] { return bestLengthProgressProperty(); });
            addPlannerProgressProperty("current free states INTEGER", [this] { return currentFreeProgressProperty(); });
            addPlannerProgressProperty("current graph vertices INTEGER",
                                       [this] { return currentVertexProgressProperty(); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this] { return stateCollisionCheckProgressProperty(); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this] { return edgeCollisionCheckProgressProperty(); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this] { return nearestNeighbourProgressProperty(); });

            // Extra progress info that aren't necessary for every day use. Uncomment if desired.
            /*
            addPlannerProgressProperty("vertex queue size INTEGER", [this]
                                       {
                                           return vertexQueueSizeProgressProperty();
                                       });
            addPlannerProgressProperty("iterations INTEGER", [this]
                                       {
                                           return iterationProgressProperty();
                                       });
            addPlannerProgressProperty("batches INTEGER", [this]
                                       {
                                           return batchesProgressProperty();
                                       });
            addPlannerProgressProperty("graph prunings INTEGER", [this]
                                       {
                                           return pruningProgressProperty();
                                       });
            addPlannerProgressProperty("total states generated INTEGER", [this]
                                       {
                                           return totalStatesCreatedProgressProperty();
                                       });
            addPlannerProgressProperty("vertices constructed INTEGER", [this]
                                       {
                                           return verticesConstructedProgressProperty();
                                       });
            addPlannerProgressProperty("states pruned INTEGER", [this]
                                       {
                                           return statesPrunedProgressProperty();
                                       });
            addPlannerProgressProperty("graph vertices disconnected INTEGER", [this]
                                       {
                                           return verticesDisconnectedProgressProperty();
                                       });
            addPlannerProgressProperty("rewiring edges INTEGER", [this]
                                       {
                                           return rewiringProgressProperty();
                                       });
            */
        }

        void IGLS::setup()
        {
            // Call the base class setup. Marks Planner::setup_ as true.
            Planner::setup();

            // Check if we have a problem definition
            if (static_cast<bool>(Planner::pdef_))
            {
                // If no objective is provided, default to path length optimization.
                if (!Planner::pdef_->hasOptimizationObjective())
                {
                    OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
                                Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
                }

                // If the problem definition *has* a goal, make sure it is of appropriate type
                if (static_cast<bool>(Planner::pdef_->getGoal()))
                {
                    if (!Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
                    {
                        OMPL_ERROR("%s::setup() IGLS currently only supports goals that can be cast to a sampleable "
                                   "goal region.",
                                   Planner::getName().c_str());
                        // Mark as not setup:
                        Planner::setup_ = false;
                        return;
                    }
                }

                // Setup the CostHelper, it provides everything I need from optimization objective plus some frills
                costHelpPtr_->setup(Planner::pdef_->getOptimizationObjective(), graphPtr_.get());

                // Setup the queue
                queuePtr_->setup(costHelpPtr_.get(), graphPtr_.get());
                repairQueuePtr_->setup(costHelpPtr_.get(), graphPtr_.get());

                // Setup the existence graph BEFORE the event or selector.
                if (existenceGraphPtr_)
                {
                    existenceGraphPtr_->setup(Planner::si_, costHelpPtr_.get(), queuePtr_.get(), this);
                }

                // Setup the event and the selector. If either is not constructed yet, default.
                if (!eventPtr_)
                {
                    eventPtr_ = std::make_shared<Event>();
                }
                eventPtr_->setup(graphPtr_.get(), existenceGraphPtr_.get());

                if (!selectorPtr_)
                {
                    selectorPtr_ = std::make_shared<Selector>();
                }
                selectorPtr_->setup(existenceGraphPtr_.get());
                // ompl::RNG::setSeed(seed_);

                // Setup the graph, it does not hold a copy of this or Planner::pis_, but uses them to create a
                // NN struct and check for starts/goals, respectively.
                graphPtr_->setup(Planner::si_, Planner::pdef_, costHelpPtr_.get(), queuePtr_.get(), this, Planner::pis_,
                                 existenceGraphPtr_.get());
                graphPtr_->setPruning(isPruningEnabled_);

                // Set the best and pruned costs to the proper objective-based values:
                bestCost_ = costHelpPtr_->infiniteCost();
                prunedCost_ = costHelpPtr_->infiniteCost();

                // Get the measure of the problem
                prunedMeasure_ = Planner::si_->getSpaceMeasure();
            }
            else
            {
                // We don't, so we can't setup. Make sure that is explicit.
                Planner::setup_ = false;
            }
        }

        void IGLS::clear()
        {
            // Clear all the variables.
            // Keep this in the order of the constructors:

            // The various helper classes. DO NOT reset the pointers, they hold configuration parameters:
            costHelpPtr_->reset();
            graphPtr_->reset();
            queuePtr_->reset();
            repairQueuePtr_->reset();

            // Reset the various calculations and convenience containers. Will be recalculated on setup
            bestCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            bestLength_ = 0u;
            prunedCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            prunedMeasure_ = 0.0;
            hasExactSolution_ = false;
            numBatches_ = 0u;
            numPrunings_ = 0u;
            numIterations_ = 0u;
            numEdgeCollisionChecks_ = 0u;
            numRewirings_ = 0u;

            // Mark as not setup:
            Planner::setup_ = false;

            // Call my base clear:
            Planner::clear();
        }

        // ============================================================================================================
        // ============================================================================================================
        ompl::base::PlannerStatus IGLS::solve(const ompl::base::PlannerTerminationCondition &ptc)
        {
            // Check that Planner::setup_ is true, if not call this->setup()
            Planner::checkValidity();
            if (!Planner::setup_)
            {
                throw ompl::Exception("%s::solve() failed to set up the planner. Has a problem definition been set?",
                                      Planner::getName().c_str());
            }
            OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

            // Assuming that the start and goal are provided with pdef and processed in setup().
            // Insert the start vertex into the queue.
            queuePtr_->enqueueVertex(graphPtr_->getStartVertex());

            /**
             * Iterate as long as:
             * We're allowed (ptc == false), AND
             * costHelpPtr_->isSatisfied(bestCost) == false, AND
             * There is a theoretically better solution:
             * (costHelpPtr_->isCostBetterThan(graphPtr_->minCost(), bestCost_) == true)
             */
            startTimer();
            while (!ptc && !costHelpPtr_->isSatisfied(bestCost_) &&
                   (costHelpPtr_->isCostBetterThan(graphPtr_->minCost(), bestCost_)))
            {
                this->iterate();
            }

            // Announce
            if (hasExactSolution_)
            {
                this->endSuccessMessage();
            }
            else
            {
                this->endFailureMessage();
            }

            // Publish
            if (hasExactSolution_)
            {
                this->publishSolution();
                return ompl::base::PlannerStatus::EXACT_SOLUTION;
            }
            // No else, no solution to publish
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        void IGLS::getPlannerData(ompl::base::PlannerData &data) const
        {
            // Get the base planner class data:
            Planner::getPlannerData(data);

            // Add the samples (the graph)
            graphPtr_->getGraphAsPlannerData(data);

            // Did we find a solution?
            if (hasExactSolution_)
            {
                // Exact solution
                data.markGoalState(graphPtr_->getGoalVertex()->state());
            }
            // No else, no solution
        }

        std::vector<std::vector<double>> IGLS::getShortestPaths() const
        {
            return shortestPaths_;
        }

        ompl::base::State const *IGLS::getNextVertexInQueue()
        {
            // The next vertex in the queue.
            ompl::base::State const *nextVertex;

            if (!queuePtr_->isEmpty())
            {
                // The edge in the front of the queue
                VertexConstPtr frontVertex = queuePtr_->getFrontVertex();

                // The next edge in the queue:
                nextVertex = frontVertex->state();
            }
            else
            {
                // An empty edge:
                nextVertex = nullptr;
            }
            return nextVertex;
        }

        ompl::base::Cost IGLS::getNextVertexValueInQueue()
        {
            // The cost of the next edge
            ompl::base::Cost nextCost;
            if (!queuePtr_->isEmpty())
            {
                // The next cost in the queue:
                nextCost = queuePtr_->getFrontVertexValue().at(0u);
            }
            else
            {
                // An infinite cost:
                nextCost = costHelpPtr_->infiniteCost();
            }
            return nextCost;
        }

        void IGLS::getVertexQueue(VertexConstPtrVector *verticesInQueue)
        {
            queuePtr_->getVertices(verticesInQueue);
        }

        unsigned int IGLS::numIterations() const
        {
            return numIterations_;
        }

        ompl::base::Cost IGLS::bestCost() const
        {
            return bestCost_;
        }

        unsigned int IGLS::numBatches() const
        {
            return numBatches_;
        }
        // ============================================================================================================
        // ============================================================================================================
        // Protected functions:

        void IGLS::iterate()
        {
            // Keep track of how many iterations we've performed.
            ++numIterations_;

            // If search is complete at the current approximation.
            // Improve the approximation and continue improving solution.
            if (isSearchDone_ || queuePtr_->isEmpty())
            {
                // Prune the graph if enabled.
                if (isPruningEnabled_)
                {
                    this->prune();
                }

                // Add a new batch.
                this->newBatch();

                // Clear the search queue.
                queuePtr_->clear();

                // Begin search with the start vertex in the queue.
                queuePtr_->enqueueVertex(graphPtr_->getStartVertex());

                // Time to exhaust search on the new approximation.
                isSearchDone_ = false;
            }
            // TODO(avk): Fixing the search depending on whether vertex was previously expanded (old).

            // Interleave lazy seach and edge evaluation until termination condition or
            // search exhausts at the current approximation.
            while (!isSearchDone_)
            {
                // Lazy seach.
                search();

                if (queuePtr_->isEmpty())
                {
                    // Search has exhausted due to the lack of a solution.
                    isSearchDone_ = true;
                    break;
                }

                // Select an edge along the most promising subpath.
                VertexPtrVector reverseSubpath = pathFromVertexToStart(queuePtr_->getFrontVertex());
                VertexPtrPair edge = selectorPtr_->edgeToEvaluate(reverseSubpath);

                // Check if we have computed a new solution.
                // Unless a path to the goal is completely evaluated, selector always returns an edge.
                if (edge == std::pair<VertexPtr, VertexPtr>())
                {
                    // Update the solution and publish to other datastructures.
                    this->registerSolution();

                    // Search has exhausted since we have a solution.
                    isSearchDone_ = true;
                    break;
                }
                evaluate(edge);
            }
        }

        void IGLS::expandToNeighbors(const VertexPtr &vertex, const VertexPtrVector &neighbors)
        {
            for (const auto &neighbor : neighbors)
            {
                // After pruning is enabled,we might want to switch the order here for efficiency?
                // There will be fewer vertices that don't improve the solution after pruning.
                if (!queuePtr_->canPossiblyImproveCurrentSolution(neighbor))
                {
                    continue;
                }
                if (!edgeCanBeConsideredForExpansion(vertex, neighbor))
                {
                    continue;
                }

                const auto edgeCost = costHelpPtr_->trueEdgeCost(vertex, neighbor);
                if (!neighbor->isInTree())
                {
                    // Add a parent to the child.
                    bool outgoingEdgeEvaluated =
                        vertex->hasWhitelistedChild(neighbor) || neighbor->hasWhitelistedChild(vertex);
                    neighbor->addParent(vertex, edgeCost, outgoingEdgeEvaluated);

                    // Add a child to the parent.
                    vertex->addChild(neighbor);

                    // Add the vertex to the set of vertices.
                    graphPtr_->registerAsVertex(neighbor);
                }
                else
                {
                    // If this vertex was previously expanded to vertices, do not repeat operation.
                    // TODO(avk): What if the neighbor was just added to the search tree though?
                    if (vertex->hasEverBeenExpandedToVertices() && !vertex->hasCachedNeighbor(neighbor))
                    {
                        continue;
                    }

                    if (costHelpPtr_->isCostWorseThanOrEquivalentTo(
                            costHelpPtr_->combineCosts(vertex->getCost(), edgeCost), neighbor->getCost()))
                    {
                        // Ignore this neighbor. It has a better parent already.
                        continue;
                    }
                    // vertex is strictly a better parent to neighbor. Rewire.
                    // TODO(avk): This replacement is cascading cost updates to the entire subtree.
                    // Is that what you want?? If not, you might want to remove child.parent == vertex
                    // in the edgeCanBeConsidered() function.
                    this->replaceParent(vertex, neighbor, edgeCost);
                }
                queuePtr_->enqueueVertex(neighbor);
            }
        }

        void IGLS::search()
        {
            // The following should be in a loop until the queuePtr is either empty or event triggers.
            while (!queuePtr_->isEmpty() && !eventPtr_->isTriggered(queuePtr_->getFrontVertex()))
            {
                // Get the most promising vertex.
                VertexPtr vertex = queuePtr_->popFrontVertex();
                assert(vertex->isConsistent());

                // TODO(avk): Why can I not assert that f-value < bestCost_?

                // Expand the vertex to populate the search queue.
                // First process already existing children.
                VertexPtrVector currentChildren;
                vertex->getChildren(&currentChildren);
                for (const auto &child : currentChildren)
                {
                    // Old children that aren't useful anymore need not be considered.
                    if (queuePtr_->canPossiblyImproveCurrentSolution(child))
                    {
                        queuePtr_->enqueueVertex(child);
                    }
                }

                // Now process nearest neighbors from current implicit graph.
                VertexPtrVector neighbors;
                graphPtr_->nearestSamples(vertex, &neighbors);
                expandToNeighbors(vertex, neighbors);

                // Process cached nearest neighbors.
                VertexPtrVector cachedNeighbors;
                vertex->getCachedNeighbors(&cachedNeighbors);
                expandToNeighbors(vertex, cachedNeighbors);

                // Mark that this vertex has been expanded.
                vertex->registerExpansion();
                vertex->registerExpansionToVertices(true);
            }
        }

        void IGLS::evaluate(const VertexPtrPair &edge)
        {
            if (!checkEdge(edge))
            {
                edge.first->blacklistChild(edge.second);
                edge.first->removeChild(edge.second);
                repair(edge.second);
            }
            else
            {
                edge.first->whitelistChild(edge.second);
                edge.second->whitelistChild(edge.first);
                edge.second->updateLazyParametersOnEdgeEvaluation(true);
            }
        }

        void IGLS::resetVertexPropertiesForRepair(const VertexPtr &vertex, VertexPtrVector &inconsistentVertices)
        {
            // Remove from the search queue. This clears its vertex queue lookup.
            // If the vertex is not in the queue, the iterator is null anyway.
            queuePtr_->removeVertexFromQueue(vertex);

            // Clear the parent. This resets the cost to come to infinity.
            // Do not cascade the costs further down. We will do it manually here.
            vertex->removeParent(false, false);

            // Mark the vertex as inconsistent.
            vertex->markInconsistent();

            // Add the vertex to the set of inconsistent vertices.
            inconsistentVertices.push_back(vertex);

            // Do the same for all the children.
            VertexPtrVector children;
            vertex->getChildren(&children);
            for (const auto &child : children)
            {
                resetVertexPropertiesForRepair(child, inconsistentVertices);
            }
            vertex->clearChildren();
        }

        void IGLS::findBestParentForRepair(const VertexPtr &vertex, const VertexPtrVector &potentialParents)
        {
            for (const auto &neighbor : potentialParents)
            {
                // Ignore invalid parents.
                if (!edgeCanBeConsideredForRepair(neighbor, vertex))
                {
                    continue;
                }
                const auto edgeCost = costHelpPtr_->trueEdgeCost(neighbor, vertex);
                if (costHelpPtr_->isCostWorseThanOrEquivalentTo(
                        costHelpPtr_->combineCosts(neighbor->getCost(), edgeCost), vertex->getCost()))
                {
                    // Ignore this neighbor. It has a better parent already.
                    continue;
                }
                // We have a valid neighbor. Rewire.
                bool outgoingEdgeEvaluated =
                    neighbor->hasWhitelistedChild(vertex) || vertex->hasWhitelistedChild(neighbor);
                vertex->addParent(neighbor, edgeCost, outgoingEdgeEvaluated);
            }
        }

        void IGLS::expandToInconsistentNeighbors(const VertexPtr &vertex, const VertexPtrVector &neighbors)
        {
            for (const auto &neighbor : neighbors)
            {
                if (neighbor->isConsistent() || !edgeCanBeConsideredForExpansion(vertex, neighbor))
                {
                    continue;
                }
                const auto edgeCost = costHelpPtr_->trueEdgeCost(vertex, neighbor);
                if (costHelpPtr_->isCostWorseThanOrEquivalentTo(costHelpPtr_->combineCosts(vertex->getCost(), edgeCost),
                                                                neighbor->getCost()))
                {
                    // Ignore this neighbor. It has a better parent already.
                    continue;
                }
                // Vertex is a strictly better parent to neighbor.
                // TODO(avk): I am going to run an expensive operation here for now since addParent()
                // by default modifies the queuePtr_.
                repairQueuePtr_->removeVertexFromQueue(neighbor);
                bool outgoingEdgeEvaluated =
                    vertex->hasWhitelistedChild(neighbor) || neighbor->hasWhitelistedChild(vertex);
                neighbor->addParent(vertex, edgeCost, outgoingEdgeEvaluated);
                repairQueuePtr_->enqueueVertex(neighbor);
            }
        }

        void IGLS::repair(const VertexPtr &root)
        {
            // Recursive lambda might have been cleaner if possible in c++.
            std::vector<VertexPtr> inconsistentVertices;
            resetVertexPropertiesForRepair(root, inconsistentVertices);

            // Find the best parent for each inconsistent vertex.
            for (const auto &vertex : inconsistentVertices)
            {
                // Try to find a parent from the nearest samples.
                VertexPtrVector neighbors;
                graphPtr_->nearestSamples(vertex, &neighbors);
                findBestParentForRepair(vertex, neighbors);

                // Also consider cached neighbors as potential parents.
                VertexPtrVector cachedNeighbors;
                vertex->getCachedNeighbors(&cachedNeighbors);
                findBestParentForRepair(vertex, cachedNeighbors);

                // Queue the vertex for repair.
                repairQueuePtr_->enqueueVertex(vertex);
            }

            numVerticesRewired_ += repairQueuePtr_->numVertices();
            while (!repairQueuePtr_->isEmpty())
            {
                // Get the most promising vertex.
                VertexPtr vertex = repairQueuePtr_->popFrontVertex();

                // Assert that the top vertex in the queue is inconsistent.
                // Whether it has found a parent or not, it is not consistent.
                assert(!vertex->isConsistent());
                vertex->markConsistent();

                // If the vertex has found a parent, give it a family.
                if (vertex->hasParent())
                {
                    // Let the parent know of its new kid.
                    vertex->getParent()->addChild(vertex);

                    // If we can trigger the event, add to search queue.
                    if (!eventPtr_->isTriggered(vertex))
                    {
                        // Check if this vertex can start its own family to nearest neighbors.
                        VertexPtrVector neighbors;
                        graphPtr_->nearestSamples(vertex, &neighbors);
                        expandToInconsistentNeighbors(vertex, neighbors);

                        // Also consider cached neighbors.
                        VertexPtrVector cachedNeighbors;
                        vertex->getCachedNeighbors(&cachedNeighbors);
                        expandToInconsistentNeighbors(vertex, cachedNeighbors);
                    }
                    // Push the vertex to the search queue since it has new values.
                    queuePtr_->enqueueVertex(vertex);
                }
                else
                {
                    // Since the vertex is completely removed from the tree, we should allow expansions to search tree
                    // vertices in the future.
                    vertex->registerExpansionToVertices(false);
                    // TODO(avk): Unregister it from being a vertex.
                    // (Affects the count of number of vertices in the graph.)
                }
            }
            // TODO(avk): Optimization: Stop processing vertices if the topcost here is greater than
            // the topcost in the open list. Make sure to reset the vertex lookups etc though.
        }

        void IGLS::newBatch()
        {
            // Increment the batch counter.
            ++numBatches_;

            // Add a new batch of samples.
            graphPtr_->addNewSamples(samplesPerBatch_);
        }

        void IGLS::prune()
        {
#ifdef IGLS_DEBUG
            if (!isPruningEnabled_)
            {
                throw ompl::Exception("Pruning is not enabled, but prune() is called nonetheless.");
            }
#endif  // IGLS_DEBUG

            // If we don't have an exact solution, we can't prune sensibly.
            if (hasExactSolution_)
            {
                /* Profiling reveals that pruning is very expensive, mainly because the nearest neighbour structure
                 * of the samples has to be updated. On the other hand, nearest neighbour lookup gets more expensive
                 * the bigger the structure, so it's a tradeoff. Pruning on every cost update seems insensible, but
                 * so does never pruning at all. The criteria to prune should depend on how many vertices/samples
                 * there are and how many of them could be pruned, as the decrease in cost associated with nearest
                 * neighbour lookup for fewer samples must justify the cost of pruning. It turns out that counting
                 * is affordable, so we don't need to use any proxy here. */

                // Count the number of samples that could be pruned.
                auto samples = graphPtr_->getCopyOfSamples();
                unsigned int numSamplesThatCouldBePruned(0u);
                for (const auto &sample : samples)
                {
                    if (graphPtr_->canSampleBePruned(sample))
                    {
                        ++numSamplesThatCouldBePruned;
                    }
                }

                // Only prune if the decrease in number of samples and the associated decrease in nearest neighbour
                // lookup cost justifies the cost of pruning. There has to be a way to make this more formal, and
                // less knob-turney, right?
                if (static_cast<float>(numSamplesThatCouldBePruned) /
                        static_cast<float>(graphPtr_->numSamples() + graphPtr_->numVertices()) >=
                    pruneFraction_)
                {
                    // Get the current informed measure of the problem space.
                    double informedMeasure = graphPtr_->getInformedMeasure(bestCost_);

                    // Increment the pruning counter:
                    ++numPrunings_;

                    // Prune the graph.
                    std::pair<unsigned int, unsigned int> numPruned = graphPtr_->prune(informedMeasure);

                    // Store the cost at which we pruned:
                    prunedCost_ = bestCost_;

                    // Also store the measure.
                    prunedMeasure_ = informedMeasure;

                    OMPL_INFORM("%s: Pruning disconnected %d vertices from the tree and completely removed %d "
                                "samples.",
                                Planner::getName().c_str(), numPruned.first, numPruned.second);
                }
            }
            // No else.
        }

        void IGLS::blacklistEdge(const VertexPtrPair &edge) const
        {
            // We store the actual blacklist with the parent vertex for efficient lookup.
            // TODO(avk): Is this enough? What if I am collision checking second->first later?
            // I need a child to parent lookup for efficient repairs.
            edge.first->blacklistChild(edge.second);
        }

        void IGLS::whitelistEdge(const VertexPtrPair &edge) const
        {
            // We store the actual whitelist with the parent vertex for efficient lookup.
            edge.first->whitelistChild(edge.second);
        }

        void IGLS::publishSolution()
        {
            // The reverse path of state pointers
            std::vector<const ompl::base::State *> reversePath;
            // Allocate a path geometric
            auto pathGeoPtr = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);

            // Get the reversed path
            reversePath = this->bestPathFromGoalToStart();

            // Now iterate that vector in reverse, putting the states into the path geometric
            for (const auto &solnState : boost::adaptors::reverse(reversePath))
            {
                pathGeoPtr->append(solnState);
            }

            // Now create the solution
            ompl::base::PlannerSolution soln(pathGeoPtr);
            soln.setPlannerName(Planner::getName());

            // Mark whether the solution met the optimization objective:
            soln.setOptimized(Planner::pdef_->getOptimizationObjective(), bestCost_,
                              Planner::pdef_->getOptimizationObjective()->isSatisfied(bestCost_));

            // Add the solution to the Problem Definition:
            Planner::pdef_->addSolutionPath(soln);
        }

        IGLS::VertexPtrVector IGLS::pathFromVertexToStart(const VertexPtr &vertex) const
        {
            // A vector of states from goal->start:
            VertexPtrVector reversePath;
            VertexPtr curVertex = vertex;

            // Insert the current vertex and iterate backwards.
            reversePath.push_back(curVertex);
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
#ifdef IGLS_DEBUG
                // Check the case where the chain ends incorrectly.
                if (curVertex->hasParent() == false)
                {
                    throw ompl::Exception("The path to the goal does not originate at a start state. Something "
                                          "went "
                                          "wrong.");
                }
#endif  // IGLS_DEBUG

                // Push back the parent into the vector as a state pointer:
                reversePath.push_back(curVertex->getParent());
            }
            return reversePath;
        }

        std::vector<const ompl::base::State *> IGLS::bestPathFromGoalToStart() const
        {
            // A vector of states from goal->start:
            std::vector<const ompl::base::State *> reversePath;
            VertexConstPtr curVertex;

            // Iterate up the chain from the goal, creating a backwards vector:
            if (hasExactSolution_)
            {
                // Start at vertex in the goal
                curVertex = graphPtr_->getGoalVertex();
            }
            else
            {
                throw ompl::Exception("bestPathFromGoalToStart called without an exact solution.");
            }

            // Insert the goal into the path and iterate backwards.
            reversePath.push_back(curVertex->state());
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
#ifdef IGLS_DEBUG
                // Check the case where the chain ends incorrectly.
                if (curVertex->hasParent() == false)
                {
                    throw ompl::Exception("The path to the goal does not originate at a start state. Something "
                                          "went "
                                          "wrong.");
                }
#endif  // IGLS_DEBUG

                // Push back the parent into the vector as a state pointer:
                reversePath.push_back(curVertex->getParent()->state());
            }
            return reversePath;
        }

        bool IGLS::checkEdge(const VertexConstPtrPair &edge)
        {
#ifdef IGLS_DEBUG
            if (edge.first->hasEvalautedChild(edge.second) || edge.second->hasEvaluatedChild(edge.first))
            {
                throw ompl::Exception("An evaluated edge is being collision-checked again!");
            }
#endif  // IGLS_DEBUG
            ++numEdgeCollisionChecks_;
            numStatesCollisionCheckedApproximate_ += static_cast<unsigned int>(
                std::ceil(Planner::si_->getStateSpace()->distance(edge.first->state(), edge.second->state()) /
                          Planner::si_->getStateSpace()->getLongestValidSegmentLength()));
            return Planner::si_->checkMotion(edge.first->state(), edge.second->state());
        }

        void IGLS::replaceParent(const VertexPtr &parent, const VertexPtr &neighbor, const ompl::base::Cost &edgeCost)
        {
#ifdef IGLS_DEBUG
            if (neighbor->getParent()->getId() == parent->getId())
            {
                throw ompl::Exception("The new and old parents of the given rewiring are the same.");
            }
#endif  // IGLS_DEBUG

            // Increment our counter:
            ++numRewirings_;

            // Remove the child from the parent, not updating costs
            neighbor->getParent()->removeChild(neighbor);

            // Remove the parent from the child, not updating costs
            neighbor->removeParent(false, false);

            // Add the parent to the child.
            // TODO(avk): This cascades updates to the entire subtree!
            // Is that necessary and will it ever be the case that this guy has children?
            // Yes, this can happen when a new sample is being rewired to an old vertex.
            bool outgoingEdgeEvaluated = parent->hasWhitelistedChild(neighbor) || neighbor->hasWhitelistedChild(parent);
            neighbor->addParent(parent, edgeCost, outgoingEdgeEvaluated);

            // Add the child to the parent.
            parent->addChild(neighbor);
        }

        bool IGLS::edgeCanBeConsideredForExpansion(const VertexPtr &parent, const VertexPtr &child)
        {
            // If the child is the root, or same as the parent, ignore.
            if (child->isRoot() || child->getId() == parent->getId())
            {
                return false;
            }
            // If the child's parent is already the parent, ignore.
            if (child->isInTree() && child->getParent()->getId() == parent->getId())
            {
                return false;
            }
            // If the child is the parent's parent, avoid this loop!
            if (!parent->isRoot() && child->getId() == parent->getParent()->getId())
            {
                return false;
            }
            // If the edge has been evaluated previously and is in collision, ignore!
            if (parent->hasBlacklistedChild(child) || child->hasBlacklistedChild(parent))
            {
                return false;
            }
            // If the child is a cached neighbor, we should not consider it during expansion.
            // TODO(avk): Remove this check if we are not considering cached neighbors during expansion.
            return true;
        }

        bool IGLS::edgeCanBeConsideredForRepair(const VertexPtr &parent, const VertexPtr &child)
        {
            // Parent needs to be connected to the search tree.
            if (!parent->isInTree())
            {
                return false;
            }
            // Triggering vertices should not be parents (yet).
            if (eventPtr_->isTriggered(parent))
            {
                return false;
            }
            // child cannot parent itself.
            if (parent->getId() == child->getId())
            {
                return false;
            }
            // If the vertex is in the search queue, this repair can happen during the expansion.
            if (parent->getVertexQueueLookup())
            {
                return false;
            }
            // If the parent is inconsistent, no point rewiring to it.
            if (!parent->isConsistent())
            {
                return false;
            }
            // If the edge was evaluated to be in collision, ignore this parent.
            if (parent->hasBlacklistedChild(child) || child->hasBlacklistedChild(parent))
            {
                return false;
            }
            // TODO(avk): Unsure if I should have this optimization.
            // if (parent->hasEverBeenExpandedToVertices() && !parent->hasCachedNeighbor(child))
            // {
            //     return false;
            // }

            // Seems like a valid parent.
            return true;
        }

        void IGLS::registerSolution()
        {
            // We have an exact solution, update tracked costs.
            recordTimer();
            hasExactSolution_ = true;
            // Cache parent as a neighbor for each child.
            this->cacheNeighbors();

            // If the cost has not changed, nothing to brag I suppose.
            if (!costHelpPtr_->isCostBetterThan(graphPtr_->getGoalVertex()->getCost(), bestCost_))
            {
                return;
            }
            bestCost_ = graphPtr_->getGoalVertex()->getCost();
            bestLength_ = graphPtr_->getGoalVertex()->getDepth() + 1u;

            // Tell everyone else about it.
            queuePtr_->registerSolutionCost(bestCost_);
            graphPtr_->registerSolutionCost(bestCost_);

            // Save the cost and the total number of samples.
            std::size_t graphSize = graphPtr_->getCopyOfSamples().size();
            plannerMetrics_.push_back(std::vector<double>{
                (double)graphSize, bestCost_.value(), (double)numEdgeCollisionChecks_,
                (double)numStatesCollisionCheckedApproximate_, (double)numVerticesRewired_, elapsedTime_});

            // Save the shortest path.
            std::vector<double> currentPosition, currentShortestPath;
            VertexPtr curVertex = graphPtr_->getGoalVertex();
            Planner::si_->getStateSpace()->copyToReals(currentPosition, curVertex->state());
            for (const auto &p : currentPosition)
            {
                currentShortestPath.push_back(p);
            }
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
                Planner::si_->getStateSpace()->copyToReals(currentPosition, curVertex->getParent()->state());
                for (const auto &p : currentPosition)
                {
                    currentShortestPath.push_back(p);
                }
            }
            shortestPaths_.push_back(currentShortestPath);

            // Brag:
            this->goalMessage();

            // If enabled, pass the intermediate solution back through the call back:
            if (static_cast<bool>(Planner::pdef_->getIntermediateSolutionCallback()))
            {
                // The form of path passed to the intermediate solution callback is not well documented, but it
                // *appears* that it's not supposed
                // to include the start or goal; however, that makes no sense for multiple start/goal problems, so
                // we're going to include it anyway (sorry).
                // Similarly, it appears to be ordered as (goal, goal-1, goal-2,...start+1, start) which
                // conveniently allows us to reuse code.
                Planner::pdef_->getIntermediateSolutionCallback()(this, this->bestPathFromGoalToStart(), bestCost_);
            }
        }

        void IGLS::cacheNeighbors()
        {
            VertexPtr curVertex = graphPtr_->getGoalVertex();
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
                if (!curVertex->hasCachedNeighbor(curVertex->getParent()))
                {
                    curVertex->cacheNeighbor(curVertex->getParent());
                    curVertex->getParent()->cacheNeighbor(curVertex);
                }
            }
        }

        // ============================================================================================================
        // ============================================================================================================
        // Messages to the user/console.
        void IGLS::goalMessage() const
        {
            OMPL_INFORM("%s (%u iters): Found a solution of cost %.4f (%u vertices) from %u samples by processing "
                        "%u "
                        "vertices, collision checking %u edges and perform %u rewirings. The graph "
                        "currently has %u vertices and a connection radius of %.4f.",
                        Planner::getName().c_str(), numIterations_, bestCost_.value(), bestLength_,
                        graphPtr_->numSamples(), queuePtr_->numVerticesPopped(), numEdgeCollisionChecks_,
                        numVerticesRewired_, graphPtr_->numVertices(), graphPtr_->getConnectivityR());
        }

        void IGLS::endSuccessMessage() const
        {
            OMPL_INFORM("%s (%u iters): Finished with a cost %.4f (%u vertices) from %u samples by processing %u "
                        "vertices, collision checking %u edges and perform %u rewirings. The final graph "
                        "has %u vertices.",
                        Planner::getName().c_str(), numIterations_, bestCost_.value(), bestLength_,
                        graphPtr_->numSamples(), queuePtr_->numVerticesPopped(), numEdgeCollisionChecks_,
                        numVerticesRewired_, graphPtr_->numVertices());
        }

        void IGLS::endFailureMessage() const
        {
            OMPL_INFORM("%s (%u iters): Did not find an exact solution from %u samples by processing %u "
                        "vertices, collision checking %u edges and perform %u rewirings. The final graph "
                        "has %u vertices.",
                        Planner::getName().c_str(), numIterations_, graphPtr_->numSamples(),
                        queuePtr_->numVerticesPopped(), numEdgeCollisionChecks_, numVerticesRewired_,
                        graphPtr_->numVertices());
        }

        void IGLS::statusMessage(const ompl::msg::LogLevel &logLevel, const std::string &status) const
        {
            // Check if we need to create the message
            if (logLevel >= ompl::msg::getLogLevel())
            {
                // Variable
                // The message as a stream:
                std::stringstream outputStream;

                // Create the stream:
                // The name of the planner
                outputStream << Planner::getName();
                outputStream << " (";
                // The current path cost:
                outputStream << "l: " << std::setw(6) << std::setfill(' ') << std::setprecision(5) << bestCost_.value();
                // The number of batches:
                outputStream << ", b: " << std::setw(5) << std::setfill(' ') << numBatches_;
                // The number of iterations
                outputStream << ", i: " << std::setw(5) << std::setfill(' ') << numIterations_;
                // The number of states current in the graph
                outputStream << ", g: " << std::setw(5) << std::setfill(' ') << graphPtr_->numVertices();
                // The number of free states
                outputStream << ", f: " << std::setw(5) << std::setfill(' ') << graphPtr_->numSamples();
                // The number vertices in the queue:
                outputStream << ", q: " << std::setw(5) << std::setfill(' ') << queuePtr_->numVertices();
                // The total number of vertices taken out of the queue:
                outputStream << ", t: " << std::setw(5) << std::setfill(' ') << queuePtr_->numVerticesPopped();
                // The number of samples generated
                outputStream << ", s: " << std::setw(5) << std::setfill(' ') << graphPtr_->numStatesGenerated();
                // The number of vertices ever added to the graph:
                outputStream << ", v: " << std::setw(5) << std::setfill(' ') << graphPtr_->numVerticesConnected();
                // The number of prunings:
                outputStream << ", p: " << std::setw(5) << std::setfill(' ') << numPrunings_;
                // The number of rewirings:
                outputStream << ", r: " << std::setw(5) << std::setfill(' ') << numRewirings_;
                // The number of nearest-neighbour calls
                outputStream << ", n: " << std::setw(5) << std::setfill(' ') << graphPtr_->numNearestLookups();
                // The number of state collision checks:
                outputStream << ", c(s): " << std::setw(5) << std::setfill(' ') << graphPtr_->numStateCollisionChecks();
                // The number of edge collision checks:
                outputStream << ", c(e): " << std::setw(5) << std::setfill(' ') << numEdgeCollisionChecks_;
                outputStream << "):    ";
                // The message:
                outputStream << status;

                if (logLevel == ompl::msg::LOG_DEBUG)
                {
                    OMPL_DEBUG("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_INFO)
                {
                    OMPL_INFORM("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_WARN)
                {
                    OMPL_WARN("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_ERROR)
                {
                    OMPL_ERROR("%s: ", outputStream.str().c_str());
                }
                else
                {
                    throw ompl::Exception("Log level not recognized");
                }
            }
            // No else, this message is below the log level
        }

        // ============================================================================================================
        // ============================================================================================================
        // Setters/Getters (Public) and progress properties (Protected):
        void IGLS::enableCascadingRewirings(bool enable)
        {
            queuePtr_->enableCascadingRewirings(enable);
        }

        void IGLS::setRewireFactor(double rewireFactor)
        {
            graphPtr_->setRewireFactor(rewireFactor);
        }

        double IGLS::getRewireFactor() const
        {
            return graphPtr_->getRewireFactor();
        }

        void IGLS::setSamplesPerBatch(unsigned int samplesPerBatch)
        {
            samplesPerBatch_ = samplesPerBatch;
        }

        unsigned int IGLS::getSamplesPerBatch() const
        {
            return samplesPerBatch_;
        }

        void IGLS::setUseKNearest(bool useKNearest)
        {
            // Store
            graphPtr_->setUseKNearest(useKNearest);

            // If the planner is default named, we change it:
            if (!graphPtr_->getUseKNearest() && Planner::getName() == "kIGLS")
            {
                // It's current the default k-nearest IGLS name, and we're toggling, so set to the default r-disc
                OMPL_WARN("IGLS: An r-disc version of IGLS can not be named 'kBITstar', as this name is reserved "
                          "for "
                          "the k-nearest version. Changing the name to 'IGLS'.");
                Planner::setName("IGLS");
            }
            else if (graphPtr_->getUseKNearest() && Planner::getName() == "IGLS")
            {
                // It's current the default r-disc IGLS name, and we're toggling, so set to the default k-nearest
                OMPL_WARN("IGLS: A k-nearest version of IGLS can not be named 'IGLS', as this name is reserved for "
                          "the r-disc version. Changing the name to 'kBITstar'.");
                Planner::setName("kBITstar");
            }
            // It's not default named, don't change it
        }

        bool IGLS::getUseKNearest() const
        {
            return graphPtr_->getUseKNearest();
        }

        void IGLS::setPruning(bool usePruning)
        {
            isPruningEnabled_ = usePruning;
            graphPtr_->setPruning(usePruning);
        }

        bool IGLS::getPruning() const
        {
            return isPruningEnabled_;
        }

        void IGLS::setPruneThresholdFraction(double fractionalChange)
        {
            if (fractionalChange < 0.0 || fractionalChange > 1.0)
            {
                throw ompl::Exception("Prune threshold must be specified as a fraction between [0, 1].");
            }

            pruneFraction_ = fractionalChange;
        }

        double IGLS::getPruneThresholdFraction() const
        {
            return pruneFraction_;
        }

        void IGLS::setStopOnSolnImprovement(bool stopOnChange)
        {
            stopOnSolutionChange_ = stopOnChange;
        }

        bool IGLS::getStopOnSolnImprovement() const
        {
            return stopOnSolutionChange_;
        }

        void IGLS::setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number)
        {
            graphPtr_->setAverageNumOfAllowedFailedAttemptsWhenSampling(number);
        }

        std::size_t IGLS::getAverageNumOfAllowedFailedAttemptsWhenSampling() const
        {
            return graphPtr_->getAverageNumOfAllowedFailedAttemptsWhenSampling();
        }

        template <template <typename T> class NN>
        void IGLS::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (Planner::setup_)
            {
                OMPL_WARN("%s: The nearest neighbour datastructures cannot be changed once the planner is setup. "
                          "Continuing to use the existing containers.",
                          Planner::getName().c_str());
            }
            else
            {
                graphPtr_->setNearestNeighbors<NN>();
            }
        }

        void IGLS::useShortestPathEvent()
        {
            eventPtr_ = std::make_shared<Event>();
        }

        void IGLS::useConstantDepthEvent(const std::size_t depth)
        {
            eventPtr_ = std::make_shared<ConstantDepthEvent>(depth);
        }

        void IGLS::useSubpathExistenceEvent(const double threshold)
        {
            eventPtr_ = std::make_shared<SubpathExistenceEvent>(threshold);
        }

        void IGLS::useForwardSelector()
        {
            selectorPtr_ = std::make_shared<Selector>();
        }

        void IGLS::useFailfastSelector()
        {
            selectorPtr_ = std::make_shared<FailfastSelector>();
        }

        std::string IGLS::bestCostProgressProperty() const
        {
            return ompl::toString(this->bestCost().value());
        }

        std::string IGLS::bestLengthProgressProperty() const
        {
            return std::to_string(bestLength_);
        }

        std::string IGLS::currentFreeProgressProperty() const
        {
            return std::to_string(graphPtr_->numSamples());
        }

        std::string IGLS::currentVertexProgressProperty() const
        {
            return std::to_string(graphPtr_->numVertices());
        }

        std::string IGLS::vertexQueueSizeProgressProperty() const
        {
            return std::to_string(queuePtr_->numVertices());
        }

        std::string IGLS::iterationProgressProperty() const
        {
            return std::to_string(this->numIterations());
        }

        std::string IGLS::batchesProgressProperty() const
        {
            return std::to_string(this->numBatches());
        }

        std::string IGLS::pruningProgressProperty() const
        {
            return std::to_string(numPrunings_);
        }

        std::string IGLS::totalStatesCreatedProgressProperty() const
        {
            return std::to_string(graphPtr_->numStatesGenerated());
        }

        std::string IGLS::verticesConstructedProgressProperty() const
        {
            return std::to_string(graphPtr_->numVerticesConnected());
        }

        std::string IGLS::statesPrunedProgressProperty() const
        {
            return std::to_string(graphPtr_->numFreeStatesPruned());
        }

        std::string IGLS::verticesDisconnectedProgressProperty() const
        {
            return std::to_string(graphPtr_->numVerticesDisconnected());
        }

        std::string IGLS::rewiringProgressProperty() const
        {
            return std::to_string(numRewirings_);
        }

        std::string IGLS::stateCollisionCheckProgressProperty() const
        {
            return std::to_string(graphPtr_->numStateCollisionChecks());
        }

        std::string IGLS::edgeCollisionCheckProgressProperty() const
        {
            return std::to_string(numEdgeCollisionChecks_);
        }

        std::string IGLS::vertexRewireProgressProperty() const
        {
            return std::to_string(numVerticesRewired_);
        }

        std::string IGLS::nearestNeighbourProgressProperty() const
        {
            return std::to_string(graphPtr_->numNearestLookups());
        }

        std::string IGLS::verticesProcessedProgressProperty() const
        {
            return std::to_string(queuePtr_->numVerticesPopped());
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        void IGLS::generateSamplesCostLog() const
        {
            std::size_t graphSize = graphPtr_->getCopyOfSamples().size();

            std::ofstream logfile;
            assert(hasExactSolution_);
            std::string solutionDataFile = "IGLS_Metrics.txt";
            logfile.open(solutionDataFile, std::ios_base::app);
            logfile << graphPtr_->getCopyOfSamples().size() << " " << numEdgeCollisionChecks_ << " " << bestCost_
                    << std::endl;
            logfile.close();
        }

        void IGLS::printGraph() const
        {
            auto samples = graphPtr_->getCopyOfSamples();
            assert(hasExactSolution_);
            for (const auto &sample : samples)
            {
                if (sample->hasParent())
                {
                    std::cout << sample->getId() << " " << sample->getParent()->getId() << std::endl;
                }
            }
        }

        void IGLS::printCompleteGraph() const
        {
            auto samples = graphPtr_->getCopyOfSamples();

            std::ofstream logfile;
            std::string datafile = "IGLS_Graph.txt";
            std::remove(datafile.c_str());
            logfile.open(datafile, std::ios_base::app);
            logfile << "--------------------------------------" << std::endl;
            std::vector<double> source, target;
            for (const auto &sample : samples)
            {
                Planner::si_->getStateSpace()->copyToReals(source, sample->state());
                VertexPtrVector neighbors;
                graphPtr_->nearestSamples(sample, &neighbors);
                for (const auto &neighbor : neighbors)
                {
                    Planner::si_->getStateSpace()->copyToReals(target, neighbor->state());
                    logfile << source[0] << " " << source[1] << " " << target[0] << " " << target[1] << std::endl;
                }
            }
            logfile.close();
        }

        std::vector<std::vector<double>> IGLS::getPlannerMetrics() const
        {
            return plannerMetrics_;
        }

        void IGLS::setExistenceGraph(const std::string &datasetPath, double edgeDiscretization, double obstacleDensity)
        {
            existenceGraphPtr_ = std::make_shared<ExistenceGraph>(datasetPath, edgeDiscretization, obstacleDensity);
        }
        void IGLS::setSeed(int seed)
        {
            seed_ = seed;
        }
    }  // namespace geometric
}  // namespace ompl
