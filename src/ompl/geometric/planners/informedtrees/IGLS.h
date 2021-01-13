/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_

#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include "ompl/base/Planner.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/NearestNeighbors.h"

// Defining IGLS_DEBUG enables (significant) debug output. Do not enable unless necessary.
// #define IGLS_DEBUG

namespace ompl
{
    namespace geometric
    {
        /** \brief Generalized Batch-Informed Lazy Search Trees */
        class IGLS : public ompl::base::Planner
        {
        public:
            // ---
            // Forward declarations.
            // ---
            /** \brief The vertex of implicit and explicit graphs. */
            class Vertex;
            /** \brief A generator of unique vertex IDs. */
            class IdGenerator;
            /** \brief A helper class to consolidate cost and heuristic calculations. */
            class CostHelper;
            /** \brief The samples viewed as an edge-implicit random geometric graph. */
            class ImplicitGraph;
            /** \brief The queue of vertices to process for expansion. */
            class SearchQueue;
            /** \brief The event defining the interleaving of search and evaluation. */
            class Event;
            class ConstantDepthEvent;
            /** \brief The selector defining the strategy to choose edges to evaluate. */
            class Selector;

            // ---
            // Aliases.
            // ---

            /** \brief A shared pointer to a vertex. */
            using VertexPtr = std::shared_ptr<Vertex>;

            /** \brief A weak pointer to a vertex. */
            using VertexWeakPtr = std::weak_ptr<Vertex>;

            /** \brief A shared pointer to a \e const vertex. */
            using VertexConstPtr = std::shared_ptr<const Vertex>;

            /** \brief A vector of shared pointers to vertices. */
            using VertexPtrVector = std::vector<VertexPtr>;

            /** \brief A vector of shared pointers to \e const vertices. */
            using VertexConstPtrVector = std::vector<VertexConstPtr>;

            /** \brief The vertex id type. */
            using VertexId = unsigned int;

            /** \brief A pair of vertices, i.e., an edge. */
            using VertexPtrPair = std::pair<VertexPtr, VertexPtr>;

            /** \brief A pair of const vertices, i.e., an edge. */
            using VertexConstPtrPair = std::pair<VertexConstPtr, VertexConstPtr>;

            /** \brief A vector of pairs of vertices, i.e., a vector of edges. */
            using VertexPtrPairVector = std::vector<VertexPtrPair>;

            /** \brief A vector of pairs of const vertices, i.e., a vector of edges. */
            using VertexConstPtrPairVector = std::vector<VertexConstPtrPair>;

            /** \brief The OMPL::NearestNeighbors structure. */
            using VertexPtrNNPtr = std::shared_ptr<NearestNeighbors<VertexPtr>>;

            /** \brief A utility functor for ImplicitGraph and SearchQueue. */
            using NameFunc = std::function<std::string()>;

            /** \brief Construct with a pointer to the space information and an optional name. */
            IGLS(const base::SpaceInformationPtr &si, const std::string &name = "IGLS");

            /** \brief Destruct using the default destructor. */
            virtual ~IGLS() override = default;

            /** \brief Setup the algorithm. */
            void setup() override;

            /** \brief Clear the algorithm's internal state. */
            void clear() override;

            /** \brief Solve the problem given a termination condition. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get results. */
            void getPlannerData(base::PlannerData &data) const override;

            // ---
            // Debugging info.
            // ---

            /** \brief Get the next vertex to expand helpful for some videos and debugging. */
            const ompl::base::State *getNextVertexInQueue();

            /** \brief Get the value of the next vertex to be processed. */
            ompl::base::Cost getNextVertexValueInQueue();

            /** \brief Get the whole messy set of edges in the queue. Expensive but helpful for some videos. */
            void getVertexQueue(VertexConstPtrVector *verticesInQueue);

            /** \brief Get the number of iterations completed. */
            unsigned int numIterations() const;

            /** \brief Retrieve the best exact-solution cost found.*/
            ompl::base::Cost bestCost() const;

            /** \brief Retrieve the number of batches processed as the raw data. */
            unsigned int numBatches() const;

            // ---
            // Settings.
            // ---

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg*. */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Set the average number of allowed failed attempts when sampling. */
            void setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number);

            /** \brief Get the average number of allowed failed attempts when sampling. */
            std::size_t getAverageNumOfAllowedFailedAttemptsWhenSampling() const;

            /** \brief Set the number of samplers per batch. */
            void setSamplesPerBatch(unsigned int n);

            /** \brief Get the number of samplers per batch. */
            unsigned int getSamplesPerBatch() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getUseKNearest() const;

            /** \brief Enable pruning of vertices/samples that CANNOT improve the current solution. When a vertex in the
             * graph is pruned, it's descendents are also pruned (if they also cannot improve the solution) or placed
             * back in the set of free samples (if they could improve the solution). This assures that a uniform density
             * is maintained. */
            void setPruning(bool prune);

            /** \brief Get whether graph and sample pruning is in use. */
            bool getPruning() const;

            /** \brief Set the fractional change in the solution cost AND problem measure necessary for pruning to
             * occur. */
            void setPruneThresholdFraction(double fractionalChange);

            /** \brief Get the fractional change in the solution cost AND problem measure necessary for pruning to
             * occur. */
            double getPruneThresholdFraction() const;

            /** \brief Stop the planner each time a solution improvement is found. Useful
            for examining the intermediate solutions found by IGLS. */
            void setStopOnSolnImprovement(bool stopOnChange);

            /** \brief Get whether IGLS stops each time a solution is found. */
            bool getStopOnSolnImprovement() const;

            /** \brief Set a different nearest neighbours datastructure. */
            template <template <typename T> class NN>
            void setNearestNeighbors();

        protected:
            /** \brief Enable the cascading of rewirings. */
            // TODO(avk) Is this ABIT* specific?
            void enableCascadingRewirings(bool enable);

        private:
            // ---
            // High level primitives.
            // ---

            /** \brief A single iteration. */
            void iterate();

            /** \brief Lazy search until the event triggers */
            void search();

            /** \brief Evaluate the most promising subpath */
            void evaluate(const VertexPtrPair &edge);

            /** \brief Repairs the search tree. */
            void repair(const VertexPtr &root);

            /** \brief Initialize variables for a new batch. */
            void newBatch();

            /** \brief Prune the problem. */
            void prune();

            /** \brief Publish the found solution to the ProblemDefinition. */
            void publishSolution();

            // ---
            // Low level primitives.
            // ---

            /** \brief Extract the subpath connecting the start to the given vertex. The subpath is returned in
             * reverse. */
            VertexPtrVector pathFromVertexToStart(const VertexPtr &vertex) const;

            /** \brief Extract the best solution, ordered \e from the goal to the \e start and including both the goal
             * and the start. Used by both publishSolution and the ProblemDefinition::IntermediateSolutionCallback. */
            std::vector<const ompl::base::State *> bestPathFromGoalToStart() const;

            /** \brief Checks an edge for collision. */
            bool checkEdge(const VertexConstPtrPair &edge);

            /** \brief Blacklists an edge (useful if an edge is in collision). */
            void blacklistEdge(const VertexPtrPair &edge) const;

            /** \brief Whitelists an edge (useful if an edge not in collision). */
            void whitelistEdge(const VertexPtrPair &edge) const;

            /** \brief Replace the parent edge with the given new edge and cost */
            void replaceParent(const VertexPtr &parent, const VertexPtr &neighbor, const ompl::base::Cost &edgeCost);

            /** \brief Checks for a series of conditions for if this edge can be used for expansion. */
            bool edgeCanBeConsideredForExpansion(const VertexPtr &parent, const VertexPtr &neighbor);

            /** \brief Checks for a series of conditions for if this edge can be used to rewire. */
            bool edgeCanBeConsideredForRepair(const VertexPtr &parent, const VertexPtr &child);

            /** \brief Resets properties of vertex and its subtree, and tracks them in inconsistentVertices. */
            void resetVertexPropertiesForRepair(const VertexPtr &vertex, VertexPtrVector &inconsistentVertices);

            /** \brief Sets the parent property of a vertex by considering all valid neighbors during repair. */
            void findBestParentForRepair(const VertexPtr &vertex);

            /** \brief Sets the parent property of a vertex by considering all valid neighbors during repair. */
            void expandToInconsistentNeighbors(const VertexPtr &vertex);

            /** \brief Caches the shortest path in an iteration as vertex neighbors. */
            void cacheNeighbors();

            /** \brief The special work that needs to be done when a collision-free path has been computed. */
            void registerSolution();

            // ---
            // Logging.
            // ---

            // Helper functions for logging
            /** \brief The message printed when a goal is found/improved. */
            void goalMessage() const;

            /** \brief The message printed when solve finishes successfully. */
            void endSuccessMessage() const;

            /** \brief The message printed when solve finishes unsuccessfully. */
            void endFailureMessage() const;

            /** \brief A detailed status message format with debug info. */
            void statusMessage(const ompl::msg::LogLevel &logLevel, const std::string &status) const;

            // ---
            // Progress properties.
            // ---

            /** \brief Retrieve the best exact-solution cost found as a planner-progress property. */
            std::string bestCostProgressProperty() const;

            /** \brief Retrieve the length of the best exact-solution found as a planner-progress property.
             * (bestLength_) */
            std::string bestLengthProgressProperty() const;

            /** \brief Retrieve the current number of free samples as a planner-progress property. (Size of the free
             * states in graphPtr_.) */
            std::string currentFreeProgressProperty() const;

            /** \brief Retrieve the current number of vertices in the graph as a planner-progress property. (Size of the
             * connected states in graphPtr_.) */
            std::string currentVertexProgressProperty() const;

            /** \brief Retrieve the current number of vertex in the search queue as a planner-progress property. (The
             * size of the edge subqueue of queuePtr_.) */
            std::string vertexQueueSizeProgressProperty() const;

            /** \brief Retrieve the number of iterations as a planner-progress property. (numIterations_) */
            std::string iterationProgressProperty() const;

            /** \brief Retrieve the number of batches processed as a planner-progress property. (numBatches_) */
            std::string batchesProgressProperty() const;

            /** \brief Retrieve the number of graph prunings performed as a planner-progress property. (numPrunings_) */
            std::string pruningProgressProperty() const;

            /** \brief Retrieve the \e total number of states generated as a planner-progress property. (From graphPtr_)
             */
            std::string totalStatesCreatedProgressProperty() const;

            /** \brief Retrieve the \e total number of vertices added to the graph as a planner-progress property. (From
             * graphPtr_) */
            std::string verticesConstructedProgressProperty() const;

            /** \brief Retrieve the number of states pruned from the problem as a planner-progress property. (From
             * graphPtr_) */
            std::string statesPrunedProgressProperty() const;

            /** \brief Retrieve the number of graph vertices that are disconnected and either returned to the set of
             * free samples or deleted completely as a planner-progress property. (From graphPtr_) */
            std::string verticesDisconnectedProgressProperty() const;

            /** \brief Retrieve the number of global-search edges that rewired the graph as a planner-progress property.
             * (numRewirings_) */
            std::string rewiringProgressProperty() const;

            /** \brief Retrieve the number of state collisions checks (i.e., calls to SpaceInformation::isValid(...)) as
             * a planner-progress property. (From graphPtr_) */
            std::string stateCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of edge (or motion) collision checks (i.e., calls to
             * SpaceInformation::checkMotion(...)) as a planner-progress property. (numEdgeCollisionChecks_) */
            std::string edgeCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of nearest neighbour calls (i.e., NearestNeighbors<T>::nearestK(...) or
             * NearestNeighbors<T>::nearestR(...)) as a planner-progress property. (From graphPtr_) */
            std::string nearestNeighbourProgressProperty() const;

            /** \brief Retrieve the total number of vertices processed from the queue as a planner-progress property.
             * (From queuePtr_) */
            std::string verticesProcessedProgressProperty() const;

            // ---
            // Member variables (Make all are configured in setup() and reset in reset()).
            // ---

            /** \brief A helper for cost and heuristic calculations. */
            std::shared_ptr<CostHelper> costHelpPtr_{nullptr};

            /** \brief The samples represented as an edge-implicit graph. */
            std::shared_ptr<ImplicitGraph> graphPtr_{nullptr};

            /** \brief The queue of vertices to expand and edges to process ordered on "f-value", i.e., estimated
             * solution cost. Remaining vertex queue "size" and edge queue size are accessible via
             * vertexQueueSizeProgressProperty and edgeQueueSizeProgressProperty, respectively. */
            std::shared_ptr<SearchQueue> queuePtr_{nullptr};

            /** \brief The queue of vertices in an invalidated subtree that need thei g-values to be
             * revised and made consistent with recent collision evaluations. This queue is used
             * as part of an LPA* style of tree repair. */
            std::shared_ptr<SearchQueue> repairQueuePtr_{nullptr};

            /** \brief The event that defines the toggle between lazy search and evaluation. */
            std::shared_ptr<Event> eventPtr_{nullptr};

            /** \brief The selector that defines the strategy to choose edges for collision checking. */
            std::shared_ptr<Selector> selectorPtr_{nullptr};

            /** \brief The best cost found to date. This is the maximum total-heuristic cost of samples we'll consider.
             * Accessible via bestCostProgressProperty */
            // Gets set in setup to the proper calls from OptimizationObjective
            ompl::base::Cost bestCost_;

            /** \brief The number of vertices in the best solution found to date. Accessible via
             * bestLengthProgressProperty. */
            unsigned int bestLength_{0u};

            /** \brief The cost to which the problem has been pruned. We will only prune the graph when a new solution
             * is sufficiently less than this value. */
            // Gets set in setup to the proper calls from OptimizationObjective.
            ompl::base::Cost prunedCost_{std::numeric_limits<double>::infinity()};

            /** \brief The measure to which the problem has been pruned. We will only prune the graph when the resulting
             * measure of a new solution is sufficiently less than this value. */
            // Gets set in setup with the proper call to Planner::si_->getSpaceMeasure()
            double prunedMeasure_{0.0};

            /** \brief If we've found an exact solution yet. */
            bool hasExactSolution_{false};

            /** \brief The flag whether the current search is done. */
            bool isSearchDone_{false};

            /** \brief The number of batches processed. Accessible via batchesProgressProperty. */
            unsigned int numBatches_{0u};

            /** \brief The number of times the graph/samples have been pruned. Accessible via pruningProgressProperty.
             */
            unsigned int numPrunings_{0u};

            /** \brief The number of iterations run. Accessible via iterationProgressProperty. */
            unsigned int numIterations_{0u};

            /** \brief The number of times a state in the graph was rewired. Accessible via rewiringProgressProperty. */
            unsigned int numRewirings_{0u};

            /** \brief The number of edge collision checks. Accessible via edgeCollisionCheckProgressProperty. */
            unsigned int numEdgeCollisionChecks_{0u};
            void generateSamplesCostLog() const;
            void printGraph() const;

            // ---
            // Parameters - Set defaults in construction/setup and do not reset in clear.
            // ---

            /** \brief The number of samples per batch. */
            unsigned int samplesPerBatch_{100u};

            /** \brief Whether to use graph pruning. */
            bool isPruningEnabled_{false};

            /** \brief The fractional decrease in solution cost required to trigger pruning. */
            double pruneFraction_{0.05};

            /** \brief Whether to stop the planner as soon as the path changes. */
            bool stopOnSolutionChange_{false};
        };  // class BITstar
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_

/** Notes:
 *
 * 1.Queue is always strictly ordered.
 *      [BIT* has a setStrictQueueOrdering of edges.]
 * 2. ABIT* settings are currently disabled.
 * 3. Rewiring of edges is never delayed.
 * 4. Disabling just in time sampling.
 * 5. Unconnected samples are not dropped without consideration.
 * 6. Only exact solutions are currently considered.
 *
 * */
