/* Authors: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IMPLICITGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IMPLICITGRAPH_

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/informedtrees/IGLS.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor ImplicitGraph
        \par Short Description
        An edge-implicit representation of a random geometric graph.
        */

        /** \brief A conceptual representation of samples as an edge-implicit random geometric graph. */
        class IGLS::ImplicitGraph
        {
        public:
            // ---
            // Construction, initialization and destruction.
            // ---

            /** \brief Construct an implicit graph. */
            ImplicitGraph(NameFunc nameFunc);

            /** \brief Destruct the graph using default destruction. */
            virtual ~ImplicitGraph() = default;

            /** \brief Setup the ImplicitGraph, must be called before use. Does not take a copy of the
             * PlannerInputStates, but checks it for starts/goals. */
            void setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition, CostHelper *costHelper,
                       SearchQueue *searchQueue, const ompl::base::Planner *plannerPtr,
                       ompl::base::PlannerInputStates &inputStates);

            /** \brief Reset the graph to the state of construction. */
            void reset();

            // ---
            // Information access.
            // ---

            /** \brief Gets whether the graph contains a start or not. */
            bool hasAStart() const;

            /** \brief Gets whether the graph contains a goal or not. */
            bool hasAGoal() const;

            // TODO(avk): Not supporting multiple start-goals. Replaced vector
            // accessors with point returns.
            VertexPtr getStartVertex() const;
            VertexPtr getGoalVertex() const;

            /** \brief Get the minimum cost solution possible for this problem. */
            ompl::base::Cost minCost() const;

            /** \brief Query whether the underlying state sampler can provide an informed measure. */
            bool hasInformedMeasure() const;

            /** \brief Query the underlying state sampler for the informed measure of the problem. */
            double getInformedMeasure(const ompl::base::Cost &cost) const;

            /** \brief Computes the distance between two states.*/
            double distance(const VertexConstPtr &a, const VertexConstPtr &b) const;

            /** \brief Computes the distance between two states.*/
            double distance(const VertexConstPtrPair &vertices) const;

            /** \brief Get the nearest unconnected samples using the appropriate "near" definition (i.e., k or r). */
            void nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples);

            /** \brief Adds the graph to the given PlannerData struct. */
            void getGraphAsPlannerData(ompl::base::PlannerData &data) const;

            // TODO(avk): Not supporting approximate solutions, so closestVertex unnecessary.

            /** \brief Get the k of this k-nearest RGG. */
            unsigned int getConnectivityK() const;

            /** \brief Get the radius of this r-disc RGG. */
            double getConnectivityR() const;

            /** \brief Get a copy of all samples. */
            VertexPtrVector getCopyOfSamples() const;

            // ---
            // Modification.
            // ---

            /** \brief Mark that a solution has been found and that the graph should be limited to the given heuristic
             * value. */
            void registerSolutionCost(const ompl::base::Cost &solutionCost);

            /** \brief Adds any new goals or starts that have appeared in the problem definition to the vector of
             * vertices and the queue. Creates a new informed sampler if necessary. */
            void updateStartAndGoalStates(ompl::base::PlannerInputStates &inputStates,
                                          const base::PlannerTerminationCondition &terminationCondition);

            /** \brief Increase the resolution of the graph-based approximation of the continuous search domain by
             * adding a batch of new samples. */
            void addNewSamples(const unsigned int &numSamples);

            /** \brief Prune the samples to the subproblem of the given measure. Returns the number of vertices
             * disconnected and the number of samples removed. */
            std::pair<unsigned int, unsigned int> prune(double prunedMeasure);

            /** \brief Add an unconnected sample. */
            void addToSamples(const VertexPtr &sample);

            /** \brief Add a vector of unconnected samples. */
            void addToSamples(const VertexPtrVector &samples);

            /** \brief Remove a sample from the sample set. */
            void removeFromSamples(const VertexPtr &sample);

            /** \brief Remove an unconnected sample.*/
            void pruneSample(const VertexPtr &sample);

            /** \brief Insert a sample into the set for recycled samples.
             * This is done to vertices in the current search tree that have g(v) + h(v, \vg) > c
             * but h(\vs, v) + h(v, \vg) < c. Such vertices *upon pruning* are disconnected from tree
             * and added to recycleSampels_. Next time a new batch is called upon, these recycled samples
             * are used and another iteration of search is run unless recycleSamples_ is empty in which
             * case a new batch of samples is used.
             */
            // TODO(avk): Why do we need recycle samples?
            void recycleSample(const VertexPtr &sample);

            /** \brief Add a vertex to the tree, optionally moving it from the set of unconnected samples. */
            void registerAsVertex(const VertexPtr &vertex);

            /** \brief Remove a vertex from the tree, can optionally be allowed to move it to the set of unconnected
             * samples if may still be useful. */
            unsigned int removeFromVertices(const VertexPtr &sample, bool moveToFree);

            /** \brief Remove a vertex and mark as pruned. */
            std::pair<unsigned int, unsigned int> pruneVertex(const VertexPtr &vertex);

            /** \brief Disconnect a vertex from its parent by removing the edges stored in itself, and its parents.
             * Cascades cost updates if requested.*/
            void removeEdgeBetweenVertexAndParent(const VertexPtr &child, bool cascadeCostUpdates);

            // ---
            // Settings.
            // ---

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg*. */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getUseKNearest() const;

            // TODO(avk): JIT not supported for now.
            // TODO(avk): Unconnected samples are not dropped without consideration.

            /** \brief Set whether samples that are provably not beneficial should be kept around. */
            void setPruning(bool usePruning);

            /** \brief Set the average number of allowed failed attempts when sampling. */
            void setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number);

            /** \brief Get the average number of allowed failed attempts when sampling. */
            std::size_t getAverageNumOfAllowedFailedAttemptsWhenSampling() const;

            /** \brief Set a different nearest neighbours datastructure. */
            template <template <typename T> class NN>
            void setNearestNeighbors();

            // ---
            // Progress counters.
            // ---

            /** \brief The number of samples. */
            unsigned int numSamples() const;

            /** \brief The number of vertices in the search tree. */
            unsigned int numVertices() const;

            /** \brief The \e total number of states generated. */
            unsigned int numStatesGenerated() const;

            /** \brief The \e total number of vertices added to the graph. */
            unsigned int numVerticesConnected() const;

            /** \brief The number of states pruned. */
            unsigned int numFreeStatesPruned() const;

            /** \brief The number of tree vertices disconnected. */
            unsigned int numVerticesDisconnected() const;

            /** \brief The number of nearest neighbour calls. */
            unsigned int numNearestLookups() const;

            /** \brief The number of state collision checks. */
            unsigned int numStateCollisionChecks() const;

            // ---
            // General helper functions.
            // ---

            /** \brief Returns whether the vertex can be pruned, i.e., whether it could provide a better solution given.
             * the current graph. The check should always be g_t(v) + h^(v) >= g_t(x_g). */
            bool canVertexBeDisconnected(const VertexPtr &vertex) const;

            /** \brief Returns whether the sample can be pruned, i.e., whether it could ever provide a better solution.
             * The check should always be g^(v) + h^(v) >= g_t(x_g). */
            bool canSampleBePruned(const VertexPtr &sample) const;

        private:
            // ---
            // High-level primitives updating the graph.
            // ---

            /** \brief Update the set of free samples such that the neighbourhood of the given vertex is sufficiently
             * sampled. */
            void updateSamples(const VertexConstPtr &vertex);

            // ---
            // High-level primitives pruning the graph.
            // ---

            /** \brief Prune any samples that provably cannot provide a better solution than the current best solution.
             * Returns the number of samples removed. */
            std::pair<unsigned int, unsigned int> pruneSamples();

            // ---
            // Low-level random geometric graph helper and calculations
            // ---

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. Currently only implemented
             * for path-length problems, for which the neighbourhood cost is the f-value of the vertex plus 2r. */
            ompl::base::Cost calculateNeighbourhoodCost(const VertexConstPtr &vertex) const;

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_. */
            virtual void updateNearestTerms();

            /** \brief Computes the number of samples in the informed set. */
            std::size_t computeNumberOfSamplesInInformedSet() const;

            /** \brief Calculate the r for r-disc nearest neighbours, a function of the current graph. */
            double calculateR(unsigned int numUniformSamples) const;

            /** \brief Calculate the k for k-nearest neighours, a function of the current graph. */
            unsigned int calculateK(unsigned int numUniformSamples) const;

            /** \brief Calculate the lower-bounding radius RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., r_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the size of the
             * problem domain. */
            double calculateMinimumRggR() const;

            /** \brief Calculate the lower-bounding k-nearest RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., k_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the state dimension
             * and is left as a double for later accuracy in calculate k. */
            double calculateMinimumRggK() const;

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

            /** \brief Whether the class is setup. */
            bool isSetup_{false};

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr spaceInformation_{nullptr};

            /** \brief The problem definition. */
            ompl::base::ProblemDefinitionPtr problemDefinition_{nullptr};

            /** \brief A cost/heuristic helper class. As this is a copy of the version owned by IGLS.cpp it can be
             * reset in a clear(). */
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The queue class. As this is a copy of the version owned by IGLS.cpp it can be reset in a
             * clear(). */
            SearchQueue *queuePtr_{nullptr};

            /** \brief An instance of a random number generator. */
            ompl::RNG rng_;

            /** \brief State sampler */
            ompl::base::InformedSamplerPtr sampler_{nullptr};

            /** \brief The start vertex of the problem. */
            VertexPtr startVertex_;

            /** \brief The goal vertex of the problem. */
            VertexPtr goalVertex_;

            /** \brief A copy of the new samples of the most recently added batch. */
            VertexPtrVector newSamples_;

            /** \brief The samples as a nearest-neighbours datastructure. Sorted by nnDistance. */
            VertexPtrNNPtr samples_{nullptr};

            /** \brief A copy of the vertices recycled into samples during the most recently added batch. */
            // TODO(avk): Again, what exactly are these?
            VertexPtrVector recycledSamples_;

            /** \brief The number of samples in this batch. */
            unsigned int numNewSamplesInCurrentBatch_{0u};

            /** \brief The number of states (vertices or samples) that were generated from a uniform distribution. Only
             * valid when refreshSamplesOnPrune_ is true, in which case it's used to calculate the RGG term of the
             * uniform subgraph. */
            unsigned int numUniformStates_{0u};

            /** \brief The current r-disc RGG connection radius. */
            double r_{0.};

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be
             * calculated once. Left as a double for later accuracy in calculate k. */
            double k_rgg_{0.};

            /** \brief The current k-nearest RGG connection number. */
            unsigned int k_{0u};

            /** \brief The measure of the continuous problem domain which we are approximating with samples. This is
             * initially the problem domain but can shrink as we focus the search. */
            double approximationMeasure_{0.};

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the start. */
            ompl::base::Cost minCost_{std::numeric_limits<double>::infinity()};

            /** \brief The maximum heuristic cost to sample (i.e., the best solution found to date). */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief The total-heuristic cost up to which we've sampled. */
            ompl::base::Cost sampledCost_{std::numeric_limits<double>::infinity()};

            /** \brief If we've found an exact solution yet. */
            bool hasExactSolution_{false};

            /** \brief The number of states generated through sampling. Accessible via numStatesGenerated. */
            unsigned int numSamples_{0u};

            /** \brief The number of vertices ever added to the tree. Accessible via numVerticesConnected. */
            unsigned int numVertices_{0u};

            /** \brief The number of free states that have been pruned. Accessible via numStatesPruned. */
            unsigned int numFreeStatesPruned_{0u};

            /** \brief The number of graph vertices that get disconnected. Accessible via numVerticesDisconnected. */
            unsigned int numVerticesDisconnected_{0u};

            /** \brief The number of nearest neighbour calls. Accessible via numNearestLookups. */
            unsigned int numNearestNeighbours_{0u};

            /** \brief The number of state collision checks. Accessible via numStateCollisionChecks. */
            unsigned int numStateCollisionChecks_{0u};

            /** \brief The current approximation id. */
            const std::shared_ptr<unsigned int> approximationId_;

            // ---
            // Parameters - Set defaults in construction/setup and do not reset in clear.
            // ---

            /** \brief The rewiring factor, s, so that r_rgg = s \times r_rgg* > r_rgg*. */
            double rewireFactor_{1.1};

            /** \brief Option to use k-nearest search for rewiring. */
            bool useKNearest_{true};

            /** \brief Whether the graph is being pruned or not. */
            bool isPruningEnabled_{true};

            /** \brief The average number of allowed failed attempts before giving up on a sample when sampling a new
             * batch. */
            std::size_t averageNumOfAllowedFailedAttemptsWhenSampling_{2u};
        };  // class ImplicitGraph
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IMPLICITGRAPH_
