/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Aditya Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_LRASTAR_LRASTAR_
#define OMPL_GEOMETRIC_PLANNERS_LRASTAR_LRASTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{
    namespace base
    {
        // Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {
    /**
       // TODO (avk): What is anchor?
       @anchor gLRAstar
       @par Short description
       LRA* is a geometric planner that operates on a roadmap of
       milestones that approximate the connectivity of the state
       space. It is a "meta-planner" encapsulating an entire spectrum
       of lazy planners from Lazy-A* [1] to LazyPRM [2] or LazySP [3],
       to balance the edge evaluation effort with graph search effort.
       @par External documentation
       // TODO (avk): Citation for the ICAPS submission
       L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
       Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
       <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
       DOI: [10.1109/70.508439](http://dx.doi.org/10.1109/70.508439)<br>
       [[PDF]](http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078)
       [[more]](http://www.kavrakilab.org/robotics/prm.html)
    */

        /** \brief Lazy Receding-Horizon A* */
        class LRAstar : public base::Planner
        {
        public:
            // TODO (avk): Component?
            struct VProp
            {
                /// The underlying state of the vertex
                /// TODO (avk): Get rid of statewrapper
                StateWrapperPtr v_state;

                /// Cost-to-Come
                double cost;

                /// Estimate Cost-to-Come
                double lazyCost;

                /// Budget
                double budget;

                /// Parent
                std::size_t parent;

                /// Children
                std::vector<std::size_t> children;

                /// Flag to check if vertex is within the lazyband
                bool inLazyBand;
            }; // Vertex Properties

            struct EProp
            {
                /// The length of the edge using the space distance metric
                double length;

                /// Flag to check if edge is evaluated
                bool isEvaluated;

                /// States embedded in an edge
                std::vector<StateWrapperPtr> edgeStates;
            }; // Edge Properties

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. We use listS for the vertex list
             because vertex descriptors are invalidated by remove operations if using vecS.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list<
                boost::vecS, boost::vecS, boost::undirectedS, VProp, EProp> Graph;

            /** @brief The type for a vertex in the roadmap. */
            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            /** @brief The type for an edge in the roadmap. */
            typedef boost::graph_traits<Graph>::edge_descriptor Edge;
            /** @brief Iterator for vertices in the roadmap. */
            typedef boost::graph_traits<Graph>::vertex_iterator Vertexiter;
            /** @brief Iterator for vertices in the roadmap. */
            typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

            /** @brief A nearest neighbors data structure for roadmap vertices. */
            typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;

            /** @brief A function returning the milestones that should be
             * attempted to connect to. */
            typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

            /** @brief A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            typedef std::function<bool(const Vertex &, const Vertex &)> ConnectionFilter;

            /** \brief Constructor */
            LRAstar(const base::SpaceInformationPtr &si, bool starStrategy = false);

            ~LRAstar() override;

            /** \brief Set the maximum length of a motion to be added to the roadmap. */
            void setRange(double distance);

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    connectionStrategy_ = ConnectionStrategy();
                if (isSetup())
                    setup();
            }

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of PRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }

            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Set the function that can reject a milestone connection.

             \par The given function is called immediately before a connection
             is checked for collision and added to the roadmap. Other neighbors
             may have already been connected before this function is called.
             This allows certain heuristics that use the structure of the
             roadmap (like connected components or useful cycles) to be
             implemented by changing this function.

             \param connectionFilter A function that takes the new milestone,
             a neighboring milestone and returns whether a connection should be
             attempted.
             */
            void setConnectionFilter(const ConnectionFilter &connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const
            {
                return boost::num_edges(g_);
            }

            void getPlannerData(base::PlannerData &data) const override;

            void setup() override;

            void clear() override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for LazyPRM. */
            void clearQuery();

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        protected:
            /** \brief Flag indicating validity of an edge or a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge or a vertex */
            static const unsigned int VALIDITY_TRUE = 1;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return std::to_string(iterations_);
            }
            std::string getBestCost() const
            {
                return boost::lexical_cast<std::string>(bestCost_);
            }
            std::string getMilestoneCountString() const
            {
                return std::to_string(milestoneCount());
            }
            std::string getEdgeCountString() const
            {
                return std::to_string(edgeCount());
            }

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
               structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state);

            void uniteComponents(Vertex a, Vertex b);

            void markComponent(Vertex v, unsigned long int newComponent);

            /** \brief Check if any pair of a start state and goal state are part of the same connected component.
                If so, return the id of that component. Otherwise, return -1. */
            long int solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
             * it as the solution */
            ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const;

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(const Vertex u, const Vertex v) const;

            ///////////////////////////////////////
            // Planner-Specific Functions
            /** \brief Find the path from leaf vertex to border vertex
            /// \param[in] _vertex Vertex on the frontier */
            std::vector<Vertex> pathToBorder(const Vertex u);

            /** \brief Extend the Lazy Band
            /// \param[t]  queue with custom comparator
            /// \param[in] qExtend priority queue of vertices to extend
            /// \param[in] qFrontier priority queue of leaf vertices */
            template<class TF>
            void extendLazyBand(TF &qExtend, TF &qFrontier);

            /** \brief Update the Lazy Band
            /// \param[t] queue with custom comparator
            /// \param[in] qUpdate priority queue of vertices to update
            /// \param[in] qExtend priority queue of vertices to extend
            /// \param[in] qFrontier priority queue of leaf vertices */
            template<class TG, class TF>
            void updateLazyBand(TG &qUpdate, TF &qExtend, TF &qFrontier);

            /** \brief Rewire the Lazy Band if Collision Encountered
            /// param[t]  queue with custom comparator
            /// param[in] qExtend priority queue of vertices to rewire
            /// param[in] qExtend priority queue of vertices to extend
            /// param[in] qFrontier priority queue of leaf vertices */
            template<class TG, class TF>
            void rewireLazyBand(TG &qRewire, TF &qExtend, TF &qFrontier);

            /** \brief Extend the Lazy Band
            /// param[t] TG queue with custom comparator
            /// param[in] pathTail sequence of edges to evaluate
            /// param[in] qExtend priority queue of vertices to update
            /// param[in] qExtend priority queue of vertices to rewire
            /// param[in] qExtend priority queue of vertices to extend
            /// param[in] qFrontier priority queue of leaf vertices */
            template<class TG, class TF>
            bool evaluatePath(std::vector<Vertex> pathTail, TG &qUpdate, TG &qRewire, TF &qExtend, TF &qFrontier);

            /** \brief Lookahead */
            double lookahead_;

            /** \brief Greediness */
            double greediness_;

            /** \brief Path to the roadmap */
            std::string roadmapFileName_;

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool starStrategy_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
             * assumed) */
            bool userSetConnectionStrategy_{false};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr sampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex> goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access the connected component of a vertex */
            boost::property_map<Graph, vertex_component_t>::type vertexComponentProperty_;

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;

            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type edgeValidityProperty_;

            /** \brief Number of connected components created so far. This is used as an ID only,
                does not represent the actual number of components currently in the graph. */
            unsigned long int componentCount_{0};

            /** \brief The number of elements in each component in the LazyPRM roadmap. */
            std::map<unsigned long int, unsigned long int> componentSize_;

            /** \brief Objective cost function for PRM graph edges */
            base::OptimizationObjectivePtr opt_;

            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            unsigned long int iterations_{0};
        };
    }
}

#endif
