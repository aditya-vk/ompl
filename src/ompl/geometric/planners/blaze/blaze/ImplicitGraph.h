/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_IMPLICITGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_IMPLICITGRAPH_

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerData.h"

#include "ompl/geometric/planners/blaze/blaze/PriorityQueue.h"
#include "ompl/geometric/planners/blaze/blaze/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            ///
            /// \brief An edge-implicit representation of a random geometric graph.
            ///
            class ImplicitGraph
            {
                // Type alias to denote the nearest neighbor structure.
                using VertexIDNN = std::shared_ptr<ompl::NearestNeighbors<VertexID>>;

            public:
                /// \brief Constructor.
                ImplicitGraph(const ompl::base::SpaceInformationPtr &si, const ompl::base::ProblemDefinitionPtr &pdef,
                              const ompl::base::Planner *planner);

                /// \brief Destructor.
                virtual ~ImplicitGraph()
                {
                    // TODO(avk): Clear out all the raw pointers in generated vertices.
                }

                /// \brief Clears the graph to the state of construction.
                void clear()
                {
                    mVerticesNN->clear();
                    mFreeSamplesNN->clear();
                    clearSolution();
                }

                /// \brief Adds start and goal from pdef to the graph and search queue.
                void addStartAndGoal();

                /// \brief Returns the start vertex.
                VertexID getStartVertex() const
                {
                    return mStartVertex;
                }

                /// \brief Returns the goal vertex.
                VertexID getGoalVertex() const
                {
                    return mGoalVertex;
                }

                /// \brief Returns the generated samples.
                const std::vector<VertexPtr> &getGeneratedSamples() const
                {
                    return mGeneratedSamples;
                }

                /// \brief The distance function.
                double distanceFunction(const VertexID a, const VertexID b) const
                {
                    return mSpaceInformation->distance(mGeneratedSamples.at(a)->getState(),
                                                       mGeneratedSamples.at(b)->getState());
                }

                /// \brief Returns the connection radius.
                double getConnectionRadius() const
                {
                    return mConnectionRadius;
                }

                /// TODO(avk): Need to know #(un)connected samples.
                /// \brief Generate a batch of samples.
                void generateSamples(std::size_t requiredNumberOfSamples);

                /// \brief Get the nearest unconnected samples.
                VertexIDVector nearestSamples(const VertexID vertex) const
                {
                    VertexIDVector neighbors;
                    mFreeSamplesNN->nearestR(vertex, mConnectionRadius, neighbors);
                    return neighbors;
                }

                /// \brief Get the nearest samples from the vertexNN.
                VertexIDVector nearestVertices(const VertexID vertex)
                {
                    VertexIDVector neighbors;
                    mVerticesNN->nearestR(vertex, mConnectionRadius, neighbors);
                    return neighbors;
                }

                /// \brief Add an unconnected sample.
                void addSample(const VertexID vertex)
                {
                    mFreeSamplesNN->add(vertex);
                }

                /// \brief Remove an unconnected sample.
                void removeSample(const VertexID vertex)
                {
                    mFreeSamplesNN->remove(vertex);
                }

                /// \brief Add a vertex to the tree.
                void addVertex(const VertexID vertex)
                {
                    mVerticesNN->add(vertex);
                }

                /// \brief Remove a vertex from the tree.
                void removeVertex(const VertexID vertex)
                {
                    mVerticesNN->remove(vertex);
                }

                /// \brief Prunes a vertex from the graph. Cleans up associated data.
                // TODO(avk): Need to clear the neighbors later.
                void pruneVertex(const VertexID vertex)
                {
                    mVerticesNN->remove(vertex);
                }

                /// \brief Set a different nearest neighbours datastructure
                template <template <typename T> class NN>
                void setNearestNeighbors();

                /// \brief The number of nearest neighbour calls.
                std::size_t numberOfNearestLookups() const;

                /// \brief Evaluate the collision status of edge. Return true if valid.
                bool evaluateEdge(const Edge &edge) const
                {
                    return mSpaceInformation->checkMotion(mGeneratedSamples.at(edge.first)->getState(),
                                                          mGeneratedSamples.at(edge.second)->getState());
                }

                /// \brief Set the solution found flag to given status.
                void setSolutionFound()
                {
                    mSolutionFound = true;
                    auto path = constructSolution();
                    mProblemDefinition->addSolutionPath(path);
                    setUpperCostBound(path->length());
                }

                /// \brief Sets solution found to be false.
                void clearSolution()
                {
                    mSolutionFound = false;
                }

                /// \brief Publishes the best available solution to the console.
                void publishSolution() const
                {
                    OMPL_INFORM("Best solution cost: %f", mUpperCostBound);
                }

                /// \brief Returns true if solution has been found.
                bool solutionFound() const
                {
                    return mSolutionFound;
                }

                /// \brief Return the path from start to vertex.
                VertexIDVector getPathToStart(const VertexID vertex) const
                {
                    VertexID currentVertex = vertex;
                    VertexIDVector path;
                    while (currentVertex != -1)
                    {
                        path.emplace_back(currentVertex);
                        currentVertex = mGeneratedSamples.at(currentVertex)->getParent();
                    }
                    return path;
                }

                /// \brief Returns the path from start to goal.
                ompl::base::PathPtr constructSolution() const
                {
                    ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(mSpaceInformation);

                    VertexID currentVertex = mGoalVertex;
                    while (currentVertex != -1)
                    {
                        path->append(mGeneratedSamples.at(currentVertex)->getState());
                        currentVertex = mGeneratedSamples.at(currentVertex)->getParent();
                    }
                    path->reverse();
                    return ompl::base::PathPtr(path);
                }

                /// \brief Sets the cost bounds.
                void setLowerCostBound(const double lower)
                {
                    mLowerCostBound = lower;
                }
                void setUpperCostBound(const double upper)
                {
                    mUpperCostBound = upper;
                }
                double getLowerCostBound() const
                {
                    return mLowerCostBound;
                };
                double getUpperCostBound() const
                {
                    return mUpperCostBound;
                };

                /// Prunes the vertices and samples that are no longer useful.
                void pruneSamplesAndVertices();

                /// Retreives the samples in the graph for logs.
                void getGraphAsPlannerData(ompl::base::PlannerData &data) const;

            private:
                /// \brief Compute the radius of this r-disc RGG from available samples.
                void updateConnectionRadius();

                /// \brief Calculate the lower-bounding radius RGG term for asymptotic
                /// almost-sure convergence to the optimal path (i.e., r_rrg* in Karaman and
                /// Frazzoli IJRR 11). This is a function of the size of the problem domain.
                double minimumRggR() const;

                /// \brief The state space used by the planner.
                ompl::base::SpaceInformationPtr mSpaceInformation{nullptr};

                /// \brief The problem definition.
                ompl::base::ProblemDefinitionPtr mProblemDefinition{nullptr};

                /// \brief An instance of a random number generator
                ompl::RNG mRNG;

                /// \brief State sampler.
                ompl::base::InformedSamplerPtr mSampler{nullptr};

                /// \brief Container for the generated vertices/samples.
                std::vector<VertexPtr> mGeneratedSamples;

                /// \brief Lower bound on the solution cost.
                double mLowerCostBound{0};

                /// \brief Upper bound on the solution cost.
                double mUpperCostBound{std::numeric_limits<double>::infinity()};

                /// \brief Holds the samples from the current batch.
                VertexIDVector mCurrentBatchSamples;

                /// \brief The start vertex.
                VertexID mStartVertex;

                /// \brief The goal vertex.
                VertexID mGoalVertex;

                /// \brief The samples as a NN datastructure. Sorted by distance.
                VertexIDNN mFreeSamplesNN{nullptr};

                /// \brief The vertices as a NN data structure. Sorted by distance.
                VertexIDNN mVerticesNN{nullptr};

                /// \brief The current r-disc RGG connection radius
                double mConnectionRadius{std::numeric_limits<double>::infinity()};

                /// \brief Flag to indicate that the roadmap has a solution determined.
                bool mSolutionFound{false};

            };  // ImplicitGraph

            // Type aliases to simplify code.
            using ImplicitGraphPtr = std::shared_ptr<ImplicitGraph>;

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_IMPLICITGRAPH_
