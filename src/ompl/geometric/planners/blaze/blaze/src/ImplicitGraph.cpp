/* Authors: Aditya Vamsikrishna Mandalika */

#include <fstream>

#include "ompl/geometric/planners/blaze/blaze/ImplicitGraph.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
#include "ompl/geometric/planners/blaze/blaze/PriorityQueue.h"
#include "ompl/geometric/planners/blaze/blaze/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            // ================================================================================================
            ImplicitGraph::ImplicitGraph(const ompl::base::SpaceInformationPtr &si,
                                         const ompl::base::ProblemDefinitionPtr &pdef,
                                         const ompl::base::Planner *planner)
              : mSpaceInformation(si), mProblemDefinition(pdef)
            {
                // Initialize the NN structures.
                mFreeSamplesNN.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexID>(planner));
                mVerticesNN.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexID>(planner));

                // Configure the distance metric within the NN structure.
                ompl::NearestNeighbors<VertexID>::DistanceFunction nnDistanceFunction(
                    [this](const VertexID &a, const VertexID &b) { return this->distanceFunction(a, b); });
                mFreeSamplesNN->setDistanceFunction(nnDistanceFunction);
                mVerticesNN->setDistanceFunction(nnDistanceFunction);

                // Allocate a sampler to generate the graph.
                mSampler = mProblemDefinition->getOptimizationObjective()->allocInformedStateSampler(
                    mProblemDefinition, std::numeric_limits<unsigned int>::max());

                // Finally, add start and goal to the graph.
                addStartAndGoal();
            }

            // ================================================================================================
            void ImplicitGraph::addStartAndGoal()
            {
                // Add the start vertex.
                mGeneratedSamples.push_back(
                    std::make_shared<Vertex>(mSpaceInformation, mProblemDefinition->getStartState(0)));
                mStartVertex = mGeneratedSamples.back()->getID();

                // Add the goal vertex.
                mGeneratedSamples.push_back(std::make_shared<Vertex>(
                    mSpaceInformation, mProblemDefinition->getGoal()->as<ompl::base::GoalState>()->getState()));
                mGoalVertex = mGeneratedSamples.back()->getID();

                // Minimum distance possible between start and goal vertices.
                const double heuristic = distanceFunction(mStartVertex, mGoalVertex);

                // Provide heuristic distances to start and goal vertices.
                mGeneratedSamples.at(mStartVertex)->setCostToCome(0);
                mGeneratedSamples.at(mStartVertex)->setCostToComeHeuristic(0);
                mGeneratedSamples.at(mStartVertex)->setCostToGoHeuristic(heuristic);

                mGeneratedSamples.at(mGoalVertex)->setCostToComeHeuristic(heuristic);
                mGeneratedSamples.at(mGoalVertex)->setCostToGoHeuristic(0);

                // Populate the mVerticesNN with the start vertex.
                // TODO(avk): The start vertex has no parent and therefore not in the tree.
                addVertex(mStartVertex);

                // Populate the mFreeSamplesNN with the goal vertex.
                addSample(mGoalVertex);

                // Set the lower bound over achievable cost.
                setLowerCostBound(heuristic);
            }

            // ================================================================================================
            void ImplicitGraph::updateConnectionRadius()
            {
                const int N = mGeneratedSamples.size();

                // The dimension cast as a double for further computation.
                const auto dimension = mSpaceInformation->getStateDimension();

                // Calculate the term and return
                const auto minRGG = this->minimumRggR();
                mConnectionRadius = minRGG * std::pow(std::log(N) / N, 1.0 / dimension);
            }

            // ================================================================================================
            double ImplicitGraph::minimumRggR() const
            {
                // The dimension cast as a double for readibility;
                const auto dimension = mSpaceInformation->getStateDimension();

                // The measure of the space.
                const auto spaceMeasure = mSpaceInformation->getSpaceMeasure();

                // RRT*
                // TODO(avk): Provide an option to use other radius computations.
                return std::pow(2.0 * (1.0 + 1.0 / dimension) * (spaceMeasure / ompl::unitNBallMeasure(dimension)),
                                1.0 / dimension);
            }

            // ================================================================================================
            void ImplicitGraph::generateSamples(std::size_t requiredNumberOfSamples)
            {
                // Begin sampling.
                std::size_t numSamples = 0;
                ompl::base::State *sampledState = mSpaceInformation->allocState();
                while (numSamples < requiredNumberOfSamples)
                {
                    mSampler->sampleUniform(sampledState, ompl::base::Cost(mLowerCostBound),
                                            ompl::base::Cost(mUpperCostBound));

                    // If the state is collision free, collect it.
                    if (mSpaceInformation->isValid(sampledState))
                    {
                        numSamples++;
                        const VertexID sampledVertexID = mGeneratedSamples.size();
                        mGeneratedSamples.push_back(std::make_shared<Vertex>(mSpaceInformation, sampledState));
                        addSample(sampledVertexID);

                        // Populate the heuristic values. This is a one-time computation.
                        mGeneratedSamples.back()->setCostToComeHeuristic(
                            distanceFunction(mStartVertex, sampledVertexID));
                        mGeneratedSamples.back()->setCostToGoHeuristic(distanceFunction(sampledVertexID, mGoalVertex));
                    }
                }
                // Update the connection radius.
                updateConnectionRadius();
            }

            // ================================================================================================
            void ImplicitGraph::pruneSamplesAndVertices()
            {
                // TODO(avk): To be implemented.
            }

            // ================================================================================================
            void ImplicitGraph::getGraphAsPlannerData(ompl::base::PlannerData &data) const
            {
                // Add the start vertex of the graph.
                data.addStartVertex(ompl::base::PlannerDataVertex(mGeneratedSamples.at(mStartVertex)->getState()));

                // Add unconnected samples.
                VertexIDVector samples;
                mFreeSamplesNN->list(samples);
                for (const auto &sample : samples)
                {
                    data.addVertex(ompl::base::PlannerDataVertex(mGeneratedSamples.at(sample)->getState()));
                }

                // Add vertices in the search tree.
                VertexIDVector vertices;
                mVerticesNN->list(vertices);
                for (const auto &vertex : vertices)
                {
                    auto &vertexPtr = mGeneratedSamples.at(vertex);
                    data.addVertex(ompl::base::PlannerDataVertex(vertexPtr->getState()));

                    // Add edges in the search tree.
                    if (vertex != mStartVertex)
                    {
                        auto &parentPtr = mGeneratedSamples.at(vertexPtr->getParent());
                        data.addEdge(ompl::base::PlannerDataVertex(vertexPtr->getState()),
                                     ompl::base::PlannerDataVertex(parentPtr->getState()));
                    }
                }
            }

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl
