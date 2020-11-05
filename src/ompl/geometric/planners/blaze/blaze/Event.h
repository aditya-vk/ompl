/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_EVENTH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_EVENTH_

#include "ompl/geometric/planners/blaze/blaze/Event.h"
#include "ompl/geometric/planners/blaze/blaze/ImplicitGraph.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            ///
            /// \brief Defines the trigger to pause search over the graph.
            /// The default functionality is the ShortestPathEvent i.e. triggers when the
            /// goal vertex is provided.
            ///
            class Event
            {
            public:
                /// \brief Construct a new Event.
                Event()
                {
                    // Do nothing.
                }

                /// \brief Destroy the Event.
                virtual ~Event() = default;

                /// \brief Setup the event with the underlying graph and the goal vertex.
                void setup(const ImplicitGraphPtr graph)
                {
                    mImplicitGraph = graph;
                    mGoalVertex = mImplicitGraph->getGoalVertex();
                }

                /// \brief Returns true if the given vertex triggers the Event.
                ///
                /// \param[in] vertex The vertex to check for the event's trigger.
                virtual bool isTriggered(const VertexID vertex) const
                {
                    return (vertex == mGoalVertex);
                }

            protected:
                /// \brief The associated graph.
                ImplicitGraphPtr mImplicitGraph;

                /// \brief The goal vertex.
                VertexID mGoalVertex;
            };
            // Type alias to specify that this event is equivalent to the ShortestPathEvent.
            using ShortestPathEvent = Event;
            using EventPtr = std::shared_ptr<Event>;

            /// \brief ConstantDepth Event.
            /// Event triggers when the lazy subpath reaches given depth.
            class ConstantDepthEvent : public Event
            {
            public:
                /// \brief Constructor.
                explicit ConstantDepthEvent(std::size_t depth) : mDepth(depth)
                {
                    // Do nothing.
                }

                /// \brief Returns true if the event is triggered.
                bool isTriggered(const VertexID vertex) const override
                {
                    if (vertex == mGoalVertex)
                    {
                        return true;
                    }

                    // Check the depth.
                    VertexID currentVertex = vertex;
                    std::size_t depth = 0;
                    const auto &graphVertices = mImplicitGraph->getGeneratedSamples();
                    while (currentVertex != -1)
                    {
                        const auto vertexPtr = graphVertices.at(currentVertex);
                        const auto parent = vertexPtr->getParent();
                        auto evaluatedEdge = graphVertices.at(parent)->hasEvaluatedNeighbor(currentVertex) ||
                                             graphVertices.at(currentVertex)->hasEvaluatedNeighbor(parent);
                        if (!evaluatedEdge)
                        {
                            depth++;
                            if (depth == mDepth)
                            {
                                return true;
                            }
                        }
                        currentVertex = parent;
                    }
                    return false;
                }

                /// \brief Return the depth the event triggers for.
                std::size_t getDepth() const
                {
                    return mDepth;
                }

            private:
                /// The depth threshold.
                const std::size_t mDepth;
            };

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_EVENTH_
