/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_SELECTORH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_SELECTORH_

#include "ompl/geometric/planners/blaze/blaze/ImplicitGraph.h"
#include "ompl/geometric/planners/blaze/blaze/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            ///
            /// \brief Defines the interface to selectors. Selectors define the strategy to
            /// choose edges for evaluation.
            ///
            class Selector
            {
            public:
                /// \brief Construct a new Selector.
                Selector()
                {
                    // Do nothing.
                }

                /// \brief Destroy the Selector.
                virtual ~Selector() = default;

                /// \brief Setup the selector with the underlying graph.
                void setup(const ImplicitGraphPtr graph)
                {
                    mImplicitGraph = graph;
                }

                ///
                /// \brief Given a path, defines the strategy to select edges for evaluation.
                ///
                /// \param[in] subPath Subpath comprising of at least one unevaluated edge.
                /// \return The edge to evaluate.
                virtual Edge selectEdgeToEvaluate(const VertexIDVector &subPath) const = 0;

            protected:
                /// The associated graph.
                ImplicitGraphPtr mImplicitGraph;
            };
            using SelectorPtr = std::shared_ptr<Selector>;

            ///
            /// \brief Returns an unevaluated edge on the given path closest to start.
            ///
            class ForwardSelector : public Selector
            {
            public:
                /// \brief Constructor.
                ForwardSelector()
                {
                    // Do nothing.
                }

                /// \brief Returns true if the event is triggered.
                Edge selectEdgeToEvaluate(const VertexIDVector &path) const override
                {
                    const auto &graphVertices = mImplicitGraph->getGeneratedSamples();
                    for (std::size_t i = path.size() - 1; i > 0; --i)
                    {
                        VertexID u = path.at(i);
                        VertexID v = path.at(i - 1);

                        // If the edge has been evaluated, continue.
                        if (graphVertices.at(u)->hasFreeNeighbor(v) || graphVertices.at(v)->hasFreeNeighbor(u))
                        {
                            continue;
                        }
                        return Edge(u, v);
                    }
                    return Edge();
                }
            };

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_SELECTORH_
