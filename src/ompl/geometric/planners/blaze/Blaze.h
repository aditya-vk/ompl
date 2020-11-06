/* Authors: Aditya Vamsikrishna Mandalika */

#include <unordered_set>
#include <vector>

#include <ompl/base/Planner.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/geometric/planners/blaze/blaze/ImplicitGraph.h"
#include "ompl/geometric/planners/blaze/blaze/PriorityQueue.h"
#include "ompl/geometric/planners/blaze/blaze/Vertex.h"
#include "ompl/geometric/planners/blaze/blaze/Event.h"
#include "ompl/geometric/planners/blaze/blaze/Selector.h"

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_

namespace ompl
{
    namespace geometric
    {
        class Blaze : public ompl::base::Planner
        {
        public:
            /// \brief Constructor
            Blaze(const ompl::base::SpaceInformationPtr &si, const std::string &name);

            /// \brief Destructor
            ~Blaze() override = default;

            /// \brief Setup
            void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

            /// \brief Clear
            void clear() override;

            /// \brief Solve
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            /// \brief Set event.
            void setEvent(const blaze::EventPtr event)
            {
                mEvent = event;
            }

            /// \brief Set selector.
            void setSelector(const blaze::SelectorPtr selector)
            {
                mSelector = selector;
            }

            /// \brief Set the number of samples per batch.
            void setBatchSize(const std::size_t batchSize)
            {
                mBatchSize = batchSize;
            }

            void getPlannerData(ompl::base::PlannerData &data) const override;

        private:
            /// \brief Search iteration.
            ompl::base::PlannerStatus iterate(const ompl::base::PlannerTerminationCondition &ptc);

            /// \brief Expansion of the search tree.
            /// Continues until the event is triggered.
            void search();

            /// \brief Evaluation of subpaths after an event trigger.
            void evaluate();

            /// \brief Prunes the graph and queues removing samples that are not helpful.
            void prune();

            /// \brief Helper function to repair the search tree if an invalid edge found.
            void repair(const blaze::VertexID repairRoot);

            /// \brief The underlying graph to search over.
            blaze::EventPtr mEvent{nullptr};

            /// \brief The underlying graph to search over.
            blaze::SelectorPtr mSelector{nullptr};

            /// \brief The underlying graph to search over.
            blaze::ImplicitGraphPtr mImplicitGraph{nullptr};

            /// \brief The search queue.
            blaze::SearchQueuePtr mSearchQueue{nullptr};

            /// \brief Repair queue.
            blaze::RepairQueuePtr mRepairQueue{nullptr};

            /// \brief Flag to determine if solution has been found.
            bool mSolutionFound{false};

            /// \brief The best cost found until now.
            double mBestPathCost{std::numeric_limits<double>::infinity()};

            /// \brief The number of samples in the graph.
            std::size_t mBatchSize{0};
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_
