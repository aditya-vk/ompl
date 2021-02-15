#include "ompl/geometric/planners/informedtrees/igls/ExistenceGraph.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::ExistenceGraph::ExistenceGraph(
          const std::string& dataset, std::size_t edgeDiscretization, double obstacleDensity_)
          : edgeExistenceSparseDiscretization_{edgeDiscretization}, obstacleDensity_{obstacleDensity}
        {
            loadDataset(dataset);
        }

        void IGLS::ExistenceGraph::setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                                         const ompl::base::ProblemDefinitionPtr &problemDefinition,
                                         CostHelper *costHelper, SearchQueue *searchQueue,
                                         const ompl::base::Planner *plannerPtr,
                                         ompl::base::PlannerInputStates &inputStates)
        {
            isSetup_ = true;
            spaceInformation_ = spaceInformation;
            // are other arguments needed?

            if (!static_cast<bool>(nn_))
            {
                nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(plannerPtr));
            }

            NearestNeighbors<VertexPtr>::DistanceFunction distanceFunction(
                [this](const VertexConstPtr &a, const VertexConstPtr &b) { return distance(a, b); });
            nn_->setDistanceFunction(distanceFunction);
        }

        template <template <typename T> class NN>
        void IGLS::ExplicitGraph::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (isSetup_)
            {
                // TODO: what's nameFunc_?
                // OMPL_WARN("%s (ExplicitGraph): The nearest neighbour datastructures cannot be changed once the problem "
                //           "is setup. Continuing to use the existing containers.",
                //           nameFunc_().c_str());
            }
            else
            {
                // The problem isn't setup yet, create NN structs of the specified type:
                nn_ = std::make_shared<NN<VertexPtr>>();
            }
        }

        double IGLS::ExplicitGraph::distance(const VertexConstPtr &a, const VertexConstPtr &b) const
        {
            ASSERT_SETUP
            // TODO: will there be a problem if the vertices aren't in the same graph?
            return spaceInformation_->distance(b->state(), a->state());
        }

        void IGLS::ExistenceGraph::loadDataset(const std::string& dataset)
        {
            // TODO(avk): Load vertices from...?

            // TODO(avk): Load dataset from...?
            // loaded dataset should be graph? with vertex properties num_coll/num_free

            NearestNeighbors<VertexPtr>::DistanceFunction distanceFunction(
                [this](const VertexConstPtr &a, const VertexConstPtr &b) { return distance(a, b); });
            nn_->setDistanceFunction(distanceFunction);
        }

        double vertexExistence(const VertexPtr& v)
        {
            if (nn_->size() < k_)
                return existencePrior_;

            VertexPtrVector *neighbors;
            nn_->nearestK(v, k_, *neighbors);

            double alpha = 1;
            double beta = 1;
            for (std::size_t i = 0; i < k_; ++i)
            {
                double weight = exp(-obstacleDensity_ * distance(v, neighbors[i]));
                alpha += weight * numFree_[neighbors[i]];
                beta  += weight * numColl_[neighbors[i]];
            }
            return alpha / (alpha + beta);
        }

        double edgeExistence(const VertexPtr& u, const VertexPtr& v)
        {
            // TODO(avk): initialize this, e.g.
            // disc = linspace(0, 1, edgeExistenceSparseDiscretization_);
            // s = u + disc * (v - u);
            VertexPtrVector sparseEdgeDiscretization;

            // the OMPL NN seems not to permit vectorized nearest-neighbor
            // queries, so we might need to do this one discretization at a time
            double edgeExistence = 1.0;
            for (const auto& s : sparseEdgeDiscretization)
            {
                // p(edge exists)
                //   = p(all vertices along edge exist)
                //   = min_v p(vertex exists)
                edgeExistence = min(edgeExistence, vertexExistence(s));
            }
            return edgeExistence;
        }
    }
}
