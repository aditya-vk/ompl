#include "ompl/geometric/planners/informedtrees/igls/ExistenceGraph.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"
#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"

namespace ompl
{
    namespace geometric
    {
        IGLS::ExistenceGraph::ExistenceGraph(const std::string &datasetPath, std::size_t edgeDiscretization,
                                             double obstacleDensity)
          : datasetPath_(datasetPath)
          , edgeExistenceSparseDiscretization_{edgeDiscretization}
          , obstacleDensity_{obstacleDensity}
        {
            // Do nothing.
        }

        void IGLS::ExistenceGraph::setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                                         CostHelper *costHelper, SearchQueue *queuePtr,
                                         const ompl::base::Planner *plannerPtr)
        {
            spaceInformation_ = spaceInformation;
            costHelpPtr_ = costHelper;
            queuePtr_ = queuePtr;
            if (!static_cast<bool>(nn_))
            {
                nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(plannerPtr));
            }
            NearestNeighbors<VertexPtr>::DistanceFunction distanceFunction(
                [this](const VertexPtr &a, const VertexPtr &b) { return distance(a, b); });
            nn_->setDistanceFunction(distanceFunction);
            loadDataset();
        }

        void IGLS::ExistenceGraph::loadDataset()
        {
            std::vector<std::vector<double>> data = readDataFromFile(datasetPath_);
            std::size_t dimension = spaceInformation_->getStateSpace()->getDimension();
            for (const auto &row : data)
            {
                std::vector<double> position;
                for (int i = 0; i < dimension; ++i)
                {
                    position.push_back(row[i]);
                }
                auto vertex = std::make_shared<Vertex>(spaceInformation_, costHelpPtr_, queuePtr_, nullptr);
                spaceInformation_->getStateSpace()->copyFromReals(vertex->state(), position);
                numFree_.push_back(row.at(dimension));
                numColl_.push_back(row.back());
                nn_->add(vertex);
            }
        }

        double IGLS::ExistenceGraph::vertexExistence(const VertexPtr &v) const
        {
            if (nn_->size() < k_)
            {
                return existencePrior_;
            }
            std::vector<double> p;
            spaceInformation_->getStateSpace()->copyToReals(p, v->state());
            std::cout << "Current position " << p[0] << " " << p[1] << std::endl;
            // std::cin.get();

            VertexPtrVector neighbors;
            nn_->nearestK(v, k_, neighbors);

            double alpha = 1;
            double beta = 1;
            for (std::size_t i = 0; i < k_; ++i)
            {
                double weight = exp(-obstacleDensity_ * distance(v, neighbors[i]));
                spaceInformation_->getStateSpace()->copyToReals(p, neighbors[i]->state());
                if (p[0] == 0.5 && p[1] == 0.5)
                {
                    std::cout << "Nearest Neighbor " << p[0] << " " << p[1] << " " << distance(v, neighbors[i]) << " "
                              << weight << std::endl;
                    std::cin.get();
                }
                // TODO(avk): Maybe this should be a map and not a vector.
                alpha += weight * numFree_[neighbors[i]->getId()];
                beta += weight * numColl_[neighbors[i]->getId()];
            }
            std::cout << "Alpha: " << alpha << " " << beta << std::endl;
            return alpha / (alpha + beta);
        }

        double IGLS::ExistenceGraph::stateExistence(const ompl::base::State *state) const
        {
            auto vertex = std::make_shared<Vertex>(spaceInformation_, costHelpPtr_, queuePtr_, nullptr);
            spaceInformation_->copyState(vertex->state(), state);
            return vertexExistence(vertex);
        }

        double IGLS::ExistenceGraph::edgeExistence(const VertexPtr &u, const VertexPtr &v) const
        {
            // TODO(avk): initialize this, e.g.
            // disc = linspace(0, 1, edgeExistenceSparseDiscretization_);
            // s = u + disc * (v - u);
            VertexPtrVector sparseEdgeDiscretization;

            // the OMPL NN seems not to permit vectorized nearest-neighbor
            // queries, so we might need to do this one discretization at a time
            double edgeExistence = 1.0;
            for (const auto &s : sparseEdgeDiscretization)
            {
                // p(edge exists)
                //   = p(all vertices along edge exist)
                //   = min_v p(vertex exists)
                edgeExistence = std::min(edgeExistence, vertexExistence(s));
            }
            return edgeExistence;
        }

        std::vector<std::vector<double>> IGLS::ExistenceGraph::readDataFromFile(std::string filename) const
        {
            std::ifstream inputFile(filename);
            std::vector<std::vector<double>> configurations;
            if (inputFile)
            {
                while (true)
                {
                    std::string line;
                    double value;

                    std::getline(inputFile, line);

                    std::stringstream ss(line, std::ios_base::out | std::ios_base::in | std::ios_base::binary);

                    if (!inputFile)
                        break;
                    std::vector<double> row;
                    while (ss >> value)
                    {
                        row.emplace_back(value);
                    }
                    configurations.emplace_back(row);
                }
            }
            return configurations;
        }
    }  // namespace geometric
}  // namespace ompl
