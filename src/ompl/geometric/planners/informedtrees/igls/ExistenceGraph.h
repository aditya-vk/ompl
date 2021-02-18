#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/informedtrees/IGLS.h"
#include "ompl/geometric/planners/informedtrees/igls/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        class IGLS::ExistenceGraph
        {
        public:
            ExistenceGraph(
                // take a vector of vertices here
                const std::string &datasetPath, double edgeDiscretization, double obstacleDensity);

            /** \brief Computes the probability of a vertex being collision-free. */
            double vertexExistence(const VertexPtr &v) const;
            double stateExistence(const ompl::base::State *state) const;

            /** \brief Computes the probability of an edge being collision-free. */
            double edgeExistence(const VertexPtr &u, const VertexPtr &v) const;
            double edgeExistence(const ompl::base::State *source, const ompl::base::State *target) const;

            /** \brief Setup existence graph datastrcucture */
            void setup(const ompl::base::SpaceInformationPtr &spaceInformation, CostHelper *costHelper,
                       SearchQueue *searchQueue, const ompl::base::Planner *plannerPtr);

        protected:
            /** \brief Loads dataset of priors. */
            void loadDataset();

        private:
            double distance(const VertexPtr &a, const VertexPtr &b) const;

            std::vector<std::vector<double>> readDataFromFile(std::string filename) const;

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr spaceInformation_{nullptr};
            CostHelper *costHelpPtr_{nullptr};
            SearchQueue *queuePtr_{nullptr};

            /** \brief Path to dataset */
            std::string datasetPath_;

            /** \brief Vector indexed by vertex. Number of environments in
             * the dataset where that vertex is free. */
            std::vector<std::size_t> numFree_;

            /** \brief Vector indexed by vertex. Number of environments in
             * the dataset where that vertex is in collision. */
            std::vector<std::size_t> numColl_;

            /** \brief Number of nearest neighbors to estimate existence. */
            std::size_t k_{1u};

            /** \brief Points to use for edge existence discretization. */
            double edgeExistenceSparseDiscretization_;

            /** \brief Tunable parameter representing density of objects. */
            double obstacleDensity_{1u};

            /** \brief Prior existence probability. */
            double existencePrior_{0.5};

            /** \brief Nearest-neighbor structure, containing vertices and their
             * collision probability in the dataset. */
            VertexPtrNNPtr nn_{nullptr};

        };  // class ExistenceGraph
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_
