#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_

#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace geometric
    {

        class IGLS::ExistenceGraph
        {
        public:
            explicit ExistenceGraph(
              // take a vector of vertices here
              const std::string& dataset,
              std::size_t edgeDiscretization,
              double obstacleDensity);

            double vertexExistence(const VertexPtr& v);

            double edgeExistence(const VertexPtr& u, const VertexPtr& v);

        protected:
            void loadDataset(const std::string& dataset);

        private:
            /** \brief Whether the class is setup. */
            bool isSetup_{false};

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr spaceInformation_{nullptr};

            /** \brief Path to dataset? */
            std::string datasetPath_;

            /** \brief Vector(?) indexed by vertex. Number of environments in
             * the dataset where that vertex is free. */
            const std::vector<std::size_t> numFree_;

            /** \brief Vector(?) indexed by vertex. Number of environments in
             * the dataset where that vertex is in collision. */
            const std::vector<std::size_t> numColl_;

            /** \brief Number of nearest neighbors to estimate existence. */
            std::size_t k_{1u};

            /** \brief Points to use for edge existence discretization. */
            std::size_t edgeExistenceSparseDiscretization_{5u};

            /** \brief Tunable parameter representing density of objects. */
            double obstacleDensity_{1u};

            /** \brief Prior existence probability. */
            double existencePrior_{0.5};

            /** \brief Nearest-neighbor structure, containing vertices and their
             * collision probability in the dataset. */
            VertexPtrNNPtr nn_{nullptr};

      }; // class ExistenceGraph
    } // namespace geometric
} //namespace ompl


#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_EXISTENCEGRAPH_
