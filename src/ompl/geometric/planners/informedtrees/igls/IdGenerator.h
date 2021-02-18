/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IDGENERATOR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IDGENERATOR_

#include "ompl/geometric/planners/informedtrees/IGLS.h"

#include <thread>
#include <mutex>
#include <boost/scoped_ptr.hpp>

namespace ompl
{
    namespace geometric
    {
        /** @anchor IdGenerator
        @par Short description
        A class to generate unique IDs for the \ref gVertex "Vertex" class. */

        /** \brief An ID generator class for vertex IDs.*/
        class IGLS::IdGenerator
        {
        public:
            IdGenerator() = default;

            /** \brief Generator a new id and increment the global/static counter of IDs. */
            IGLS::VertexId getNewId()
            {
                // Create a scoped mutex copy of idMutex that unlocks when it goes out of scope:
                std::lock_guard<std::mutex> lockGuard(idMutex_);

                // Return the next id, purposefully post-decrementing:
                return nextId_++;
            }

        private:
            // Variables:
            // The next ID to be returned. We never use 0.
            IGLS::VertexId nextId_{0u};
            // The mutex
            std::mutex idMutex_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_IDGENERATOR_
