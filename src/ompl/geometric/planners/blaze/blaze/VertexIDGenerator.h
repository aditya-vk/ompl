/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXIDGENERATORH_
#define OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXIDGENERATORH_

#include <mutex>
#include <thread>

namespace ompl
{
    namespace geometric
    {
        namespace blaze
        {
            /// Each vertex has its own ID of type int.
            using VertexID = int;

            /// A class to generate unique IDs for the \c Vertex.
            /// \brief An ID generator class for vertex IDs.
            class VertexIDGenerator
            {
            public:
                /// Constructor.
                VertexIDGenerator() = default;

                /// \brief Generates a new ID and increments the global/static counter of IDs.
                VertexID getNewID()
                {
                    std::lock_guard<std::mutex> lockGuard(mMutex);
                    return mNextID++;
                }

            private:
                /// The next ID to be returned. IDs start at 0.
                VertexID mNextID{0u};

                /// The mutex.
                std::mutex mMutex;
            };

        }  // namespace blaze

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_BLAZE_BLAZE_VERTEXIDGENERATORH_
