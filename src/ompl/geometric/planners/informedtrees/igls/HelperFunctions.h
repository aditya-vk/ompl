/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_HELPERFUNCTIONS_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_HELPERFUNCTIONS_

// std::pair
#include <utility>

////////////////////////////////
// Anonymous helpers
namespace
{
    /** \brief Define the addition operator for a std::pair<T,U> from the addition operators of T and U. */
    template <typename T, typename U>
    std::pair<T, U> operator+(const std::pair<T, U> &lhs, const std::pair<T, U> &rhs)
    {
        return std::make_pair(lhs.first + rhs.first, lhs.second + rhs.second);
    };

    /** \brief Delete an iterator in a vector. */
    template <typename V>
    void swapPopBack(typename V::iterator iter, V *vect)
    {
        // Swap to the back if not already there
        if (iter != (vect->end() - 1))
        {
            std::swap(*iter, vect->back());
        }

        // Delete the back of the vector
        vect->pop_back();
    };

    /** \brief A stream operator for an std::array of Cost */
    template <std::size_t SIZE>
    std::ostream &operator<<(std::ostream &out, const std::array<ompl::base::Cost, SIZE> &costArray)
    {
        // Start with a bracket
        out << "(";

        // Iterate through the values
        for (unsigned int i = 0u; i < costArray.size(); ++i)
        {
            // Print value
            out << costArray.at(i);

            // If not last, print a ,
            if (i != costArray.size() - 1u)
            {
                out << ", ";
            }
        }

        // End with a bracket
        out << ")";

        return out;
    };
}  // namespace
////////////////////////////////
#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_IGLS_HELPERFUNCTIONS_
