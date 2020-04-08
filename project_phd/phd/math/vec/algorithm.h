#ifndef MATH_ALGORITHM
#define MATH_ALGORITHM

#include "../math.h"
#include "../logic/constant.h"
#include <functional>
#include <cmath>
#include <iostream>
#include <fstream>

namespace math {

	class vec; // used before it is defined

MATH_API
bool check_range(const double& op1, const double& op2, const double& opX, bool& flag);
/**< checks if the input magnitude opX is within the range defined by the op1
and op2 input2, assumming that op2 is higher than op1. If yes, returns true and
flag is meaningless. If not, returns false and flag is true if above higher margin,
false if below lower margin. */

MATH_API
int search_binary(const double& op1, const math::vec& vec);
/**< For op1, and given an input vector of magnitudes, it
returns an integer locating the latest position in the vector that is smaller
OR equal to op1. For a size "n" vector, the response range is
[-1,n-1]. If -1, it means magnitude is less than first (0) vector magnitude.
If "n-1" (vector size), it means magnitude is higher or equal than last (n-1)
vector magnitude. Only overloaded for longitudes and bearings, in which the
range is [0,n-1], as being located before the first is analogous to after the
last. */

MATH_API
int search_equispaced(const double& op1, const math::vec& vec, const double& diff);
/**< For op1, and given an input vector of magnitudes and
the difference between consecutive members of the vector, it returns an integer
locating the latest position in the vector that is smaller OR equal to the
op1. For a size "n" vector, the response range is [-1,n-1]. If -1, it
means magnitude is less than first (0) vector magnitude. If "n-1" (vector size),
it means magnitude is higher or equal than last (n-1) vector magnitude. Only
overloaded for longitudes and bearings, in which the range is [0,n-1], as being
located before the first is analogous to after the last.  */

} // closes namespace math

#endif

